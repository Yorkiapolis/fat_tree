/*
 * router.cc
 *
 *  Created on: 2016��7��30��
 *      Author: Vincent
 *
 *  Function:
 *      ·���������뻺�棬��ͨ������ͨ���ٲ�(Round Robin)�����濪���ٲ�(Round Robin)�����濪�ش���
 *      ���ݰ���Flit��ʽ��Flit���ȿɵ�(����Head Flit����)
 */


#include <stdio.h>
#include <string.h>
#include <omnetpp.h>
#include <math.h>
#include "fat_tree_m.h"
#include "fat_tree.h"
#include "buffer_info_m.h"
#include "tech_power.h"



using namespace omnetpp;



/**
 * This model is exciting enough so that we can collect some statistics.
 * We'll record in output vectors the hop count of every message upon arrival.
 * Output vectors are written into the omnetpp.vec file and can be visualized
 * with the Plove program.
 *
 * We also collect basic statistics (min, max, mean, std.dev.) and histogram
 * about the hop count which we'll print out at the end of the simulation.
 */
/*************************************************
 * TODO:
 *
 *
 *
 * 1.Switch AllocatorĿǰҲ�ǲ�����ѭ��ʽ���ɷ�Ľ���
 *
 *
 * 2.��ͨ���Ĺ��ܣ�һ��Input channel��2����ͨ��������һ���ٲ�ʤ����������ס�ˣ�����һ��Ӧ�ÿ���ͨ�С������ٲõ�λ��package������������һ��ͨ������Ҫ�Ľ�
 * ����˿��ٲã�����˿����ݵ�������packageΪ��λ������ᷢ�����ݻ�����������ͨ�����ٲò���Ҫ��packageΪ��λ����ÿ����ͨ������һ���Ĵ���
 * �����������ͨ�����package������˿����ĸ���Ȼ����ͨ���ٲ�����Ѳ�����Է�ֹһ����ͨ������ʱ��������ͨ������
 *
 * 3.��ͨ�������⣬��ͨ�������Ƿ�������˿�����һ�£�������Ŷ�Ӧ����˿ڵ�����
 *
 * Feature:
 * 1. Credit-based flow control
 * ***********************************************
 */


// ��Router���н�ģ
class Router : public cSimpleModule
{
  private:
    //long numSent;
    //long numReceived;
    cMessage *selfMsgAlloc; //message�ٲö�ʱ�ź�
    //cMessage *selfMsgBufferInfo; //buffer֪ͨ��ʱ�ź�

    //cLongHistogram hopCountStats;
    //cOutVector hopCountVector;

    //ÿ��Port��buffer״̬
    //bool BufferAvailCurrent[PortNum][VC];//��ǰ·������buffer״̬
    //bool BufferAvailConnect[PortNum][VC];//����·������Ӧ�˿ںŵ���һ��·����buffer״̬
    int BufferConnectCredit[PortNum][VC]; //����·�����˿ڵ�buffer��credit�������л����С

    //Input Buffer, Routing
    FatTreeMsg* VCMsgBuffer[PortNum][VC][BufferDepth]; //virtual channel��buffer,�������յ���Flit��Ϣ
    //Խ�絽�����ݷ���IDС���Ǳߣ��涨��,0��ʾbuffer�е�һ����ȥ������
    //int VCOutPort[PortNum][VC][BufferDepth]; //��ӦVChannelMsg����Ÿ�msg������˿ں�out_port_id

    int VCInportVCID[PortNum]; //���ÿ���˿ڽ����Flit��VCID��VCID��Head Flitȷ��
    //int VCOutport[PortNum][VC]; //���ÿ������˿ڵ�Virtual Channel�Ľ���Crossbar��flit������˿�
    int VCInportFlitCount[PortNum]; //���ÿ������˿�Head Flit��Flit Count�����ڼ�����0��ʾʣ���flitΪ0�����Ѵ���


    //����VC�ٲ�
    int VCAllocWinVCID[PortNum];//���ÿ��portʤ����VChannelMsg��id��
    //FatTreeMsg* VCAllocWinMsg[PortNum];//�����洢ÿ��portʤ����VChannelMsg��msg��ע�⣬����VChannelMsg��msg�Ȳ�ɾ����󷢳�ȥ����ɾ
    int VCAllocWinOutPort[PortNum];//�����洢ÿ��portʤ����vc��msg�����port
    int VCAllocWinFlitCount[PortNum];//���ڴ洢ÿ��Portʤ����VC��Flit Count

    //����SA�ٲ�
    //FatTreeMsg* SAllocWinMsg[PortNum];//�洢ÿ������˿�Switch Allocator�ٲ���ʤ������˿ں����msg��PortNum������˿���ҪPortNum���ٲ�
    int SAllocWinInPort[PortNum];//�洢ÿ������˿�Switch Allocator�ٲ���ʤ������˿ںţ������������˿ںţ�����VC��ʤ����vcid
    int SAllocFlitCount[PortNum];//�������˿�һ��packge��flit count��ͬʱ�����ж�����˿��Ƿ�ռ��
    int SAllocNextVCID[PortNum];//�������˿�Ҫ���Flit����һ��Router��VCID

    double RouterPower;

    double flitReceived; //���ڼ���toggle rate





  public:
    Router();
    virtual ~Router();
  protected:
    //virtual FatTreeMsg *generateMessage();
    virtual void forwardMessage(FatTreeMsg *msg, int out_port_id);
    virtual void forwardBufferInfoMsg(BufferInfoMsg *msg, int out_port_id);
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    virtual int ppid2plid(int ppid);
    virtual int plid2ppid(int plid);
    virtual int swpid2swlid(int swpid);
    virtual int swlid2swpid(int swlid);
    virtual int calRoutePort(FatTreeMsg *msg);
    virtual int getNextRouterPort(int current_out_port); //������һ��������router�Ķ˿�
    virtual int getNextRouterAvailVCID(int port_num);//������һ���ڵ���Ӧ�˿ڿ��õ�virtual channel
    virtual bool connectToProcessor(int port_num);
    virtual double getRouterPower();//����·��������


    // The finish() function is called by OMNeT++ at the end of the simulation:
    virtual void finish() override;
};

Define_Module(Router);

Router::Router(){
    selfMsgAlloc=nullptr;
    //selfMsgBufferInfo=nullptr;
}


Router::~Router(){
    cancelAndDelete(selfMsgAlloc);
    //cancelAndDelete(selfMsgBufferInfo);
}

void Router::initialize()
{
    // Initialize variables
    //numSent = 0;
    //numReceived = 0;
    //��ʼ���к��еĲ���
    //WATCH(numSent);
    //WATCH(numReceived);

    //hopCountStats.setName("hopCountStats");
    //hopCountStats.setRangeAutoUpper(0, 10, 1.5);
    //hopCountVector.setName("HopCount");

    //��Buffer���г�ʼ��
    for(int i=0;i<PortNum;i++)
        for(int j=0;j<VC;j++)
            for(int k=0;k<BufferDepth;k++){
                VCMsgBuffer[i][j][k]=nullptr;
                //VCOutPort[i][j][k]=-1;
            }

    for(int i=0;i<PortNum;i++){
        VCInportVCID[i]=-1;
        VCInportFlitCount[i]=0;
        //for(int j=0;j<VC;j++){
        //    VCOutport[i][j]=-1;
        //}
    }

    //��VC�ٲ������г�ʼ��
    for(int i=0;i<PortNum;i++){
        VCAllocWinVCID[i]=-1;
        //VCAllocWinMsg[i]=nullptr;
        VCAllocWinOutPort[i]=-1;
        VCAllocWinFlitCount[i]=0;
    }

    //��SA�ٲ������г�ʼ��
    for(int i=0;i<PortNum;i++){
        //SAllocWinMsg[i]=nullptr;
        SAllocWinInPort[i]=-1;
        SAllocFlitCount[i]=0;
        SAllocNextVCID[i]=-1;
    }

    //��selfMsg���г�ʼ��
    selfMsgAlloc = new cMessage("selfMsgAlloc");
    scheduleAt(Sim_Start_Time, selfMsgAlloc);
    //selfMsgBufferInfo = new cMessage("selfMsgBufferInfo");
    //scheduleAt(Buffer_Info_Sim_Start, selfMsgBufferInfo);

    for(int i=0;i<PortNum;i++){
        for(int j=0;j<VC;j++){
            //BufferAvailCurrent[i][j]=true;
            //BufferAvailConnect[i][j]=false;
            BufferConnectCredit[i][j] = BufferDepth;
        }
    }

    RouterPower = 0;
    flitReceived = 0;

}

void Router::handleMessage(cMessage *msg)
{

    if (msg->isSelfMessage()) {
        //****************�ٲö�ʱ****************************

        if(msg==selfMsgAlloc){//����ϢΪ�ٲö�ʱ��Ϣ

            scheduleAt(simTime()+CLK_CYCLE,selfMsgAlloc);


            // Step 3 ��ͨ���ٲ� Virtual Channel Allocation
            //��ͨ���ٲò���Round Robin��ѭ�����㷨
            //����VCAllocWinVCID������ָ��VC����ͨ��
            //��ÿ���˿ڶ������о�
            for(int i=0;i<PortNum;i++){
                //�����һ���ٲ�ʤ����Virtual Channel��Flit��û�����꣬��VCAllocWinFlitCount��Ϊ0����ô���������ٲã�������һ���ٲý��
                if(VCAllocWinFlitCount[i] == 0){
                    int vcid=(VCAllocWinVCID[i]+1)%VC;//port i��VChannelMsgָ�룬����һ���ٲóɹ���vcid����һ��vcid��ʼ
                    for(int j=0;j<VC;j++){
                        int vcid_tmp=(vcid+j)%VC;
                        if(VCMsgBuffer[i][vcid_tmp][0]!=nullptr){ //һ����Head Flit����������Ƕ����Ļ�
                            if(VCMsgBuffer[i][vcid_tmp][0]->getIsHead() == true){
                                //VCAllocWinMsg[i]=VCMsgBuffer[i][vcid_tmp][0];//�ȴ��Ų�ɾ��������ȥ����ɾ
                                VCAllocWinVCID[i]=vcid_tmp;//�����ٲ�ʤ����vcid

                                // Step 2 ·�ɼ��� Routing Computation, ��VC�ٲ�ʤ��������·�ɼ��㣬��ʡ�洢��Դ
                                // ���flitһ����vc�ͽ���·�ɼ��㣬��1��vc��������ж��packet����ô����Ҫ����Ĵ���
                                // ������·�ɼ���Ľ�����෴�������vc�ĳ��ڽ���·�ɼ��㣬��ôֻ��Ҫ1���Ĵ������ܱ�������
                                // �����ͨ����������������˿���ȷ����ÿ����ͨ����Ӧ����Ķ˿ںţ���ô��Ҫһ����ͽ���·�ɼ��㣬
                                // ȷ������˿ں�
                                VCAllocWinOutPort[i]=calRoutePort(VCMsgBuffer[i][vcid_tmp][0]);//�洢ʤ����vc��msg������˿�
                                VCAllocWinFlitCount[i]=VCMsgBuffer[i][vcid_tmp][0]->getFlitCount();//ȡ��Flit Count
                                if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
                                    EV<<"VC Allocation >> ROUTER: "<<getIndex()<<"("<<swpid2swlid(getIndex())<<"), INPORT: "<<i<<
                                            ", VCAllocWinVCID: "<<vcid_tmp<<", VCAllocWinOutPort: "<<VCAllocWinOutPort[i]<<
                                            ", Flit Count: "<<VCAllocWinFlitCount[i]<<", Win Msg: { "<<VCMsgBuffer[i][vcid_tmp][0]<<" }\n";
                                }
                                break;
                            }

                        }
                    }
                }
            }




            //Step 4 ���濪���ٲ� Switch Allocation
            //�ڶ��׶ε��ٲ���Ҫ�ǶԴ�ÿ������˿ڹ�������ͬһ����˿��������������ٲá�
            //��Ȼ��Щ�������������ĳһʤ��������ͨ�������ġ���Ϊ��P������˿ڣ������������ٲ���ҪP�����롣
            //���ﲻ��crossbar��ʵ�֣���forѭ������crossbar
            for(int i=0;i<PortNum;i++){//��ÿһ������˿�SAlloc����ѭ��
                if(SAllocFlitCount[i] == 0){//����˿ڻ�δ��ռ��
                    int last_win_inport=(SAllocWinInPort[i]+1)%PortNum;//����һ���ٲ�ʤ���Ķ˿ڵ���һ���˿ڿ�ʼ������ѭ�ٲ�
                    for(int j=0;j<PortNum;j++){//PortNum������˿�
                        int inport_tmp=(last_win_inport+j)%PortNum;
                        if(VCAllocWinOutPort[inport_tmp] == i
                                && VCMsgBuffer[inport_tmp][VCAllocWinVCID[inport_tmp]][0] != nullptr){
                            //���inport�ٲ�ʤ����vc�����port��outport��idһ�£����ٲ�ʤ��,ͬʱ��Ҫ�ж�VCAllocWinOutPort�����
                            //��һ���ٲ�ʤ����package���Ǳ����ٲ�ʤ����package�����Ϊ��һ�Σ���VCMsgBuffer��Ӧ��bufferΪnullptr����������˿�

                            int vc_id=getNextRouterAvailVCID(i);//������һ��·������buffer��vcid�����򷵻�-1
                            if(vc_id != -1){ //�¸�router��Ӧ�˿ڵ�vc����buffer�������
                                //SAllocWinMsg[i]=VCAllocWinMsg[inport_tmp];
                                VCMsgBuffer[inport_tmp][VCAllocWinVCID[inport_tmp]][0]->setVc_id(vc_id);//������һ���ڵ��vcid
                                SAllocNextVCID[i] = vc_id;
                                SAllocWinInPort[i]=inport_tmp;
                                SAllocFlitCount[i]=VCAllocWinFlitCount[inport_tmp];
                                if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
                                    EV<<"Switch Allocation >> ROUTER: "<<getIndex()<<"("<<swpid2swlid(getIndex())<<"), OUTPORT: "<<i<<
                                            ", SAllocWinInPort: "<<inport_tmp<<", VCAllocWinVCID: "<<VCAllocWinVCID[inport_tmp]<<
                                            ", SAllocNextVCID: "<<SAllocNextVCID[i]<<", SAllocFlitCount: "<<SAllocFlitCount[i]<<
                                            ", SAlloc Win Msg: { "<<VCMsgBuffer[inport_tmp][VCAllocWinVCID[inport_tmp]][0]<<" }\n";
                                }
                                break;//break����ѭ��
                            }

                        }
                    }
                }
            }



            //Step 5 ���濪�ش��� Switch Traversal
            //��ÿһ������˿ڶ�ת����msg��ͬʱ�Դ�����ת����msg�������
            for(int i=0;i<PortNum;i++){//��ÿһ������˿ڽ���ת��
                if(SAllocFlitCount[i] != 0){//����˿�������
                    //���ж���һ��Router��Ӧ�˿ڶ�Ӧvc�Ƿ���buffer�����flitһ��ʱ���п���ǰ�漸��flit������buffer
                    int input_port = SAllocWinInPort[i];
                    int win_vcid = VCAllocWinVCID[input_port];
                    int nextVCID = SAllocNextVCID[i];
                    //�ж��¸�·�����Ƿ��пռ���ܴ�Flit
                    if(BufferConnectCredit[i][nextVCID] != 0){
                        FatTreeMsg* forward_msg=VCMsgBuffer[input_port][win_vcid][0];
                        int outport = VCAllocWinOutPort[input_port]; //outputӦ�õ���i
                        if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
                            EV<<"Switch Traversal >> ROUTER: "<<getIndex()<<"("<<swpid2swlid(getIndex())<<"), OUTPORT: "<<i<<
                                    ", SAllocWinInPort: "<<input_port<<", VCAllocWinVCID: "<<win_vcid<<
                                    ", nextVCID: "<<nextVCID<<", SAllocFlitCount: "<<SAllocFlitCount[i]<<
                                    ", VCMsgBuffer: { "
                                    <<forward_msg<<" }, VCAllocWinOutPort: "<<outport<<'\n';
                        }
                        //ת��msg
                        forwardMessage(forward_msg,outport); //����router����sink message, ֱ��ת������

                        //�����Ӧbuffer
                        //SAllocWinMsg[i]=nullptr;
                        //VCAllocWinMsg[inport]=nullptr;

                        //���¼Ĵ�����Ϣ
                        VCAllocWinFlitCount[input_port]--;
                        SAllocFlitCount[i]--;

                        //�����뻺��Buffer����shift
                        for(int j=0;j<BufferDepth-1;j++){
                            VCMsgBuffer[input_port][win_vcid][j]=VCMsgBuffer[input_port][win_vcid][j+1];
                            //VCOutPort[inport][vcid][j]=VCOutPort[inport][vcid][j+1];
                        }
                        VCMsgBuffer[input_port][win_vcid][BufferDepth-1]=nullptr;
                        //VCOutPort[inport][vcid][BufferDepth-1]=-1;

                        //�ж�·�����Ƿ��processor�����������������Ҫdecrement credit����Ϊprocessor�Ľ������������޵�
                        if(!connectToProcessor(outport)){ //��·��������
                            BufferConnectCredit[i][nextVCID]--;
                        }
                        if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
                            EV<<"BufferConnectCredit["<<i<<"]["<<nextVCID<<"]="<<BufferConnectCredit[i][nextVCID]<<"\n";
                        }


                        //ÿת��input buffer�����һ��flit���Ͳ���һ�������źţ�֪ͨ����router������increment credit����
                        int from_port = getNextRouterPort(input_port);
                        BufferInfoMsg *bufferInfoMsg = new BufferInfoMsg("bufferInfoMsg");
                        bufferInfoMsg->setFrom_port(from_port);
                        bufferInfoMsg->setVcid(win_vcid);
                        //����bufferInfoMsg
                        forwardBufferInfoMsg(bufferInfoMsg, input_port);

                    }

                }

            }
        }


    }else{ //������Ϣ�����յ�����·��������Ϣ
        //*************************�յ������˿�buffer������Ϣ************************
        if(strcmp("bufferInfoMsg", msg->getName()) == 0){
            //�յ�����ϢΪbuffer״̬��Ϣ������BufferAvailConnect[PortNum][VC]

            BufferInfoMsg *bufferInfoMsg = check_and_cast<BufferInfoMsg *>(msg);
            int from_port=bufferInfoMsg->getFrom_port();
            //����BufferAvailConnect[PortNum][VC]
            //for(int j=0;j<VC;j++){
            //    BufferAvailConnect[from_port][j]=bufferInfoMsg->getBufferAvail(j);
            //}
            int vcid = bufferInfoMsg->getVcid();
            BufferConnectCredit[from_port][vcid]++;

            if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
                EV<<"Receiving bufferInfoMsg >> ROUTER: "<<getIndex()<<"("<<swpid2swlid(getIndex())<<"), INPORT: "<<from_port<<
                    ", Received MSG: { "<<bufferInfoMsg<<" }\n";
                EV<<"BufferConnectCredit["<<from_port<<"]["<<vcid<<"]="<<BufferConnectCredit[from_port][vcid]<<"\n";
            }
            delete bufferInfoMsg;


        }else{
            //**********************�յ������˿ڵ�fatTreeMsg������Ϣ*******************
            //�յ�����ϢΪFatTreeMsg������Ϣ

            FatTreeMsg *ftmsg = check_and_cast<FatTreeMsg *>(msg);
            flitReceived += 1;

            //Step 1 ����洢 Input Buffer�������ŵ��ĸ�virtual channel
            //�ж��Ƿ�ΪHead Flit
            if(ftmsg->getIsHead() == true){
                int input_port = ftmsg->getFrom_router_port();
                int vc_id = ftmsg->getVc_id();
                int flit_count = ftmsg->getFlitCount();
                VCInportVCID[input_port] = vc_id;
                VCInportFlitCount[input_port] = flit_count -1; //��ȥHead Flit��ʣ��ĸ���
                if(VCMsgBuffer[input_port][vc_id][BufferDepth-1]==nullptr){
                    for(int i=0;i<BufferDepth;i++){
                        if(VCMsgBuffer[input_port][vc_id][i]==nullptr){
                            VCMsgBuffer[input_port][vc_id][i]=ftmsg;
                            break;
                        }
                    }
                    if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
                        EV<<"Input Buffer >> ROUTER: "<<getIndex()<<"("<<swpid2swlid(getIndex())<<"), INPORT: "<<input_port<<
                            ", VCID: "<<vc_id<<", Remaining Flit Count: "<<VCInportFlitCount[input_port]<<
                            ", Received MSG: { "<<ftmsg<<" }\n";
                    }
                }else{
                    //������һ����router������routerת��flitʱ�Ѿ�ȷ����buffer����˲�����뵽�����������
                    if (Verbose >= VERBOSE_DEBUG_MESSAGES){
                        EV<<"Input Buffer and Routing >> ROUTER: "<<getIndex()<<"("<<swpid2swlid(getIndex())<<"), INPORT: "<<input_port<<
                            ", VCID: "<<vc_id<<", Received Head MSG: { "<<ftmsg<<" }, Buffer is full, dropping message\n";
                    }

                }

            }else{ //Body/Tail Flit
                int input_port = ftmsg->getFrom_router_port();
                int vc_id = VCInportVCID[input_port];
                VCInportFlitCount[input_port]--;

                if(VCMsgBuffer[input_port][vc_id][BufferDepth-1]==nullptr){
                    for(int i=0;i<BufferDepth;i++){
                        if(VCMsgBuffer[input_port][vc_id][i]==nullptr){
                            VCMsgBuffer[input_port][vc_id][i]=ftmsg;
                            break;
                        }
                    }
                    if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
                        EV<<"Input Buffer >> ROUTER: "<<getIndex()<<"("<<swpid2swlid(getIndex())<<"), INPORT: "<<input_port<<
                            ", VCID: "<<vc_id<<", Remaining Flit Count: "<<VCInportFlitCount[input_port]<<
                            ", Received MSG: { "<<ftmsg<<" }\n";
                    }
                }else{
                    //������һ����router������routerת��flitʱ�Ѿ�ȷ����buffer����˲�����뵽�����������
                    if (Verbose >= VERBOSE_DEBUG_MESSAGES){
                        EV<<"Input Buffer and Routing >> ROUTER: "<<getIndex()<<"("<<swpid2swlid(getIndex())<<"), INPORT: "<<input_port<<
                            ", VCID: "<<vc_id<<", Received Body/Tail MSG: { "<<ftmsg<<" }, Buffer is full, dropping message\n";
                    }

                }


            }

            //2 ·�ɼ���ŵ�Virtual Channel Allocation�������
            /*
            int vc_id = ftmsg->getVc_id(); // ��ȡvc id�� ��̬��ʽ����vc id
            int port_id = ftmsg->getFrom_router_port();//��ȡ�յ�msg�Ķ˿ںţ������˷����ĸ��˿ڵ�vc��
            int avil_buff_id=-1;
            //�ж�buffer�Ƿ��пռ䣬���buffer���пռ䣬���յ����źŴ浽buffer��ȥ




            // 2 ·�ɼ��� Routing Computation
            int out_port_id=calRoutePort(ftmsg);//���㷢��msg�Ķ˿ں�
            if(avil_buff_id!=-1){
                VCOutPort[port_id][vc_id][avil_buff_id]=out_port_id;
                if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
                    EV<<"Input Buffer and Routing >> ROUTER: "<<getIndex()<<"("<<swpid2swlid(getIndex())<<"), INPORT: "<<port_id<<
                        ", VCID: "<<vc_id<<", Received MSG: { "<<ftmsg<<" }, OUTPORT: "<<out_port_id<<'\n';
                }
            }
            */
        }

    }

}


//����...����·�ɶ˿�
void Router::forwardMessage(FatTreeMsg *msg, int out_port_id)
{

    // Increment hop count.
    msg->setHopCount(msg->getHopCount()+1);

    //int k=calRoutePort(msg);//���㷢��msg�Ķ˿ں�
    int k = out_port_id;
    char str1[20]="port_";
    char str2[20];
    sprintf(str2, "%d", k);
    strcat(str1,str2);
    strcat(str1,"$o");
    //EV<<"k="<<k<<" str1="<<str1<<" str2="<<str2<<"\n";
    msg->setFrom_router_port(getNextRouterPort(k));//���ý��ܸ�msg��Router��port�˿ں�
    send(msg,str1);
    int cur_swpid=getIndex();//��ǰ·������id
    int cur_swlid=swpid2swlid(cur_swpid);
    if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
        EV << "Forwarding message { " << msg << " } from router "<<cur_swpid<<"("<<cur_swlid<<")"<< " through port "<<k<<"\n";
    }

}

void Router::forwardBufferInfoMsg(BufferInfoMsg *msg, int out_port_id){

    int k = out_port_id;
    char str1[20]="port_";
    char str2[20];
    sprintf(str2, "%d", k);
    strcat(str1,str2);
    strcat(str1,"$o");
    //EV<<"k="<<k<<" str1="<<str1<<" str2="<<str2<<"\n";
    msg->setFrom_port(getNextRouterPort(k));//���ý��ܸ�msg��Router��port�˿ں�
    send(msg,str1);
    int cur_swpid=getIndex();//��ǰ·������id
    int cur_swlid=swpid2swlid(cur_swpid);
    if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
        EV << "Forwarding BufferInfoMsg { " << msg << " } from router "<<cur_swpid<<"("<<cur_swlid<<")"<< " through port "<<k<<"\n";
    }

}

//��ppid����plid
int Router::ppid2plid(int ppid){
    int idtmp=ppid;
    int idfinal=0;
    int mul=1;
    for(int i=0;i<LevelNum-1;i++){
        idfinal=idfinal+idtmp%(PortNum/2)*mul;
        mul=mul*10;
        idtmp=(int)(idtmp/(PortNum/2));
    }
    idfinal=idfinal+idtmp*mul;
    return idfinal;
}
//��plid����ppid
int Router::plid2ppid(int plid){
    int tmp=plid;
    int mul=1;
    int IDtmp=0;
    for(int i=0;i<LevelNum;i++){
        IDtmp=IDtmp+mul*(tmp%10);
        mul=mul*(PortNum/2);
        tmp=tmp/10;
    }
    return IDtmp;
}

//��swpid����swlid
int Router::swpid2swlid(int swpid){
    //�����ж�swpid����һ��
    int level=0;
    bool find_level=false;
    for(int i=0;i<LevelNum-1;i++){
        if(swpid>=i*SwLowEach and swpid<(i+1)*SwLowEach){
            level=i;
            find_level=true;
            break;
        }
    }
    if(!find_level)
        level=LevelNum-1;
    //�Ѿ��ҵ�switch���ڲ㣬������������б���
    //�ȶԷǶ����switch���б���
    if(level<LevelNum-1){
        int tmp=swpid-level*SwLowEach;
        int IDtmp=0;
        int mul=1;
        for(int i=0;i<LevelNum-2;i++){
            IDtmp=mul*(tmp%(PortNum/2))+IDtmp;
            tmp=(int)(tmp/(PortNum/2));
            mul=mul*10;
        }
        IDtmp=IDtmp+mul*tmp;
        mul=mul*10;
        IDtmp=mul*level+IDtmp;//��ǰ��������Ĳ���
        return IDtmp;
    }
    //�������Զ����switch���в���
    else{
        int tmp=swpid;
        int IDtmp=0;
        int mul=1;
        for(int i=0;i<LevelNum-1;i++){
            IDtmp=mul*(tmp%(PortNum/2))+IDtmp;
            tmp=(int)(tmp/(PortNum/2));
            mul=mul*10;
        }
        IDtmp=mul*level+IDtmp;
        return IDtmp;
    }
}

//swlidתswpid
int Router::swlid2swpid(int swlid){
    int tmp=swlid;
    int level=tmp/(pow(10,(LevelNum-1)));
    tmp=tmp%((int)pow(10,(LevelNum-1)));
    int IDtmp=level*SwLowEach;
    int mul=1;
    for(int i=0;i<LevelNum-1;i++){
        IDtmp=IDtmp+mul*(tmp%10);
        mul=mul*(PortNum/2);
        tmp=tmp/10;
    }
    return IDtmp;
}

//���ݵ�ǰrouter��swpid��msg��dst_ppid������ת���Ķ˿�
int Router::calRoutePort(FatTreeMsg *msg){
    int cur_swpid=getIndex();//��ǰ·������id
    int cur_swlid=swpid2swlid(cur_swpid);
    int level=cur_swlid/pow(10,LevelNum-1);//Router��level
    int dst_ppid=msg->getDst_ppid();
    int dst_plid=ppid2plid(dst_ppid);
    //EV<<dst_ppid<<" "<<dst_plid<<"\n";
    //�ж�switch�Ƿ�Ϊ����
    int ptmp=dst_plid/pow(10,level+1);//
    int ctmp=(cur_swlid%((int)pow(10,LevelNum-1)))/pow(10,level);
    bool isAncestor=(ptmp==ctmp);
    int k;//ת���Ķ˿�
    //EV<<cur_swpid<<" "<<cur_swlid<<" "<<level<<" "<<dst_ppid<<" "<<dst_plid<<" "<<ptmp<<" "<<ctmp<<"\n";
    //���switch��dst_ppid�����ȣ������¶˿�ת�����������϶˿�ת��
    if(isAncestor){
        //����ת��
        k=(dst_plid/((int)pow(10,level)))%10;//ͨ���˿�pl������ת��
        //EV<<"isAncestor, k="<<k<<"\n";
        return k;
    }else{
        //����ת��
        k=(dst_plid/((int)pow(10,level)))%10+PortNum/2;//k=pl��+m/2
        //EV<<"notAncestor, k="<<k<<" "<<dst_ppid<<" "<<dst_plid<<" "<<(int)pow(10,level)<<" "<<(dst_plid/((int)pow(10,level)))%10<<"\n";
        return k;
    }

}

//�����յ���msg��·�����Ķ˿ں�
int Router::getNextRouterPort(int current_out_port){


    int cur_swpid=getIndex();//��ǰ·������id
    int cur_swlid=swpid2swlid(cur_swpid);
    int level=cur_swlid/pow(10,LevelNum-1);//Router��level
    bool lowerRouter = (current_out_port>=(PortNum/2))&&(level!=LevelNum-1); //�ж��Ƿ�����ת��������ת����Ϊ�²�router

    int ctmp=(cur_swlid%((int)pow(10,LevelNum-1)));//ȥ����level��swlid
    int k;//��һ��Router�Ľ��ܶ˿�
    if(!lowerRouter){//�ϲ��Router
        if(level==0){
            k=0;//level==0ʱ��Ϊ�϶˿ڣ����msg����processor����processorֻ��һ���˿ڣ�Ĭ��processor�Ľ��ܶ˿�Ϊ0
        }else{
            int lowLevel=level-1;//�²��level
            k = (ctmp/((int) pow(10,lowLevel)))%10+PortNum/2;
        }

    }else{ //�²��Router
        k = (ctmp/((int) pow(10,level)))%10;
    }
    return k;
}

bool Router::connectToProcessor(int port_num) {
    int cur_swpid=getIndex();//��ǰ·������id
    int cur_swlid=swpid2swlid(cur_swpid);
    int level=cur_swlid/pow(10,LevelNum-1);//Router��level
    if (level == 0 && port_num < PortNum/2)
        return true;
    return false;
}


int Router::getNextRouterAvailVCID(int port_num){
    int vc_id = intuniform(0,VC-1); //�������һ��ͨ�����Դ�vc��ʼѭ���ж��Ƿ��пյ�vc����������ֲ�����ֹÿ�ζ���0��ʼ
    for(int i=0;i<VC;i++){
        int vcid_tmp = (vc_id + i)%VC;
        if(BufferConnectCredit[port_num][vcid_tmp] != 0){
            return vcid_tmp;
        }
    }
    return -1;

}
double Router::getRouterPower() {

    double timeCount = simTime().dbl() - Sim_Start_Time;
    double clockCount = timeCount / CLK_CYCLE; //ʱ��������
    double TR = flitReceived / (PortNum * clockCount);
    //recordScalar("TR", );
    //instances
    int XBAR_insts = PortNum * PortNum * FlitWidth;
    int SWVC_insts = 9 * ((pow(PortNum,2) * VC * VC) + pow(PortNum,2) + (PortNum * VC) - PortNum);
    int INBUF_insts = 180 * PortNum * VC + 2 * PortNum * VC * BufferDepth * FlitWidth + 2* PortNum *
            PortNum * VC * BufferDepth + 3 * PortNum * VC * BufferDepth + 5 * PortNum * PortNum
            * BufferDepth + PortNum * PortNum + PortNum * FlitWidth + 15 * PortNum;
    int OUTBUF_insts = 25 * PortNum + 80 * PortNum * VC;
    int CLKCTRL_insts = 0.02 * (SWVC_insts + INBUF_insts + OUTBUF_insts);

    //leakage power
    double XBAR_leakage_power = MUX2_leak_nW * XBAR_insts;
    double SWVC_leakage_power = ((6*NOR_leak_nW + 2*INV_leak_nW + DFF_leak_nW)/9)* SWVC_insts;
    double INBUF_leakage_power = ((AOI_leak_nW + DFF_leak_nW)/2) * INBUF_insts;
    double OUTBUF_leakage_power = ((AOI_leak_nW + DFF_leak_nW)/2) * OUTBUF_insts;
    double CLKCTRL_leakage_power = ((AOI_leak_nW + INV_leak_nW)/2) * CLKCTRL_insts;

    //internal power
    double XBAR_internal_power = MUX2_int_J * TR * XBAR_insts;
    double SWVC_internal_power = (6*NOR_int_J + 2*INV_int_J + DFF_int_J) * TR * SWVC_insts;
    double INBUF_internal_power = (AOI_int_J + DFF_int_J) * .5 * (INBUF_insts * TR + .05 * INBUF_insts);
    double OUTBUF_internal_power = (AOI_int_J + DFF_int_J) * .5 * (OUTBUF_insts * TR + .05 * OUTBUF_insts);
    double CLKCTRL_internal_power = (AOI_int_J + INV_int_J) * CLKCTRL_insts * TR;

    //switching power
    double XBAR_switching_power = 0.5 * 1.4 * MUX2_load_pF * VDD * VDD * TR * FREQ;
    double SWVC_switching_power = 0.5 *1.4 * (NOR_load_pF + INV_load_pF + DFF_load_pF) * VDD *  VDD * FREQ * SWVC_insts * TR;
    double INBUF_switching_power = 0.5 *1.4 * VDD * VDD * FREQ * .5 * (INBUF_insts * TR * AOI_load_pF + .05 * INBUF_insts * DFF_load_pF);
    double OUTBUF_switching_power = 0.5 *1.4 * VDD * VDD * FREQ * .5 * (OUTBUF_insts * TR * AOI_load_pF + .05 * OUTBUF_insts * DFF_load_pF);
    double CLKCTRL_switching_power = .5 * 1.4 *(INV_load_pF + AOI_load_pF) * VDD * VDD * FREQ * CLKCTRL_insts * TR;

    double p_leakage = 1e-6 * (XBAR_leakage_power + SWVC_leakage_power + INBUF_leakage_power + OUTBUF_leakage_power + CLKCTRL_leakage_power);
    double p_internal = XBAR_internal_power + SWVC_internal_power + INBUF_internal_power + OUTBUF_internal_power + CLKCTRL_internal_power;
    double p_switching = 1e-9 * (XBAR_switching_power + SWVC_switching_power + INBUF_switching_power + OUTBUF_switching_power + CLKCTRL_switching_power);

    double p_total = p_leakage + p_internal + p_switching;

    return p_total;


}
void Router::finish()
{
    // This function is called by OMNeT++ at the end of the simulation.
    //EV << "Sent:     " << numSent << endl;
    //EV << "Received: " << numReceived << endl;
    //EV << "Hop count, min:    " << hopCountStats.getMin() << endl;
    //EV << "Hop count, max:    " << hopCountStats.getMax() << endl;
    //EV << "Hop count, mean:   " << hopCountStats.getMean() << endl;
    //EV << "Hop count, stddev: " << hopCountStats.getStddev() << endl;

    //recordScalar("#sent", numSent);
    //recordScalar("#received", numReceived);

    //hopCountStats.recordAs("hop count");
    double routerPower = getRouterPower();
    EV <<"Router power: " << routerPower <<endl;
    recordScalar("router power", routerPower);
}

