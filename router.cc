/*
 * router.cc
 *
 *  Created on: 2016��7��30��
 *      Author: Vincent
 */


#include <stdio.h>
#include <string.h>
#include <omnetpp.h>
#include <math.h>
#include "fat_tree_m.h"
#include "fat_tree.h"
#include "buffer_info_m.h"



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
 * msg������ͨ����̬�ķ�ʽѡ��virtual channel�����ԸĽ�
 *
 * Switch AllocatorĿǰҲ�ǲ�����ѭ��ʽ���ɷ�Ľ���
 *
 * flitע�뷽ʽ�о�һ��
 *
 * Ŀǰ�ǵ����źŰ������һ�������flit�����ƣ�
 * ***********************************************
 */

// ��Router���н�ģ
class Router : public cSimpleModule
{
  private:
    //long numSent;
    //long numReceived;
    cMessage *selfMsgAlloc;
    cMessage *selfMsgBufferInfo;

    cLongHistogram hopCountStats;
    cOutVector hopCountVector;

    //ÿ��Port��buffer״̬
    bool BufferAvailCurrent[PortNum][VC];//��ǰ·������buffer״̬
    bool BufferAvailConnect[PortNum][VC];//����·������Ӧ�˿ںŵ�buffer״̬

    //Input Buffer, Routing
    FatTreeMsg* VChannelMsg[PortNum][VC][BufferDepth]; //virtual channel ��buffer
    //Խ�絽�����ݷ���IDС���Ǳߣ��涨��,0��ʾbuffer�е�һ����ȥ������
    int VCOutPort[PortNum][VC][BufferDepth]; //��ӦVChannelMsg����Ÿ�msg������˿ں�out_port_id

    //����VC�ٲ�
    int VCAllocWinVCID[PortNum];//���ÿ��portʤ����VChannelMsg��id��
    FatTreeMsg* VCAllocWinMsg[PortNum];//�����洢ÿ��portʤ����VChannelMsg��msg��ע�⣬����VChannelMsg��msg�Ȳ�ɾ����󷢳�ȥ����ɾ
    int VCAllocWinOutPort[PortNum];//�����洢ÿ��portʤ����vc��msg�����port

    //����SA�ٲ�
    FatTreeMsg* SAllocWinMsg[PortNum];//�洢ÿ������˿�Switch Allocator�ٲ���ʤ������˿ں����msg��PortNum������˿���ҪPortNum���ٲ�
    int SAllocWinInPort[PortNum];//�洢ÿ������˿�Switch Allocator�ٲ���ʤ������˿ںţ������������˿ںţ�����VC��ʤ����vcid





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


    // The finish() function is called by OMNeT++ at the end of the simulation:
    virtual void finish() override;
};

Define_Module(Router);

Router::Router(){
    selfMsgAlloc=nullptr;
    selfMsgBufferInfo=nullptr;
}


Router::~Router(){
    cancelAndDelete(selfMsgAlloc);
    cancelAndDelete(selfMsgBufferInfo);
}

void Router::initialize()
{
    // Initialize variables
    //numSent = 0;
    //numReceived = 0;
    //��ʼ���к��еĲ���
    //WATCH(numSent);
    //WATCH(numReceived);

    hopCountStats.setName("hopCountStats");
    hopCountStats.setRangeAutoUpper(0, 10, 1.5);
    hopCountVector.setName("HopCount");

    //��Buffer���г�ʼ��
    for(int i=0;i<PortNum;i++)
        for(int j=0;j<VC;j++)
            for(int k=0;k<BufferDepth;k++){
                VChannelMsg[i][j][k]=nullptr;
                VCOutPort[i][j][k]=-1;
            }
    //��VC�ٲ������г�ʼ��
    for(int i=0;i<PortNum;i++){
        VCAllocWinVCID[i]=-1;
        VCAllocWinMsg[i]=nullptr;
        VCAllocWinOutPort[i]=-1;

    }

    //��SA�ٲ������г�ʼ��
    for(int i=0;i<PortNum;i++){
        SAllocWinMsg[i]=nullptr;
        SAllocWinInPort[i]=-1;
    }

    //��selfMsg���г�ʼ��
    selfMsgAlloc = new cMessage("selfMsgAlloc");
    scheduleAt(Sim_Start_Time, selfMsgAlloc);
    selfMsgBufferInfo = new cMessage("selfMsgBufferInfo");
    scheduleAt(Buffer_Info_Sim_Start, selfMsgBufferInfo);

    for(int i=0;i<PortNum;i++){
        for(int j=0;j<VC;j++){
            BufferAvailCurrent[i][j]=true;
            BufferAvailConnect[i][j]=false;
        }
    }

}

void Router::handleMessage(cMessage *msg)
{

    if (msg->isSelfMessage()) {
        //****************�ٲö�ʱ****************************

        if(msg==selfMsgAlloc){//����ϢΪ�ٲö�ʱ��Ϣ

            scheduleAt(simTime()+CLK_CYCLE,selfMsgAlloc);

            // 3 ��ͨ���ٲ� Virtual Channel Allocation
            //��ͨ���ٲò���Round Robin��ѭ�����㷨
            //����VCAllocWinVCID������ָ��VC����ͨ��
            //��ÿ���˿ڶ������о�
            for(int i=0;i<PortNum;i++){
                //����ĳ������˿�����һ�������ٲ�ʤ����vc��sa�ٲ�ʱû�������˿ڵ��ٲ�ʤ��
                //��������غϾͲ�Ҫ�����ٲ��ˣ�������һ�εĽ��
                if(VCAllocWinMsg[i]==nullptr){
                    int vcid=(VCAllocWinVCID[i]+1)%VC;//port i��VChannelMsgָ�룬����һ���ٲóɹ���vcid����һ��vcid��ʼ
                    for(int j=0;j<VC;j++){
                        int vcid_tmp=(vcid+j)%VC;
                        if(VChannelMsg[i][vcid_tmp][0]!=nullptr){
                            VCAllocWinMsg[i]=VChannelMsg[i][vcid_tmp][0];//�ȴ��Ų�ɾ��������ȥ����ɾ
                            VCAllocWinVCID[i]=vcid_tmp;//����ʤ����vcid
                            VCAllocWinOutPort[i]=VCOutPort[i][vcid_tmp][0];//�洢ʤ����vc��msg������˿�
                            if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
                                EV<<"VC Allocation >> ROUTER: "<<getIndex()<<"("<<swpid2swlid(getIndex())<<"), INPORT: "<<i<<
                                        ", VCAllocWinVCID: "<<vcid_tmp<<", VCAllocWinMsg: { "<<VCAllocWinMsg[i]<<" }\n";
                            }
                            break;
                        }
                    }
                }
            }



            // 4 ���濪���ٲ� Switch Allocation
            //�ڶ��׶ε��ٲ���Ҫ�ǶԴ�ÿ������˿ڹ�������ͬһ����˿��������������ٲá�
            //��Ȼ��Щ�������������ĳһʤ��������ͨ�������ġ���Ϊ��P������˿ڣ������������ٲ���ҪP�����롣
            //���ﲻ��crossbar��ʵ�֣���forѭ������crossbar
            for(int i=0;i<PortNum;i++){//��ÿһ������˿�SAlloc����ѭ��
                if(SAllocWinMsg[i]==nullptr){//����˿ڻ�δ��ռ��
                    int last_win_inport=(SAllocWinInPort[i]+1)%PortNum;//����һ���ٲ�ʤ���Ķ˿ڵ���һ���˿ڿ�ʼ������ѭ�ٲ�
                    for(int j=0;j<PortNum;j++){//PortNum������˿�
                        int inport_tmp=(last_win_inport+j)%PortNum;
                        if(VCAllocWinOutPort[inport_tmp]==i){//���inport�ٲ�ʤ����vc�����port��outport��idһ�£����ٲ�ʤ��
                            int vc_id=VCAllocWinVCID[inport_tmp];
                            if(BufferAvailConnect[i][vc_id]==true){ //�¸�router��Ӧ�˿ڵ�vc����buffer�������
                                SAllocWinMsg[i]=VCAllocWinMsg[inport_tmp];
                                SAllocWinInPort[i]=inport_tmp;
                                if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
                                    EV<<"Switch Allocation >> ROUTER: "<<getIndex()<<"("<<swpid2swlid(getIndex())<<"), OUTPORT: "<<i<<
                                            ", SAllocWinInPort: "<<inport_tmp<<", SAllocWinMsg: { "<<SAllocWinMsg[i]<<" }\n";
                                }
                                break;//���SAllocΪ�գ���ѻ�ʤ������˿ڵ�msg��ֵ���������վ����������break����ѭ��
                            }

                        }
                    }
                }
            }


            // 5 ���濪�ش��� Switch Traversal
            // ��ÿһ������˿ڶ�ת����msg��ͬʱ�Դ�����ת����msg�������
            for(int i=0;i<PortNum;i++){//��ÿһ������˿ڽ���ת��
                if(SAllocWinMsg[i]!=nullptr){//����˿�������
                    int inport=SAllocWinInPort[i];
                    int vcid=VCAllocWinVCID[inport];
                    FatTreeMsg* forward_msg=VChannelMsg[inport][vcid][0];
                    int k=VCOutPort[inport][vcid][0];
                    if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
                        EV<<"Switch Traversal >> ROUTER: "<<getIndex()<<"("<<swpid2swlid(getIndex())<<"), OUTPORT: "<<i<<
                                ", SAllocWinInPort: "<<inport<<", VCAllocWinVCID: "<<vcid<<", VChannelMsg: { "
                                <<forward_msg<<" }, VCOutPort: "<<k<<'\n';
                    }
                    //ת��msg
                    forwardMessage(forward_msg,k); //����router����sink message, ֱ��ת������
                    //�����Ӧbuffer
                    SAllocWinMsg[i]=nullptr;
                    VCAllocWinMsg[inport]=nullptr;
                    VCAllocWinOutPort[inport]=-1;
                    //�����뻺��Buffer����shift
                    for(int j=0;j<BufferDepth-1;j++){
                        VChannelMsg[inport][vcid][j]=VChannelMsg[inport][vcid][j+1];
                        VCOutPort[inport][vcid][j]=VCOutPort[inport][vcid][j+1];
                    }
                    VChannelMsg[inport][vcid][BufferDepth-1]=nullptr;
                    VCOutPort[inport][vcid][BufferDepth-1]=-1;
                }

            }
        }else{//����ϢΪbuffer���¶�ʱ��Ϣ
            //******************����buffer��Ϣ��ʱ**********************
            //����buffer��Ϣ
            scheduleAt(simTime()+Buffer_Info_Update_Interval,selfMsgBufferInfo);
            for(int i=0;i<PortNum;i++){
                for(int j=0;j<VC;j++){
                    if(VChannelMsg[i][j][BufferDepth-1]==nullptr)
                        BufferAvailCurrent[i][j]=true;
                    else
                        BufferAvailCurrent[i][j]=false;
                }
            }
            //EV<<msg->getName()<<" name\n";
            //����bufferInfo�źŲ�����Ӧ�˿ڷ����ź�
            for(int i=0;i<PortNum;i++){
                int from_port=getNextRouterPort(i);
                BufferInfoMsg *bufferInfoMsg = new BufferInfoMsg("bufferInfoMsg");
                bufferInfoMsg->setFrom_port(from_port);
                bufferInfoMsg->setBufferAvailArraySize(VC);//arraySize�Ĵ�СΪVC������
                //����buffer��Ϣ
                for(int j=0;j<VC;j++){
                    bool avail=BufferAvailCurrent[i][j];
                    bufferInfoMsg->setBufferAvail(j, avail);
                }
                //����bufferInfoMsg
                forwardBufferInfoMsg(bufferInfoMsg, i);


            }

        }


    }else{ //������Ϣ�����յ�����·��������Ϣ
        //*************************�յ������˿�buffer������Ϣ************************
        if(strcmp("bufferInfoMsg", msg->getName()) == 0){
            //�յ�����ϢΪbuffer״̬��Ϣ������BufferAvailConnect[PortNum][VC]

            BufferInfoMsg *bufferInfoMsg = check_and_cast<BufferInfoMsg *>(msg);
            int from_port=bufferInfoMsg->getFrom_port();
            //����BufferAvailConnect[PortNum][VC]
            for(int j=0;j<VC;j++){
                BufferAvailConnect[from_port][j]=bufferInfoMsg->getBufferAvail(j);
            }
            if (Verbose >= VERBOSE_BUFFER_INFO_MESSAGES) {
                EV<<"Receiving bufferInfoMsg, Updating buffer state >> ROUTER: "<<getIndex()<<"("<<swpid2swlid(getIndex())<<"), INPORT: "<<from_port<<
                    ", Received MSG: { "<<bufferInfoMsg<<" }\n";
            }
            delete bufferInfoMsg;


        }else{
            //**********************�յ������˿ڵ�fatTreeMsg������Ϣ*******************
            //�յ�����ϢΪFatTreeMsg������Ϣ

            FatTreeMsg *ftmsg = check_and_cast<FatTreeMsg *>(msg);

            // 1 ����洢 Input Buffer�������ŵ��ĸ�virtual channel
            int vc_id = ftmsg->getVc_id(); // ��ȡvc id�� ��̬��ʽ����vc id
            int port_id = ftmsg->getFrom_router_port();//��ȡ�յ�msg�Ķ˿ںţ������˷����ĸ��˿ڵ�vc��
            int avil_buff_id=-1;
            //�ж�buffer�Ƿ��пռ䣬���buffer���пռ䣬���յ����źŴ浽buffer��ȥ
            if(VChannelMsg[port_id][vc_id][BufferDepth-1]==nullptr){
                //buffer_avail = true;
                for(int i=0;i<BufferDepth;i++){
                    if(VChannelMsg[port_id][vc_id][i]==nullptr){
                        VChannelMsg[port_id][vc_id][i]=ftmsg;
                        avil_buff_id=i;
                        break;
                    }
                }
            }else{
                if (Verbose >= VERBOSE_DEBUG_MESSAGES){
                    EV<<"Input Buffer and Routing >> ROUTER: "<<getIndex()<<"("<<swpid2swlid(getIndex())<<"), INPORT: "<<port_id<<
                        ", VCID: "<<vc_id<<", Received MSG: { "<<ftmsg<<" }, Buffer is full, dropping message\n";
                }

            }



            // 2 ·�ɼ��� Routing Computation
            int out_port_id=calRoutePort(ftmsg);//���㷢��msg�Ķ˿ں�
            if(avil_buff_id!=-1){
                VCOutPort[port_id][vc_id][avil_buff_id]=out_port_id;
                if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
                    EV<<"Input Buffer and Routing >> ROUTER: "<<getIndex()<<"("<<swpid2swlid(getIndex())<<"), INPORT: "<<port_id<<
                        ", VCID: "<<vc_id<<", Received MSG: { "<<ftmsg<<" }, OUTPORT: "<<out_port_id<<'\n';
                }
            }
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
    EV << "Forwarding message { " << msg << " } from router "<<cur_swpid<<"("<<cur_swlid<<")"<< " through port "<<k<<"\n";

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
    if (Verbose >= VERBOSE_BUFFER_INFO_MESSAGES) {
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


void Router::finish()
{
    // This function is called by OMNeT++ at the end of the simulation.
    //EV << "Sent:     " << numSent << endl;
    //EV << "Received: " << numReceived << endl;
    EV << "Hop count, min:    " << hopCountStats.getMin() << endl;
    EV << "Hop count, max:    " << hopCountStats.getMax() << endl;
    EV << "Hop count, mean:   " << hopCountStats.getMean() << endl;
    EV << "Hop count, stddev: " << hopCountStats.getStddev() << endl;

    //recordScalar("#sent", numSent);
    //recordScalar("#received", numReceived);

    hopCountStats.recordAs("hop count");
}

