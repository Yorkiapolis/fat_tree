/*
 * processor.cc
 *
 *  Created on: 2016��7��30��
 *      Author: Vincent
 *
 *  Function:
 *      Flitģʽ��Head Flit��Body Flit
 *      ʱ��ֲ���������(FlitLength������)������(FlitLength����)
 *      �ռ�ֲ�������
 *
 */


#include <stdio.h>
#include <string.h>
#include <omnetpp.h>
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


// Processor
class Processor : public cSimpleModule
{
  private:
    cMessage *selfMsgGenMsg;//����flit��ʱ��package���������ɷֲ�����ȷֲ�
    cMessage *selfMsgSendMsg;//����flit��ʱ��ÿ���ڶ����buffer���ٷ���
    cMessage *selfMsgBufferInfoP;//Processor��bufer���¶�ʱ�źţ���������������router������vcͨ����Ϊavail

    long numSent;
    long numReceived;
    long numDropped;
    cLongHistogram hopCountStats;
    cOutVector hopCountVector;

    bool BufferAvailCurrentP[VC];//��ǰProcessor��buffer״̬��Ĭ�϶�Ϊavailable
    bool BufferAvailConnectP[VC];//����Processor�˿ڵ�Router��buffer״̬

    int FlitCir; //����ѭ������Head Flit��Body Flit
    int curFlitLength; //������package��Flit����
    int preHeadFlitVCID; //���������һ��������Head Flit��VCID��body flit������vcid
    FatTreeMsg* OutBuffer[FixedFlitLength]; //һ��package�Ĵ�С�����ڴ��flit���п��ܲ���body flitʱ���¸�router��buffer�����������Ҫ����һ��

  public:
    Processor();
    virtual ~Processor();
  protected:
    virtual FatTreeMsg *generateMessage(bool isHead, int flitCount);
    virtual void forwardMessage(FatTreeMsg *msg);
    virtual void forwardBufferInfoMsgP(BufferInfoMsg *msg);
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    virtual int ppid2plid(int ppid);
    virtual int plid2ppid(int plid);
    virtual int getNextRouterPortP(); //������processor������router�Ķ˿�
    virtual double ParetoON();
    virtual double ParetoOFF();
    virtual double Poisson();


    // The finish() function is called by OMNeT++ at the end of the simulation:
    virtual void finish() override;
};

Define_Module(Processor);

Processor::Processor(){
    selfMsgGenMsg=nullptr;
    selfMsgSendMsg=nullptr;
    selfMsgBufferInfoP=nullptr;
}


Processor::~Processor(){
    cancelAndDelete(selfMsgGenMsg);
    cancelAndDelete(selfMsgSendMsg);
    cancelAndDelete(selfMsgBufferInfoP);
}

void Processor::initialize()
{
    // Initialize variables
    numSent = 0;
    numReceived = 0;
    numDropped = 0;
    //��ʼ���к��еĲ���
    WATCH(numSent);
    WATCH(numReceived);
    WATCH(numDropped);

    hopCountStats.setName("hopCountStats");
    hopCountStats.setRangeAutoUpper(0, 10, 1.5);
    hopCountVector.setName("HopCount");

    selfMsgSendMsg = new cMessage("selfMsgSendMsg");//ע��˳���ȷ���buffer�����msg���ٲ����µ�msg������һ��flit��Ҫ2�����ڲŻᷢ��ȥ
    scheduleAt(Sim_Start_Time, selfMsgSendMsg);
    selfMsgGenMsg = new cMessage("selfMsgGenMsg");
    scheduleAt(Sim_Start_Time, selfMsgGenMsg);
    selfMsgBufferInfoP = new cMessage("selfMsgBufferInfoP");
    scheduleAt(Buffer_Info_Sim_Start, selfMsgBufferInfoP);

    for(int i=0;i<VC;i++){
        BufferAvailCurrentP[i]=true;
        BufferAvailConnectP[i]=false;
    }

    FlitCir = 0; //��0��ʼѭ��
    curFlitLength = FixedFlitLength;
    preHeadFlitVCID = 0;

    for(int i=0;i<FixedFlitLength;i++)
        OutBuffer[i]=nullptr;


    // Module 0 sends the first message
    // ѡ��ĳ��ppid��processor����������
    /*
    if (getIndex()%2 == 0) {
        // Boot the process scheduling the initial message as a self-message.
        FatTreeMsg *msg = generateMessage();
        //�ӳٷ���0.1s
        scheduleAt(1, msg);
    }
    */
}

void Processor::handleMessage(cMessage *msg)
{
    if (msg->isSelfMessage()) {
        //********************���������ݵ��Զ�ʱ��Ϣ********************
        if(msg == selfMsgSendMsg){
            //****************************ת��flit**************************
            if(OutBuffer[0] != nullptr && BufferAvailConnectP[preHeadFlitVCID] == true){ //��һ���ڵ���buffer���ܴ�flit
                FatTreeMsg* current_forward_msg = OutBuffer[0];
                forwardMessage(current_forward_msg);
                numSent++;
                for(int i=0;i<FixedFlitLength-1;i++){
                    OutBuffer[i] = OutBuffer[i+1];
                }
                OutBuffer[FixedFlitLength-1] = nullptr;
            }
            scheduleAt(simTime()+CLK_CYCLE,selfMsgSendMsg);

        }else if(msg == selfMsgGenMsg){

            if(getIndex() == 0){ //processor����msg��ģʽ,��Ҫ�Ľ�

                //**********************����flit*****************************
                if(FlitCir == 0 && OutBuffer[0] == nullptr){ //Ҫ�����µ�head flit��ͬʱbuffer���пռ����洢ʣ���body flit

#ifdef SELF_SIMILARITY
                    double onTime = ParetoON();
                    curFlitLength = onTime / CLK_CYCLE;
                    if (curFlitLength == 0) {
                        curFlitLength = 1;
                    }else if (curFlitLength > FixedFlitLength) {
                        curFlitLength = FixedFlitLength;
                    }
#else
                    curFlitLength = FixedFlitLength;
#endif

                    EV << "<<<<<<<<<<Processor: "<<getIndex()<<"("<<ppid2plid(getIndex())<<") is generating Head Flit>>>>>>>>>>\n";
                    FatTreeMsg *newmsg = generateMessage(true, curFlitLength); //Head Flit
                    EV << newmsg << endl;
                    preHeadFlitVCID = newmsg->getVc_id();
                    OutBuffer[0] = newmsg;
                    FlitCir = (FlitCir+1) % curFlitLength;

                }else if(FlitCir != 0 ){//����body flit��bufferһ���пռ�
                    EV << "<<<<<<<<<<Processor: "<<getIndex()<<"("<<ppid2plid(getIndex())<<") is generating Body/Tail Flit>>>>>>>>>>\n";
                    FatTreeMsg *newmsg = generateMessage(false, -1); // Body or Tail Flit
                    EV << newmsg << endl;
                    for(int i=0;i<FixedFlitLength;i++){
                        if(OutBuffer[i] == nullptr){
                            OutBuffer[i] = newmsg;
                            break;
                        }
                    }
                    FlitCir = (FlitCir+1) % curFlitLength;


                }else{//Ҫ�����µ�package������buffer�ռ䲻����drop����package
                    numDropped++;

                }

                //int vc_id=newmsg->getVc_id();//��ȡvcid
                     /*
                     bool nextRouterHasBuffer = false;
                     int nextVCID = -1;
                     for(int i=0;i<VC;i++){
                         if(BufferAvailConnectP[i]==true){
                             nextVCID = i;
                             nextRouterHasBuffer = true;
                             break;
                         }

                     }
                     if(nextRouterHasBuffer){//ת���µ�fatTreeMsg
                         newmsg->setVc_id(nextVCID);
                         forwardMessage(newmsg);
                         if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
                             EV<<"Processor Generating and Forwarding New FatTreeMsg >> PROCESSOR: "<<getIndex()<<"("<<ppid2plid(getIndex())<<"), VCID: "
                                     <<nextVCID<<"\n";
                         }
                         //bubble("Starting new one!");
                     }else{//buffer������dropping msg
                         if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
                             EV<<"Next Router's Buffer is Full, Dropping FatTreeMsg >> PROCESSOR: "<<getIndex()<<"("<<ppid2plid(getIndex())<<")\n";
                         }
                         delete newmsg;

                     }
                     */



                //simtime_t delay = par("delayTime");
                //simtime_t sendNext = par("sendNext");
                //scheduleAt(simTime()+0.1, msg);

                //package֮���ʱ����Ϊ���ɷֲ��������Ʒֲ���ͬһ��package��flit֮����ΪCLK_CYCLE
                if(FlitCir == 0){
#ifdef SELF_SIMILARITY //�����Ʒֲ�
                    double offTime = ParetoOFF();

                    //if (Verbose >= VERBOSE_DETAIL_DEBUG_MESSAGES) {
                        EV << "Self Similarity interval offTime: "<<offTime<<"\n";
                    //}
                    scheduleAt(simTime()+offTime,selfMsgGenMsg);

#elif defined POISSON_DIST //���ɷֲ�
                    double expTime = Poisson();

                    if (Verbose >= VERBOSE_DETAIL_DEBUG_MESSAGES) {
                        EV << "Poisson interval: "<<expTime<<"\n";
                    }

                    scheduleAt(simTime()+expTime,selfMsgGenMsg);
#else //���ȷֲ�

                    scheduleAt(simTime()+CLK_CYCLE,selfMsgGenMsg);
#endif

                }else{
                    scheduleAt(simTime()+CLK_CYCLE,selfMsgGenMsg);
                }


            }

        }else{
            //******************����buffer��Ϣ��ʱ**********************
            scheduleAt(simTime()+Buffer_Info_Update_Interval,selfMsgBufferInfoP);


            //����bufferInfo�źŲ�����Ӧ�˿ڷ����ź�

            int from_port=getNextRouterPortP();
            BufferInfoMsg *bufferInfoMsg = new BufferInfoMsg("bufferInfoMsg");//��Ϣ���ֲ�Ҫ��
            bufferInfoMsg->setFrom_port(from_port);
            bufferInfoMsg->setBufferAvailArraySize(VC);//arraySize�Ĵ�СΪVC������
            //����buffer��Ϣ
            for(int j=0;j<VC;j++){
                bool avail=BufferAvailCurrentP[j];
                bufferInfoMsg->setBufferAvail(j, avail);
            }
            //����bufferInfoMsg
            forwardBufferInfoMsgP(bufferInfoMsg);


        }

    }else{
        //************************��self message*********************
        //************************�յ�buffer������Ϣ******************
        if(strcmp("bufferInfoMsg", msg->getName()) == 0){
            //�յ�����ϢΪbuffer״̬��Ϣ������BufferAvailConnect[PortNum][VC]


            BufferInfoMsg *bufferInfoMsg = check_and_cast<BufferInfoMsg *>(msg);
            int from_port=bufferInfoMsg->getFrom_port();
            //����BufferAvailConnect[PortNum][VC]
            for(int j=0;j<VC;j++){
                BufferAvailConnectP[j]=bufferInfoMsg->getBufferAvail(j);
            }
            if (Verbose >= VERBOSE_BUFFER_INFO_MESSAGES) {
                EV<<"Receiving bufferInfoMsg, Updating buffer state >> PROCESSOR: "<<getIndex()<<"("<<ppid2plid(getIndex())<<"), INPORT: "<<from_port<<
                    ", Received MSG: { "<<bufferInfoMsg<<" }\n";
            }
            delete bufferInfoMsg;


        }else{
        //***********************�յ�FatTreeMsg��Ϣ*******************
            FatTreeMsg *ftmsg = check_and_cast<FatTreeMsg *>(msg);
            //EV<<"Message goto "<<getRow()<<","<<getCol()<<"\n";
            //int src_ppid=ftmsg->getSrc_ppid();

            // Message arrived
            int current_ppid=getIndex();
            int hopcount = ftmsg->getHopCount();
            if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
                EV << ">>>>>>>>>>Message {" << ftmsg << " } arrived after " << hopcount <<
                        " hops at node "<<current_ppid<<"("<<ppid2plid(current_ppid)<<")<<<<<<<<<<\n";
            }
            //bubble("ARRIVED!");

            // update statistics.
            numReceived++;
            hopCountVector.record(hopcount);
            hopCountStats.collect(hopcount);

            delete ftmsg;

            /*
            int dst_ppid=ftmsg->getDst_ppid();
            int current_ppid=getIndex();
            if (current_ppid==dst_ppid) {
                // Message arrived
                int hopcount = ftmsg->getHopCount();
                EV << "Message {" << ftmsg << " } arrived after " << hopcount <<
                        " hops at node "<<current_ppid<<"("<<ppid2plid(current_ppid)<<")\n";
                bubble("ARRIVED!");

                // update statistics.
                numReceived++;
                hopCountVector.record(hopcount);
                hopCountStats.collect(hopcount);

                delete ftmsg;

                // Generate another one.
                //EV << "Generating another message: ";
                //FatTreeMsg *newmsg = generateMessage();
                //EV << newmsg << endl;
                //forwardMessage(newmsg);
                //numSent++;
            }
            else {
                //*******************�յ�������Ϣ************************
                // We need to forward the message.
                //forwardMessage(ftmsg);
                EV << "Received the wrong message: {" << ftmsg << " }\n";
                delete ftmsg;

            }
            */

        }

    }
}

FatTreeMsg* Processor::generateMessage(bool isHead, int flitCount)
{

    if(isHead){
        // Head Flit

        // Produce source and destination address
        int current_ppid=getIndex();
        int n = getVectorSize();//processor������
        //EV<<n<<"\n";
#ifdef UNIFORM //���ȷֲ�
        int dst_ppid = intuniform(0, n-2); //��������ģ��
        //EV<<dst_ppid<<"\n";
        if (dst_ppid >= current_ppid)
            dst_ppid++;//��֤��ȡ��current_ppid
#endif

        int vc_id = intuniform(0,VC-1); //�������vcͨ��, ������һ��router��vc buffer�����ѡ����ʵ�vcid
        for(int i=0;i<VC;i++){
            vc_id = (vc_id + i) % VC;
            if(BufferAvailConnectP[vc_id] == true){
                break;
            }
        }



        int current_plid=ppid2plid(current_ppid);
        int dst_plid=ppid2plid(dst_ppid);

        char msgname[200];//��ʼ����Ŀռ�̫С�������ݱ��ı�!!!!!!!
        sprintf(msgname, "Head Flit, From processor node %d(%d) to node %d(%d), Flit Length: %d", current_ppid,current_plid,dst_ppid,dst_plid,flitCount);

        // Create message object and set source and destination field.
        FatTreeMsg *msg = new FatTreeMsg(msgname);
        msg->setSrc_ppid(current_ppid);//���÷�����processor���
        msg->setDst_ppid(dst_ppid);//���ý��յ�processor���
        msg->setFrom_router_port(getNextRouterPortP());//�����յ���msg��Router�˿�
        msg->setVc_id(vc_id);//����VC ID
        msg->setIsHead(isHead); //����isHead flag
        msg->setFlitCount(flitCount); // ����Flit Count


        if (Verbose >= VERBOSE_DETAIL_DEBUG_MESSAGES) {
            EV<<"From Processor::generateMessage, flit count: "<<msg->getFlitCount()<<", FlitLength: "<<flitCount<<"\n";
        }
        //EV<<current_ppid<<" "<<dst_ppid<<"\n";
        //EV<<msg->getSrc_ppid()<<" "<<msg->getDst_ppid()<<"\n";

        return msg;
    }else{
        char msgname[200];//��ʼ����Ŀռ�̫С�������ݱ��ı�!!!!!!!
        sprintf(msgname, "Body/Tail Flit");

        // Create message object and set source and destination field.
        FatTreeMsg *msg = new FatTreeMsg(msgname);
        msg->setSrc_ppid(-1);//���÷�����processor���
        msg->setDst_ppid(-1);//���ý��յ�processor���
        msg->setFrom_router_port(getNextRouterPortP());//�����յ���msg��Router�˿�
        msg->setVc_id(-1);//����VC ID
        msg->setIsHead(isHead); //����isHead flag
        msg->setFlitCount(-1);

        return msg;

    }


}

//processorת����·���㷨,processorֻ��һ��prot,ֱ��ת����ȥ����
void Processor::forwardMessage(FatTreeMsg *msg)
{

    msg->setFrom_router_port(getNextRouterPortP());// �����յ�����Ϣ·�����Ķ˿ں�
    send(msg,"port$o");
    EV << "Forwarding message { " << msg << " } from processor to router, VCID = "<<preHeadFlitVCID<<"\n";

}
void Processor::forwardBufferInfoMsgP(BufferInfoMsg *msg){
    send(msg,"port$o");
    int cur_ppid=getIndex();//��ǰ·������id
    int cur_plid=ppid2plid(cur_ppid);
    if (Verbose >= VERBOSE_BUFFER_INFO_MESSAGES) {
        EV << "Forwarding BufferInfoMsg { " << msg << " } from processor "<<cur_ppid<<"("<<cur_plid<<")\n";
    }
}


int Processor::getNextRouterPortP(){
    int current_ppid=getIndex();
    int plid=ppid2plid(current_ppid);
    int port=plid%10;
    return port; //���غ�processor������router�Ķ˿�
}


//��ppid����plid
int Processor::ppid2plid(int ppid){
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
int Processor::plid2ppid(int plid){
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

double Processor::ParetoON() {
    double exp_time = exponential((double)1/ALPHA_ON);
    exp_time = round(exp_time*ROUND)/ROUND;
    if(exp_time < CLK_CYCLE) {
        exp_time = CLK_CYCLE;
    }
    return exp_time;
}

double Processor::ParetoOFF() {
    double exp_time = exponential((double)1/ALPHA_OFF);
    exp_time = round(exp_time*ROUND)/ROUND;
    if(exp_time < CLK_CYCLE) {
        exp_time = CLK_CYCLE;
    }
    return exp_time;
}

double Processor::Poisson() {
    double exp_time = exponential((double)1/LAMBDA);
    exp_time = round(exp_time*ROUND)/ROUND;
    if(exp_time < CLK_CYCLE) {
        exp_time = CLK_CYCLE;
    }
    return exp_time;
}

void Processor::finish()
{
    // This function is called by OMNeT++ at the end of the simulation.
    EV << "Sent:     " << numSent << endl;
    EV << "Received: " << numReceived << endl;
    EV << "Dropped:  " << numDropped << endl;
    EV << "Hop count, min:    " << hopCountStats.getMin() << endl;
    EV << "Hop count, max:    " << hopCountStats.getMax() << endl;
    EV << "Hop count, mean:   " << hopCountStats.getMean() << endl;
    EV << "Hop count, stddev: " << hopCountStats.getStddev() << endl;

    recordScalar("#sent", numSent);
    recordScalar("#received", numReceived);
    recordScalar("#dropped", numDropped);


    hopCountStats.recordAs("hop count");
}




