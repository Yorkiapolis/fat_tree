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
#include <algorithm>
#include "fat_tree_pkt_m.h"
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

    long numFlitSent;
    long numPackageSent;
    long numFlitReceived;
    long numPackageReceived;
    long numPktDropped; //
    long flitByHop; //���ڼ�����·������, flit * Hop
    bool dropFlag; //�жϱ����Ƿ�drop��

    long headFlitGenTime; //head flit�Ĳ���ʱ�䣬���ڼ���package delay
    int packageDelayCount; //�Ե����package��flit���м���
    //cLongHistogram hopCountStats;
    cOutVector hopCountVector;
    cOutVector flitDelayTime;
    cOutVector packageDelayTime;

    int BufferConnectCredit[VC]; //����Processor�˿ڵ�Router��buffer��credit

    //int FlitCir; //����ѭ������Head Flit��Body Flit
    //int curFlitLength; //������package��Flit����
    //int preHeadFlitVCID; //���������һ��������Head Flit��VCID
    //FatTreeMsg* OutBuffer[FixedFlitLength]; //һ��package�Ĵ�С�����ڴ��flit���п��ܲ���body flitʱ���¸�router��buffer�����������Ҫ����һ��
    cQueue txQueue; //�������ݶ���

  public:
    Processor();
    virtual ~Processor();
  protected:
    virtual FatTreePkt* generateMessage(bool isHead, int flitCount, int vcid);
    virtual void forwardMessage(FatTreePkt *msg);
    virtual void forwardBufferInfoMsgP(BufferInfoMsg *msg);
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    virtual simtime_t channelAvailTime();
    virtual int generateBestVCID();
    virtual int ppid2plid(int ppid);
    virtual int plid2ppid(int plid);
    virtual int getNextRouterPortP(); //������processor������router�Ķ˿�
//    virtual double ParetoON();
//    virtual double ParetoOFF();
    virtual double Poisson();
    virtual double Uniform();


    // The finish() function is called by OMNeT++ at the end of the simulation:
    virtual void finish() override;
};

Define_Module(Processor);

Processor::Processor(){
    selfMsgGenMsg=nullptr;
    selfMsgSendMsg=nullptr;

}


Processor::~Processor(){
    cancelAndDelete(selfMsgGenMsg);
    cancelAndDelete(selfMsgSendMsg);

}

void Processor::initialize()
{
    // Initialize variables
    numFlitSent = 0;
    numPackageSent = 0;
    numFlitReceived = 0;
    numPackageReceived = 0;
    numPktDropped = 0;
    flitByHop = 0;
    //��ʼ���к��еĲ���
    //WATCH(numSent);
    //WATCH(numReceived);
    //WATCH(numPktDropped);

    //hopCountStats.setName("hopCountStats");
    //hopCountStats.setRangeAutoUpper(0, 10, 1.5);
    hopCountVector.setName("HopCount");
    flitDelayTime.setName("flitDelayTime");
    packageDelayTime.setName("packageDelayTime");

    selfMsgSendMsg = new cMessage("selfMsgSendMsg");//ע��˳���ȷ���buffer�����msg���ٲ����µ�msg������һ��flit��Ҫ2�����ڲŻᷢ��ȥ
    scheduleAt(Sim_Start_Time, selfMsgSendMsg);
    selfMsgGenMsg = new cMessage("selfMsgGenMsg");
    scheduleAt(Sim_Start_Time, selfMsgGenMsg);

    for (int i = 0; i < VC; i++) {
        BufferConnectCredit[i] = BufferDepth; //��ʼ������������router��buffer��Ϊ��
    }

    //FlitCir = 0; //��0��ʼѭ��
    //curFlitLength = FixedFlitLength;
    //preHeadFlitVCID = 0;

//    for(int i=0;i<FixedFlitLength;i++)
//        OutBuffer[i]=nullptr;

    dropFlag = false;


}

void Processor::handleMessage(cMessage *msg)
{
    if (msg->isSelfMessage()) {
        //********************���������ݵ��Զ�ʱ��Ϣ********************
        if(msg == selfMsgSendMsg){
            //****************************ת��flit**************************
            if(!txQueue.isEmpty()){ //���Ͷ���������
                FatTreePkt* current_forward_msg = (FatTreePkt*) txQueue.front();
                int vcid = current_forward_msg->getVc_id();
                if(channelAvailTime() <= simTime() && BufferConnectCredit[vcid] != 0) { //���Ͷ˿ڿ��У���һ���ڵ���buffer���ܴ�flit
                    txQueue.pop();
                    forwardMessage(current_forward_msg);
                    BufferConnectCredit[vcid]--;//decrement credit count
                    if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
                        EV<<"BufferConnectCredit["<<vcid<<"]="<<BufferConnectCredit[vcid]<<"\n";
                        numFlitSent++;
                    }
                    if (current_forward_msg->getIsHead() == true) {
                        numPackageSent++;
                    }
                }

            }

            scheduleAt(max(simTime()+CLK_CYCLE,channelAvailTime()),selfMsgSendMsg);

        }else if(msg == selfMsgGenMsg){

            if(getIndex() == 0){ //processor����msg��ģʽ,��Ҫ�Ľ�
            //if (true) {

                //**********************����flit*****************************
                if(txQueue.getLength() < ProcessorBufferDepth){ //Ҫ�����µ�Packet(head flit + body flit)��ͬʱbuffer���пռ����洢
                    int bestVCID = generateBestVCID();
                    for(int i = 0; i < FlitLength; i++) {
                        if(i == 0) {
                            FatTreePkt* msg = generateMessage(true, FlitLength, bestVCID);
                            txQueue.insert(msg);
                            if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
                                EV << "<<<<<<<<<<Processor: "<<getIndex()<<"("<<ppid2plid(getIndex())<<") is generating Head Flit>>>>>>>>>>\n";
                                EV << msg << endl;
                            }
                        } else {
                            FatTreePkt* msg = generateMessage(false, FlitLength, bestVCID);
                            txQueue.insert(msg);
                            if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
                                EV << "<<<<<<<<<<Processor: "<<getIndex()<<"("<<ppid2plid(getIndex())<<") is generating Body/Tail Flit>>>>>>>>>>\n";
                                EV << msg << endl;
                            }
                        }
                    }

                }else{//Ҫ�����µ�package������buffer�ռ䲻����drop����package
                    numPktDropped++;
                    dropFlag = true; //���drop��һ��packet�ģ�����Ϊ1����������Ķ�ʱʱ������һ��ʱ��������Ϊ��ʱ��λ�������ǲ��ɻ������Ƶ�ʱ����

                }

                //**********************������ʱ��Ϣ*****************************
                //package֮���ʱ����Ϊ���ɷֲ�����ȷֲ�
                if(dropFlag == false){
//#ifdef SELF_SIMILARITY //�����Ʒֲ�
//                    double offTime = ParetoOFF();
//
//                    if (Verbose >= VERBOSE_DETAIL_DEBUG_MESSAGES) {
//                        EV << "Self Similarity interval offTime: "<<offTime<<"\n";
//                    }
//                    scheduleAt(simTime()+offTime,selfMsgGenMsg);

#ifdef POISSON_DIST //���ɷֲ�
                    double expTime = Poisson();
                    if (Verbose >= VERBOSE_DETAIL_DEBUG_MESSAGES) {
                        EV << "Poisson interval: "<<expTime<<"\n";
                    }
                    scheduleAt(simTime()+expTime,selfMsgGenMsg);
#else //���ȷֲ�
                    double unitime = Uniform();
                    if (Verbose >= VERBOSE_DETAIL_DEBUG_MESSAGES) {
                        EV << "Uniform interval: "<<unitime<<"\n";
                    }
                    scheduleAt(simTime()+unitime,selfMsgGenMsg);
#endif

                }else{
                    scheduleAt(simTime() + CLK_CYCLE * FlitLength,selfMsgGenMsg);
                    dropFlag = false;
                }

            }

        }


    }else{
        //************************��self message*********************
        //************************�յ�buffer������Ϣ******************
        if(strcmp("bufferInfoMsg", msg->getName()) == 0){

            BufferInfoMsg *bufferInfoMsg = check_and_cast<BufferInfoMsg *>(msg);
            int from_port=bufferInfoMsg->getFrom_port();
            int vcid = bufferInfoMsg->getVcid();
            BufferConnectCredit[vcid]++; //increment credit count

            if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
                EV<<"Receiving bufferInfoMsg >> PROCESSOR: "<<getIndex()<<"("<<ppid2plid(getIndex())<<"), INPORT: "<<from_port<<
                    ", Received MSG: { "<<bufferInfoMsg<<" }\n";
                EV<<"BufferConnectCredit["<<vcid<<"]="<<BufferConnectCredit[vcid]<<"\n";
            }
            delete bufferInfoMsg;

        }else{
        //***********************�յ�FatTreePkt��Ϣ*******************
            FatTreePkt *ftmsg = check_and_cast<FatTreePkt *>(msg);

            // Message arrived
            int current_ppid=getIndex();
            int hopcount = ftmsg->getHopCount();
            if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
                EV << ">>>>>>>>>>Message {" << ftmsg << " } arrived after " << hopcount <<
                        " hops at node "<<current_ppid<<"("<<ppid2plid(current_ppid)<<")<<<<<<<<<<\n";
            }

            // update statistics.
            numFlitReceived++;
            flitByHop += hopcount + 1; //�������һ��·������processor
            if (ftmsg->getIsHead() == true) {
                packageDelayCount = ftmsg->getFlitCount();
                headFlitGenTime = ftmsg->getCreationTime(); //��ʱ�ı�ʾ�����Ӳ�����Flit����Flit��Ŀ��ڵ���գ����а����ڷ��Ͷ�txQueue�ĵȴ�ʱ��
            }
            packageDelayCount--;
            if (packageDelayCount == 0) {
                packageDelayTime.record(simTime().dbl() - headFlitGenTime);
                numPackageReceived++;
            }
            flitDelayTime.record(simTime().dbl() - ftmsg->getCreationTime());
            hopCountVector.record(hopcount);

            delete ftmsg;

        }

    }
}

int Processor::generateBestVCID() {
    //int vc_id = intuniform(0,VC-1); //�������vcͨ��, ������һ��router��vc buffer�����ѡ����ʵ�vcid
    int best_vcid = 0;
    for(int i = 0; i < VC; i++){
        if(BufferConnectCredit[i] > BufferConnectCredit[best_vcid]){
            best_vcid = i;
        }
    }
    return best_vcid;
}


FatTreePkt* Processor::generateMessage(bool isHead, int flitCount, int vcid)
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

        int current_plid=ppid2plid(current_ppid);
        int dst_plid=ppid2plid(dst_ppid);

        char msgname[200];//��ʼ����Ŀռ�̫С�������ݱ��ı�!!!!!!!
        sprintf(msgname, "Head Flit, From processor node %d(%d) to node %d(%d), Flit Length: %d", current_ppid,current_plid,dst_ppid,dst_plid,flitCount);

        // Create message object and set source and destination field.
        FatTreePkt *msg = new FatTreePkt(msgname);
        msg->setSrc_ppid(current_ppid);//���÷�����processor���
        msg->setDst_ppid(dst_ppid);//���ý��յ�processor���
        msg->setFrom_router_port(getNextRouterPortP());//�����յ���msg��Router�˿�
        msg->setVc_id(vcid);//����VCID
        msg->setIsHead(isHead); //����isHead flag
        msg->setFlitCount(flitCount); // ����Flit Count
        msg->setPackageGenTime(simTime().dbl());
        msg->setByteLength(FlitSize);

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
        FatTreePkt *msg = new FatTreePkt(msgname);
        msg->setSrc_ppid(-1);//���÷�����processor���
        msg->setDst_ppid(-1);//���ý��յ�processor���
        msg->setFrom_router_port(getNextRouterPortP());//�����յ���msg��Router�˿�
        msg->setVc_id(vcid);//����VC ID
        msg->setIsHead(isHead); //����isHead flag
        msg->setFlitCount(-1);
        msg->setByteLength(FlitSize);
        return msg;

    }
}

//�ж�ͨ���Ƿ���У����Դ�������
simtime_t Processor::channelAvailTime(){
    cChannel* txChannel = gate("port$o")->getTransmissionChannel();
    simtime_t txFinishTime = txChannel->getTransmissionFinishTime();
    return txFinishTime;

}

//processorת����·���㷨,processorֻ��һ��prot,ֱ��ת����ȥ����
void Processor::forwardMessage(FatTreePkt *msg)
{

    msg->setFrom_router_port(getNextRouterPortP());// �����յ�����Ϣ·�����Ķ˿ں�
    send(msg,"port$o");
    if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
        EV << "Forwarding message { " << msg << " } from processor to router, VCID = "<<msg->getVc_id()<<"\n";
    }

}

void Processor::forwardBufferInfoMsgP(BufferInfoMsg *msg){
    send(msg,"port$o");
    int cur_ppid=getIndex();//��ǰ·������id
    int cur_plid=ppid2plid(cur_ppid);
    if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
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

//double Processor::ParetoON() {
//    double exp_time = exponential((double)1/ALPHA_ON);
//    exp_time = round(exp_time / TimeScale) * CLK_CYCLE;
//    if(exp_time < CLK_CYCLE) {
//        exp_time = CLK_CYCLE;
//    }
//    return exp_time;
//}
//
//double Processor::ParetoOFF() {
//    double exp_time = exponential((double)1/ALPHA_OFF);
//    exp_time = round(exp_time / TimeScale) * CLK_CYCLE;
//    if(exp_time < CLK_CYCLE) {
//        exp_time = CLK_CYCLE;
//    }
//    return exp_time;
//}

double Processor::Poisson() {
    double exp_time = exponential((double)1.0/LAMBDA);
    exp_time = round(exp_time / TimeScale) * CLK_CYCLE;
    if(exp_time < CLK_CYCLE) {
        exp_time = CLK_CYCLE;
    }
    return exp_time * FlitLength; //����һ���Բ���FlitLength��Flit�����ʱ����ΪFlitLength�ı���������txQueue�ᱬ
}

double Processor::Uniform() {
    double time = round(1.0 / INJECTION_RATE) * CLK_CYCLE * FlitLength;
    return time;
}


void Processor::finish()
{
    // This function is called by OMNeT++ at the end of the simulation.
    EV << "Flit Sent: " << numFlitSent << endl;
    EV << "Package Sent: " << numPackageSent << endl;
    EV << "Flit Received: " << numFlitReceived << endl;
    EV << "Package Received: " << numPackageReceived << endl;
    EV << "Dropped:  " << numPktDropped << endl;
    //EV << "Hop count, min:    " << hopCountStats.getMin() << endl;
    //EV << "Hop count, max:    " << hopCountStats.getMax() << endl;
    //EV << "Hop count, mean:   " << hopCountStats.getMean() << endl;
    //EV << "Hop count, stddev: " << hopCountStats.getStddev() << endl;

    recordScalar("#flit sent", numFlitSent);
    recordScalar("#package sent", numPackageSent);
    recordScalar("#flit received", numFlitReceived);
    recordScalar("#package received", numPackageReceived);
    recordScalar("#packet dropped", numPktDropped);
    recordScalar("#flitByHop", flitByHop);


    if(getIndex() == 0) {
        double timeCount = simTime().dbl() - Sim_Start_Time;
        recordScalar("#timeCount", timeCount);
    }

}

