/*
 * processor.cc
 *
 *  Created on: 2016年7月30日
 *      Author: Vincent
 *
 *  Function:
 *      Flit模式：Head Flit和Body Flit
 *      时间分布：自相似(FlitLength不定长)，泊松(FlitLength定长)
 *      空间分布：均匀
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
    cMessage *selfMsgGenMsg;//产生flit定时，package产生按泊松分布或均匀分布
    cMessage *selfMsgSendMsg;//发送flit定时，每周期都检查buffer，再发送

    long numFlitSent;
    long numPackageSent;
    long numFlitReceived;
    long numPackageReceived;
    long numPktDropped; //
    long flitByHop; //用于计算链路利用率, flit * Hop
    bool dropFlag; //判断本轮是否drop过

    long headFlitGenTime; //head flit的产生时间，用于计算package delay
    int packageDelayCount; //对到达的package的flit进行计数
    //cLongHistogram hopCountStats;
    cOutVector hopCountVector;
    cOutVector flitDelayTime;
    cOutVector packageDelayTime;

    int BufferConnectCredit[VC]; //连接Processor端口的Router的buffer的credit

    //int FlitCir; //用来循环产生Head Flit和Body Flit
    //int curFlitLength; //不定长package的Flit长度
    //int preHeadFlitVCID; //用来存放上一个产生的Head Flit的VCID
    //FatTreeMsg* OutBuffer[FixedFlitLength]; //一个package的大小，用于存放flit，有可能产生body flit时，下个router的buffer已满，因此需要缓冲一下
    cQueue txQueue; //发送数据队列

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
    virtual int getNextRouterPortP(); //计算与processor相连的router的端口
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
    //初始化行和列的参数
    //WATCH(numSent);
    //WATCH(numReceived);
    //WATCH(numPktDropped);

    //hopCountStats.setName("hopCountStats");
    //hopCountStats.setRangeAutoUpper(0, 10, 1.5);
    hopCountVector.setName("HopCount");
    flitDelayTime.setName("flitDelayTime");
    packageDelayTime.setName("packageDelayTime");

    selfMsgSendMsg = new cMessage("selfMsgSendMsg");//注意顺序，先发送buffer里面的msg，再产生新的msg，这样一个flit需要2个周期才会发出去
    scheduleAt(Sim_Start_Time, selfMsgSendMsg);
    selfMsgGenMsg = new cMessage("selfMsgGenMsg");
    scheduleAt(Sim_Start_Time, selfMsgGenMsg);

    for (int i = 0; i < VC; i++) {
        BufferConnectCredit[i] = BufferDepth; //初始化与它相连的router的buffer都为空
    }

    //FlitCir = 0; //从0开始循环
    //curFlitLength = FixedFlitLength;
    //preHeadFlitVCID = 0;

//    for(int i=0;i<FixedFlitLength;i++)
//        OutBuffer[i]=nullptr;

    dropFlag = false;


}

void Processor::handleMessage(cMessage *msg)
{
    if (msg->isSelfMessage()) {
        //********************发送新数据的自定时消息********************
        if(msg == selfMsgSendMsg){
            //****************************转发flit**************************
            if(!txQueue.isEmpty()){ //发送队列有数据
                FatTreePkt* current_forward_msg = (FatTreePkt*) txQueue.front();
                int vcid = current_forward_msg->getVc_id();
                if(channelAvailTime() <= simTime() && BufferConnectCredit[vcid] != 0) { //发送端口空闲，下一个节点有buffer接受此flit
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

            if(getIndex() == 0){ //processor产生msg的模式,需要改进
            //if (true) {

                //**********************产生flit*****************************
                if(txQueue.getLength() < ProcessorBufferDepth){ //要产生新的Packet(head flit + body flit)，同时buffer又有空间来存储
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

                }else{//要产生新的package，但是buffer空间不够，drop掉该package
                    numPktDropped++;
                    dropFlag = true; //如果drop了一个packet的，则置为1，进入下面的定时时，会以一个时钟周期作为定时单位，而不是泊松或自相似的时间间隔

                }

                //**********************产生定时消息*****************************
                //package之间的时间间隔为泊松分布或均匀分布
                if(dropFlag == false){
//#ifdef SELF_SIMILARITY //自相似分布
//                    double offTime = ParetoOFF();
//
//                    if (Verbose >= VERBOSE_DETAIL_DEBUG_MESSAGES) {
//                        EV << "Self Similarity interval offTime: "<<offTime<<"\n";
//                    }
//                    scheduleAt(simTime()+offTime,selfMsgGenMsg);

#ifdef POISSON_DIST //泊松分布
                    double expTime = Poisson();
                    if (Verbose >= VERBOSE_DETAIL_DEBUG_MESSAGES) {
                        EV << "Poisson interval: "<<expTime<<"\n";
                    }
                    scheduleAt(simTime()+expTime,selfMsgGenMsg);
#else //均匀分布
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
        //************************非self message*********************
        //************************收到buffer更新消息******************
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
        //***********************收到FatTreePkt消息*******************
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
            flitByHop += hopcount + 1; //包含最后一跳路由器到processor
            if (ftmsg->getIsHead() == true) {
                packageDelayCount = ftmsg->getFlitCount();
                headFlitGenTime = ftmsg->getCreationTime(); //延时的表示方法从产生该Flit到该Flit被目标节点接收，其中包括在发送端txQueue的等待时间
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
    //int vc_id = intuniform(0,VC-1); //随机分配vc通道, 根据下一跳router的vc buffer情况来选择合适的vcid
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
        int n = getVectorSize();//processor的数量
        //EV<<n<<"\n";
#ifdef UNIFORM //均匀分布
        int dst_ppid = intuniform(0, n-2); //均匀流量模型
        //EV<<dst_ppid<<"\n";
        if (dst_ppid >= current_ppid)
            dst_ppid++;//保证不取到current_ppid
#endif

        int current_plid=ppid2plid(current_ppid);
        int dst_plid=ppid2plid(dst_ppid);

        char msgname[200];//初始分配的空间太小导致数据被改变!!!!!!!
        sprintf(msgname, "Head Flit, From processor node %d(%d) to node %d(%d), Flit Length: %d", current_ppid,current_plid,dst_ppid,dst_plid,flitCount);

        // Create message object and set source and destination field.
        FatTreePkt *msg = new FatTreePkt(msgname);
        msg->setSrc_ppid(current_ppid);//设置发出的processor编号
        msg->setDst_ppid(dst_ppid);//设置接收的processor编号
        msg->setFrom_router_port(getNextRouterPortP());//设置收到该msg的Router端口
        msg->setVc_id(vcid);//设置VCID
        msg->setIsHead(isHead); //设置isHead flag
        msg->setFlitCount(flitCount); // 设置Flit Count
        msg->setPackageGenTime(simTime().dbl());
        msg->setByteLength(FlitSize);

        if (Verbose >= VERBOSE_DETAIL_DEBUG_MESSAGES) {
            EV<<"From Processor::generateMessage, flit count: "<<msg->getFlitCount()<<", FlitLength: "<<flitCount<<"\n";
        }
        //EV<<current_ppid<<" "<<dst_ppid<<"\n";
        //EV<<msg->getSrc_ppid()<<" "<<msg->getDst_ppid()<<"\n";

        return msg;
    }else{

        char msgname[200];//初始分配的空间太小导致数据被改变!!!!!!!
        sprintf(msgname, "Body/Tail Flit");

        // Create message object and set source and destination field.
        FatTreePkt *msg = new FatTreePkt(msgname);
        msg->setSrc_ppid(-1);//设置发出的processor编号
        msg->setDst_ppid(-1);//设置接收的processor编号
        msg->setFrom_router_port(getNextRouterPortP());//设置收到该msg的Router端口
        msg->setVc_id(vcid);//设置VC ID
        msg->setIsHead(isHead); //设置isHead flag
        msg->setFlitCount(-1);
        msg->setByteLength(FlitSize);
        return msg;

    }
}

//判断通道是否空闲，可以传输数据
simtime_t Processor::channelAvailTime(){
    cChannel* txChannel = gate("port$o")->getTransmissionChannel();
    simtime_t txFinishTime = txChannel->getTransmissionFinishTime();
    return txFinishTime;

}

//processor转发的路由算法,processor只有一个prot,直接转发出去即可
void Processor::forwardMessage(FatTreePkt *msg)
{

    msg->setFrom_router_port(getNextRouterPortP());// 设置收到该信息路由器的端口号
    send(msg,"port$o");
    if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
        EV << "Forwarding message { " << msg << " } from processor to router, VCID = "<<msg->getVc_id()<<"\n";
    }

}

void Processor::forwardBufferInfoMsgP(BufferInfoMsg *msg){
    send(msg,"port$o");
    int cur_ppid=getIndex();//当前路由器的id
    int cur_plid=ppid2plid(cur_ppid);
    if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
        EV << "Forwarding BufferInfoMsg { " << msg << " } from processor "<<cur_ppid<<"("<<cur_plid<<")\n";
    }
}

int Processor::getNextRouterPortP(){
    int current_ppid=getIndex();
    int plid=ppid2plid(current_ppid);
    int port=plid%10;
    return port; //返回和processor相连的router的端口
}


//从ppid计算plid
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
//从plid计算ppid
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
    return exp_time * FlitLength; //由于一次性产生FlitLength个Flit，因此时间间隔为FlitLength的倍数，否则txQueue会爆
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

