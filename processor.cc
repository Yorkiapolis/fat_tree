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
    cMessage *selfMsgGenMsg;//产生flit定时，package产生按泊松分布或均匀分布
    cMessage *selfMsgSendMsg;//发送flit定时，每周期都检查buffer，再发送
    //cMessage *selfMsgBufferInfoP;//Processor的bufer更新定时信号，告诉与它相连的router，所有vc通道都为avail

    long numFlitSent;
    long numPackageSent;
    long numFlitReceived;
    long numPackageReceived;
    long numDropped;
    long flitByHop; //用于计算链路利用率
    bool dropFlag; //判断本轮是否drop过

    long headFlitGenTime; //head flit的产生时间，同于计算package delay
    int packageDelayCount; //对到达的package的flit进行计数
    //cLongHistogram hopCountStats;
    cOutVector hopCountVector;
    cOutVector flitDelayTime;
    cOutVector packageDelayTime;

    //bool BufferAvailCurrentP[VC];//当前Processor的buffer状态，默认都为available
    //bool BufferAvailConnectP[VC];//相连Processor端口的Router的buffer状态
    int BufferConnectCredit[VC]; //连接Processor端口的Router的buffer的credit

    int FlitCir; //用来循环产生Head Flit和Body Flit
    int curFlitLength; //不定长package的Flit长度
    int preHeadFlitVCID; //用来存放上一个产生的Head Flit的VCID，body flit不保存vcid
    FatTreeMsg* OutBuffer[FixedFlitLength]; //一个package的大小，用于存放flit，有可能产生body flit时，下个router的buffer已满，因此需要缓冲一下

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
    virtual int getNextRouterPortP(); //计算与processor相连的router的端口
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
    //selfMsgBufferInfoP=nullptr;
}


Processor::~Processor(){
    cancelAndDelete(selfMsgGenMsg);
    cancelAndDelete(selfMsgSendMsg);
    //cancelAndDelete(selfMsgBufferInfoP);
}

void Processor::initialize()
{
    // Initialize variables
    numFlitSent = 0;
    numPackageSent = 0;
    numFlitReceived = 0;
    numPackageReceived = 0;
    numDropped = 0;
    flitByHop = 0;
    //初始化行和列的参数
    //WATCH(numSent);
    //WATCH(numReceived);
    //WATCH(numDropped);

    //hopCountStats.setName("hopCountStats");
    //hopCountStats.setRangeAutoUpper(0, 10, 1.5);
    hopCountVector.setName("HopCount");
    flitDelayTime.setName("flitDelayTime");
    packageDelayTime.setName("packageDelayTime");

    selfMsgSendMsg = new cMessage("selfMsgSendMsg");//注意顺序，先发送buffer里面的msg，再产生新的msg，这样一个flit需要2个周期才会发出去
    scheduleAt(Sim_Start_Time, selfMsgSendMsg);
    selfMsgGenMsg = new cMessage("selfMsgGenMsg");
    scheduleAt(Sim_Start_Time, selfMsgGenMsg);
    //selfMsgBufferInfoP = new cMessage("selfMsgBufferInfoP");
    //scheduleAt(Buffer_Info_Sim_Start, selfMsgBufferInfoP);

    /*
    for(int i=0;i<VC;i++){
        BufferAvailCurrentP[i]=true;
        BufferAvailConnectP[i]=false;
    }
    */

    for (int i = 0; i < VC; i++) {
        BufferConnectCredit[i] = BufferDepth; //初始化与它相连的router的buffer都为空
    }

    FlitCir = 0; //从0开始循环
    curFlitLength = FixedFlitLength;
    preHeadFlitVCID = 0;

    for(int i=0;i<FixedFlitLength;i++)
        OutBuffer[i]=nullptr;

    dropFlag = false;


    // Module 0 sends the first message
    // 选定某个ppid的processor来产生数据
    /*
    if (getIndex()%2 == 0) {
        // Boot the process scheduling the initial message as a self-message.
        FatTreeMsg *msg = generateMessage();
        //延迟发送0.1s
        scheduleAt(1, msg);
    }
    */
}

void Processor::handleMessage(cMessage *msg)
{
    if (msg->isSelfMessage()) {
        //********************发送新数据的自定时消息********************
        if(msg == selfMsgSendMsg){
            //****************************转发flit**************************
            if(OutBuffer[0] != nullptr && BufferConnectCredit[preHeadFlitVCID] != 0){ //下一个节点有buffer接受此flit
                FatTreeMsg* current_forward_msg = OutBuffer[0];
                forwardMessage(current_forward_msg);
                BufferConnectCredit[preHeadFlitVCID]--;//decrement credit count
                if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
                    EV<<"BufferConnectCredit["<<preHeadFlitVCID<<"]="<<BufferConnectCredit[preHeadFlitVCID]<<"\n";
                }
                numFlitSent++;
                if (current_forward_msg->getIsHead() == true) {
                    numPackageSent++;
                }
                for(int i=0;i<FixedFlitLength-1;i++){
                    OutBuffer[i] = OutBuffer[i+1];
                }
                OutBuffer[FixedFlitLength-1] = nullptr;
            }
            scheduleAt(simTime()+CLK_CYCLE,selfMsgSendMsg);

        }else if(msg == selfMsgGenMsg){

            if(getIndex() == 0){ //processor产生msg的模式,需要改进
            //if (true) {

                //**********************产生flit*****************************
                if(FlitCir == 0 && OutBuffer[0] == nullptr){ //要产生新的head flit，同时buffer又有空间来存储剩余的body flit

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
                    FatTreeMsg *newmsg = generateMessage(true, curFlitLength); //Head Flit
                    if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
                        EV << "<<<<<<<<<<Processor: "<<getIndex()<<"("<<ppid2plid(getIndex())<<") is generating Head Flit>>>>>>>>>>\n";
                        EV << newmsg << endl;
                    }

                    preHeadFlitVCID = newmsg->getVc_id();
                    OutBuffer[0] = newmsg;
                    FlitCir = (FlitCir+1) % curFlitLength;

                }else if(FlitCir != 0 ){//产生body flit，buffer一定有空间

                    FatTreeMsg *newmsg = generateMessage(false, -1); // Body or Tail Flit
                    if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
                        EV << "<<<<<<<<<<Processor: "<<getIndex()<<"("<<ppid2plid(getIndex())<<") is generating Body/Tail Flit>>>>>>>>>>\n";
                        EV << newmsg << endl;
                    }

                    for(int i=0;i<FixedFlitLength;i++){
                        if(OutBuffer[i] == nullptr){
                            OutBuffer[i] = newmsg;
                            break;
                        }
                    }
                    FlitCir = (FlitCir+1) % curFlitLength;


                }else{//要产生新的package，但是buffer空间不够，drop掉该package
                    //FlitCir == 0 && OutBuffer[0] != nullptr
                    numDropped++;
                    dropFlag = true; //如果drop了一个flit的，则置为1，进入下面的定时时，会以一个时钟周期作为定时单位，而不是泊松或自相似的时间间隔

                }



                //**********************产生定时消息*****************************
                //package之间的时间间隔为泊松分布或自相似分布，同一个package的flit之间间隔为CLK_CYCLE
                if(FlitCir == 0 && dropFlag == false){
#ifdef SELF_SIMILARITY //自相似分布
                    double offTime = ParetoOFF();

                    if (Verbose >= VERBOSE_DETAIL_DEBUG_MESSAGES) {
                        EV << "Self Similarity interval offTime: "<<offTime<<"\n";
                    }
                    scheduleAt(simTime()+offTime,selfMsgGenMsg);

#elif defined POISSON_DIST //泊松分布
                    double expTime = Poisson();

                    if (Verbose >= VERBOSE_DETAIL_DEBUG_MESSAGES) {
                        EV << "Poisson interval: "<<expTime<<"\n";
                    }

                    scheduleAt(simTime()+expTime,selfMsgGenMsg);
#else //均匀分布

                    scheduleAt(simTime()+CLK_CYCLE,selfMsgGenMsg);
#endif

                }else{
                    //FlitCir!=0 || (FlitCir == 0 && dropFlag == true)
                    scheduleAt(simTime()+CLK_CYCLE,selfMsgGenMsg);
                    dropFlag = false;
                }


            }

        }
        /*
        else{
            //******************更新buffer信息定时**********************
            scheduleAt(simTime()+Buffer_Info_Update_Interval,selfMsgBufferInfoP);


            //产生bufferInfo信号并向相应端口发送信号

            int from_port=getNextRouterPortP();
            BufferInfoMsg *bufferInfoMsg = new BufferInfoMsg("bufferInfoMsg");//消息名字不要改
            bufferInfoMsg->setFrom_port(from_port);
            bufferInfoMsg->setBufferAvailArraySize(VC);//arraySize的大小为VC的数量
            //设置buffer信息
            for(int j=0;j<VC;j++){
                bool avail=BufferAvailCurrentP[j];
                bufferInfoMsg->setBufferAvail(j, avail);
            }
            //发送bufferInfoMsg
            forwardBufferInfoMsgP(bufferInfoMsg);


        }
        */

    }else{
        //************************非self message*********************
        //************************收到buffer更新消息******************
        if(strcmp("bufferInfoMsg", msg->getName()) == 0){
            //收到的消息为buffer状态消息，更新BufferAvailConnect[PortNum][VC]


            BufferInfoMsg *bufferInfoMsg = check_and_cast<BufferInfoMsg *>(msg);
            int from_port=bufferInfoMsg->getFrom_port();
            //更新BufferAvailConnect[PortNum][VC]
            //for(int j=0;j<VC;j++){
            //    BufferAvailConnectP[j]=bufferInfoMsg->getBufferAvail(j);
            //}

            int vcid = bufferInfoMsg->getVcid();
            BufferConnectCredit[vcid]++; //increment credit count

            if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
                EV<<"Receiving bufferInfoMsg >> PROCESSOR: "<<getIndex()<<"("<<ppid2plid(getIndex())<<"), INPORT: "<<from_port<<
                    ", Received MSG: { "<<bufferInfoMsg<<" }\n";
                EV<<"BufferConnectCredit["<<vcid<<"]="<<BufferConnectCredit[vcid]<<"\n";
            }
            delete bufferInfoMsg;


        }else{
        //***********************收到FatTreeMsg消息*******************
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
            numFlitReceived++;
            flitByHop += hopcount + 1; //包含最后一跳路由器到processor
            if (ftmsg->getIsHead() == true) {
                packageDelayCount = ftmsg->getFlitCount();
                headFlitGenTime = ftmsg->getPackageGenTime();

            }
            packageDelayCount--;
            if (packageDelayCount == 0) {
                packageDelayTime.record(simTime().dbl() - headFlitGenTime);
                numPackageReceived++;
            }
            flitDelayTime.record(simTime().dbl() - ftmsg->getFlitGenTime());
            hopCountVector.record(hopcount);


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
                //*******************收到错误消息************************
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
        int n = getVectorSize();//processor的数量
        //EV<<n<<"\n";
#ifdef UNIFORM //均匀分布
        int dst_ppid = intuniform(0, n-2); //均匀流量模型
        //EV<<dst_ppid<<"\n";
        if (dst_ppid >= current_ppid)
            dst_ppid++;//保证不取到current_ppid
#endif

        int vc_id = intuniform(0,VC-1); //随机分配vc通道, 根据下一跳router的vc buffer情况来选择合适的vcid
        for(int i=0;i<VC;i++){
            vc_id = (vc_id + i) % VC;
            if(BufferConnectCredit[vc_id] != 0){
                break;
            }
        }



        int current_plid=ppid2plid(current_ppid);
        int dst_plid=ppid2plid(dst_ppid);

        char msgname[200];//初始分配的空间太小导致数据被改变!!!!!!!
        sprintf(msgname, "Head Flit, From processor node %d(%d) to node %d(%d), Flit Length: %d", current_ppid,current_plid,dst_ppid,dst_plid,flitCount);

        // Create message object and set source and destination field.
        FatTreeMsg *msg = new FatTreeMsg(msgname);
        msg->setSrc_ppid(current_ppid);//设置发出的processor编号
        msg->setDst_ppid(dst_ppid);//设置接收的processor编号
        msg->setFrom_router_port(getNextRouterPortP());//设置收到该msg的Router端口
        msg->setVc_id(vc_id);//设置VC ID
        msg->setIsHead(isHead); //设置isHead flag
        msg->setFlitCount(flitCount); // 设置Flit Count
        msg->setPackageGenTime(simTime().dbl());
        msg->setFlitGenTime(simTime().dbl());


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
        FatTreeMsg *msg = new FatTreeMsg(msgname);
        msg->setSrc_ppid(-1);//设置发出的processor编号
        msg->setDst_ppid(-1);//设置接收的processor编号
        msg->setFrom_router_port(getNextRouterPortP());//设置收到该msg的Router端口
        msg->setVc_id(-1);//设置VC ID
        msg->setIsHead(isHead); //设置isHead flag
        msg->setFlitCount(-1);
        msg->setFlitGenTime(simTime().dbl());

        return msg;

    }
}

//processor转发的路由算法,processor只有一个prot,直接转发出去即可
void Processor::forwardMessage(FatTreeMsg *msg)
{

    msg->setFrom_router_port(getNextRouterPortP());// 设置收到该信息路由器的端口号
    send(msg,"port$o");
    if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
        EV << "Forwarding message { " << msg << " } from processor to router, VCID = "<<preHeadFlitVCID<<"\n";
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

double Processor::ParetoON() {
    double exp_time = exponential((double)1/ALPHA_ON);
    exp_time = round(exp_time / TimeScale) * CLK_CYCLE;
    if(exp_time < CLK_CYCLE) {
        exp_time = CLK_CYCLE;
    }
    return exp_time;
}

double Processor::ParetoOFF() {
    double exp_time = exponential((double)1/ALPHA_OFF);
    exp_time = round(exp_time / TimeScale) * CLK_CYCLE;
    if(exp_time < CLK_CYCLE) {
        exp_time = CLK_CYCLE;
    }
    return exp_time;
}

double Processor::Poisson() {
    double exp_time = exponential((double)1/LAMBDA);
    exp_time = round(exp_time / TimeScale) * CLK_CYCLE;
    if(exp_time < CLK_CYCLE) {
        exp_time = CLK_CYCLE;
    }
    return exp_time;
}

void Processor::finish()
{
    // This function is called by OMNeT++ at the end of the simulation.
    EV << "Flit Sent: " << numFlitSent << endl;
    EV << "Package Sent: " << numPackageSent << endl;
    EV << "Flit Received: " << numFlitReceived << endl;
    EV << "Package Received: " << numPackageReceived << endl;
    EV << "Dropped:  " << numDropped << endl;
    //EV << "Hop count, min:    " << hopCountStats.getMin() << endl;
    //EV << "Hop count, max:    " << hopCountStats.getMax() << endl;
    //EV << "Hop count, mean:   " << hopCountStats.getMean() << endl;
    //EV << "Hop count, stddev: " << hopCountStats.getStddev() << endl;

    recordScalar("#flit sent", numFlitSent);
    recordScalar("#package sent", numPackageSent);
    recordScalar("#flit received", numFlitReceived);
    recordScalar("#package received", numPackageReceived);
    recordScalar("#flit dropped", numDropped);
    recordScalar("#flitByHop", flitByHop);


    if(getIndex() == 0) {
        double timeCount = simTime().dbl() - Sim_Start_Time;
        recordScalar("#timeCount", timeCount);
    }



}




