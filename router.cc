/*
 * router.cc
 *
 *  Created on: 2016年7月30日
 *      Author: Vincent
 *
 *  Function:
 *      路由器：输入缓存，虚通道，虚通道仲裁(Round Robin)，交叉开关仲裁(Round Robin)，交叉开关传输
 *      数据包：Flit形式，Flit长度可调(根据Head Flit决定)
 */

#include <stdio.h>
#include <string.h>
#include <omnetpp.h>
#include <math.h>
#include "fat_tree_pkt_m.h"
#include "fat_tree.h"
#include "buffer_info_m.h"
#include "tech_power.h"

using namespace omnetpp;


/*************************************************
 * TODO:
 *
 *
 *
 * 1.Switch Allocator目前也是采用轮循方式，可否改进？
 *
 *
 * 2.虚通道的功能，一个Input channel有2个虚通道，其中一个仲裁胜利，但阻塞住了，另外一个应该可以通行。现在仲裁单位是package，会阻塞另外一个通道，需要改进
 * 输出端口仲裁，输入端口数据的数据以package为单位，否则会发生数据混淆。但是虚通道的仲裁不需要以package为单位，对每个虚通道设置一个寄存器
 * 用来保存该虚通道这个package的输出端口是哪个，然后虚通道仲裁用轮巡，可以防止一个虚通道阻塞时，其他虚通道可用
 *
 * 3.虚通道的问题，虚通道数量是否与输出端口数量一致，用来存放对应输出端口的数据
 *
 *
 * Feature:
 * 1. Credit-based flow control
 * 2. 数据包的Head Flit和Body Flit都是具有VCID，链路和缓存的分配以Flit为单位
 * 3. 互联资源buffer，channel，以flit为单位进行分配
 * 4. 某个packet阻塞住某个virtual channel后（可能阻塞好几个node），其他vc仍可以被利用
 * 5. 虚通道分配以packet为单位，一个packet占用一个vc（同一packet的不同flit占用一个固定的vc），（注意和buffer分配以flit为单位区分），物理信道分配以flit为单位
 * 6. 每个输出端口需要寄存器记录与它相连的下个路由器的输入端口的vc状态，记录输入buffer中lane是否空闲，有多少空闲flit buffer
 * 7. 输入端口的virtual channel需要以下几个状态：free，waiting（等待被分配），active
 * 8. 一个virtual channel只能被1个packet所占有
 * 9. 路由计算，head flit到达虚通道fifo头部才计算
 * 10. crossbar输入不耦合，输出耦合（输入vc对输出端口竞争，如果输入和输出都耦合，输入端口和输出端口需要同时仲裁）
 * 11. round robin轮巡
 * 12. 输入端口vc buffer深度为BufferDepth，输出端口vc buffer深度为1
 * 13. 发送反向流控信息，通过独立传输线
 * 14. 输出端口对所有请求该输出端口的输入vc进行仲裁（以输入vc为基本单位）
 *
 * 整个路由器仲裁的流程：
 *
 * 1. Routing Logic，路由算法决定输出端口和vc，routing function返回单个output vc（也可返回多个，但是仲裁会变得复杂）
 * 2. Virtual Channel Allocation， P个输出端口的V个Virtual Channel都有其VC Arbiter，一共PV个，仲裁器的输入为P*V，对请求同一输出端口的同一vc的不同input vc进行仲裁，选出一个胜利的input
 *  vc（共PV个input vc）
 * 3. Switch Arbitration， 两个阶段，输入端口V：1仲裁，第二阶段输出端口P：1仲裁（基于第一阶段胜利的vc），这种仲裁方式会浪费带宽资源
 * 4. 交叉开关传输
 * 注意：Head Flit有：RC，VA，SA，ST；而Body Flit只有：SA，ST
 * ***********************************************
 */


// 对Router进行建模
class Router : public cSimpleModule
{
  private:
    //long numSent;
    //long numReceived;
    cMessage* selfMsgAlloc; //message仲裁定时信号
    //cMessage *selfMsgBufferInfo; //buffer通知定时信号

    //cLongHistogram hopCountStats;
    //cOutVector hopCountVector;

    //每个Port的buffer状态
    //bool BufferAvailCurrent[PortNum][VC];//当前路由器的buffer状态
    //bool BufferAvailConnect[PortNum][VC];//相连路由器对应端口号的下一跳路由器buffer状态
    int BufferConnectCredit[PortNum][VC]; //连接路由器端口的buffer的credit，即空闲缓存大小

    //Step 1. Input Buffer
    //Input Buffer, Output Buffer, VC State
    FatTreePkt* InputBuffer[PortNum][VC][BufferDepth]; //输入端口virtual channel的buffer,里面存放收到的Flit信息
    FatTreePkt* OutputBuffer[PortNum][VC]; //输出端口的virtual channel的buffer，深度为1

    //看看能否删除
    //int InputVCFlitCount[PortNum][VC]; //输入端口每个虚通道队投packet的剩余flit

    //Step 2. Routing Logic
    int RCInputVCState[PortNum][VC]; //-1代表没有分配结果，即对应的vc没有数据，否则表示被分配的output vc标号，port_number * vc + vc_id
    //越早到的数据放在ID小的那边，规定好,0表示buffer中第一个出去的数据
    //int VCOutPort[PortNum][VC][BufferDepth]; //对应VChannelMsg，存放该msg的输出端口号out_port_id

    //Step 3. Virtual Channel Allocation
    int VAOutputVCState[PortNum][VC]; //-1代表output vc闲置，否则表示被分配的input vc标号，port_number * vc + vc_id，输出端口vc被锁住后，下一跳的输入端口对应的vc就被锁住
    int VAOutputVCStatePre[PortNum][VC]; //记录上一次仲裁胜利的输入端口虚通道标号，用于Round Robin仲裁法
    bool VAInputVCState[PortNum][VC]; //0代表VCA失败，1代表VCA成功，请求的output vc在RCInputVCState中


    //Step 4. Switch Arbitration
    int SAInputWinVcid[PortNum]; //保存胜利的vcid,仲裁失败保存-1，不需要重置
    int SAOutputWin[PortNum]; //保存胜利的input port，仲裁失败保存-1，不需要重置

    //bufferInfoMsg Queue
    cQueue bufTxQueue[PortNum]; //发送bufferInfoMsg数据队列






//    int VCInportVCID[PortNum]; //存放每个端口进入的Flit的VCID，VCID由Head Flit确定
//    //int VCOutport[PortNum][VC]; //存放每个输入端口的Virtual Channel的进入Crossbar的flit的输出端口
//    int VCInportFlitCount[PortNum]; //存放每个输入端口Head Flit的Flit Count，用于计数，0表示剩余的flit为0个，已传完
//
//
//
//
//    //用于VC仲裁
//    int VCAllocWinVCID[PortNum];//存放每个port胜利的VChannelMsg的id号
//    //FatTreeMsg* VCAllocWinMsg[PortNum];//用来存储每个port胜利的VChannelMsg的msg，注意，存在VChannelMsg的msg先不删，最后发出去了再删
//    int VCAllocWinOutPort[PortNum];//用来存储每个port胜利的vc的msg的输出port
//    int VCAllocWinFlitCount[PortNum];//用于存储每个Port胜利的VC的Flit Count
//
//    //用于SA仲裁
//    //FatTreeMsg* SAllocWinMsg[PortNum];//存储每个输出端口Switch Allocator仲裁中胜出输入端口号里的msg，PortNum个输出端口需要PortNum个仲裁
//    int SAllocWinInPort[PortNum];//存储每个输出端口Switch Allocator仲裁中胜出输入端口号，看清楚，输入端口号，不是VC中胜出的vcid
//    int SAllocFlitCount[PortNum];//存放输出端口一个packge的flit count，同时用于判断输出端口是否被占用
//    int SAllocNextVCID[PortNum];//存放输出端口要输出Flit的下一个Router的VCID

    double RouterPower;
    double flitReceived; //用于计算toggle rate





  public:
    Router();
    virtual ~Router();
  protected:
    virtual void forwardMessage(FatTreePkt *msg, int out_port_id);
    virtual void forwardBufferInfoMsg(BufferInfoMsg *msg, int out_port_id);
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    virtual int ppid2plid(int ppid);
    virtual int plid2ppid(int plid);
    virtual int swpid2swlid(int swpid);
    virtual int swlid2swpid(int swlid);
    virtual int calRoutePort(FatTreePkt* msg);
    virtual int getPortAndVCID(FatTreePkt* msg);
    virtual int getNextRouterPort(int current_out_port); //计算下一个相连的router的端口
    virtual int getNextRouterAvailVCID(int port_num); //计算下一个节点相应端口可用的virtual channel
    virtual simtime_t channelAvailTime(int port_num);
    virtual bool connectToProcessor(int port_num);
    virtual double getRouterPower();//计算路由器功耗

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


    //对Buffer进行初始化
    for(int i = 0; i < PortNum; i++) {
        SAInputWinVcid[i] = -1;
        SAOutputWin[i] = -1;
        for(int j = 0; j < VC; j++) {
            OutputBuffer[i][j] = nullptr;
            RCInputVCState[i][j] = -1;
            BufferConnectCredit[i][j] = BufferDepth;
            VAOutputVCState[i][j] = -1;
            VAOutputVCStatePre[i][j] = -1;
            VAInputVCState[i][j] = false;
            //InputVCFlitCount[i][j] = 0;
            for(int k = 0; k < BufferDepth; k++){
                InputBuffer[i][j][k] = nullptr;
            }
        }
    }


//    for(int i=0;i<PortNum;i++){
//        VCInportVCID[i]=-1;
//        VCInportFlitCount[i]=0;
//        //for(int j=0;j<VC;j++){
//        //    VCOutport[i][j]=-1;
//        //}
//    }
//
//    //对VC仲裁器进行初始化
//    for(int i=0;i<PortNum;i++){
//        VCAllocWinVCID[i]=-1;
//        //VCAllocWinMsg[i]=nullptr;
//        VCAllocWinOutPort[i]=-1;
//        VCAllocWinFlitCount[i]=0;
//    }
//
//    //对SA仲裁器进行初始化
//    for(int i=0;i<PortNum;i++){
//        //SAllocWinMsg[i]=nullptr;
//        SAllocWinInPort[i]=-1;
//        SAllocFlitCount[i]=0;
//        SAllocNextVCID[i]=-1;
//    }

    //对selfMsg进行初始化
    selfMsgAlloc = new cMessage("selfMsgAlloc");
    scheduleAt(Sim_Start_Time, selfMsgAlloc);

    RouterPower = 0;
    flitReceived = 0;

}

void Router::handleMessage(cMessage *msg)
{

    if (msg->isSelfMessage()) {
        //****************仲裁定时****************************

        if(msg==selfMsgAlloc){//自消息为仲裁定时消息

            scheduleAt(simTime()+CLK_CYCLE, selfMsgAlloc);



            //Step 2. Routing Logic
            for(int i = 0; i < PortNum; i++) {
                for(int j = 0; j < VC; j++) {
                    FatTreePkt* current_pkt = InputBuffer[i][j][0];
                    //InputBuffer队头的Pkt还没有经过Routing Computing，-1代表还没RC过
                    if(current_pkt != nullptr && current_pkt->getIsHead() == true && RCInputVCState[i][j] == -1) { //数据包有可能阻塞在队头
                        RCInputVCState[i][j] = getPortAndVCID(current_pkt);
                        if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
                            EV<<"Step 2. Routing Computation >> ROUTER: "<<getIndex()<<"("<<swpid2swlid(getIndex())<<"), INPORT: "<<i<<
                                    ", INPORT VCID: "<<j<<", Routing Result: OUTPORT: "<<RCInputVCState[i][j]/VC<<
                                    ", OUTPUT VCID: "<<RCInputVCState[i][j]%VC<<", Msg: { "<<InputBuffer[i][j][0]<<" }\n";
                        }
                    }
                }
            }

            //Step 3. Virtual Channel Allocation
            //每一个输出端口的每一个virtual channel要对每个输入端口的每个virtual channel进行仲裁，只有一个胜利
            //外循环，对每个output virtual channel进行循环
            for(int i = 0; i < PortNum; i++) {
                for(int j = 0; j < VC; j++) {
                    //内循环，对每个input virtual channel进行判断
                    if(VAOutputVCState[i][j] >= 0) continue; //若输出端口vc已被分配，则跳过下面循环
                    bool flag = false;
                    int port_vc = VAOutputVCStatePre[i][j] + 1;
                    int count = 0, total = PortNum * VC;
                    for(; count < total && flag == false; count++, port_vc++) {
                        int m = (port_vc / VC) % PortNum;
                        int n = port_vc % VC;
                        if(RCInputVCState[m][n] == i * VC + j) {//该输入虚通道里面的数据一定是head flit
                            if(VAInputVCState[m][n] == true) {
                                EV << "Error: Step 3. VC Allocation >> ROUTER: " << getIndex()<<"("<<swpid2swlid(getIndex()) << "), VAInputVCState != false" << endl;
                            }
                            VAInputVCState[m][n] = true; //该虚通道仲裁胜利
                            VAOutputVCStatePre[i][j] = m * VC + n;
                            VAOutputVCState[i][j] = m * VC + n;
                            flag = true;
                            if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
                                EV<<"Step 3. VC Allocation >> ROUTER: "<<getIndex()<<"("<<swpid2swlid(getIndex())<<"), WIN INPORT: "<<m<<
                                        ", WIN INPORT VCID: "<<n<<", OUTPORT: "<<i<<
                                        ", OUTPORT VCID: "<<j<<", Win Msg: { "<<InputBuffer[m][n][0]<<" }\n";
                            }
                        }
                    }
                }
            }

            //Step 4. Switch Arbitration
            //在VAInputVCState中为true的输入vc中做选择，而且其对应的输出端口的vc的buffer必须为空，能够容纳它
            //4.1. 输入端口仲裁
            for(int i = 0; i < PortNum; i++) {
                int last_vcid = SAInputWinVcid[i] + 1;
                bool flag = false;
                for(int j = 0; j < VC && flag == false; j++, last_vcid++) {
                    int cur_vcid = last_vcid % VC; //Round Robin
                    //必须保证输入虚通道在虚通道仲裁中胜利（即输出虚通道为次输入虚通道保留）
                    //此外还需要保证输入buffer有数据，可能head flit传输胜利，但是body flit还没过来
                    if(VAInputVCState[i][cur_vcid] == true && InputBuffer[i][cur_vcid][0] != nullptr) { //之前写成InputBuffer[i][cur_vcid] != nullptr，查了3天，泪崩
                        int port = RCInputVCState[i][cur_vcid] / VC;
                        int out_vcid = RCInputVCState[i][cur_vcid] % VC;
                        if(OutputBuffer[port][out_vcid] == nullptr) {
                            SAInputWinVcid[i] = cur_vcid;
                            flag = true;
                            if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
                                EV<<"Step 4.1 Switch Allocation, Input Port Stage >> ROUTER: "<<getIndex()<<"("<<swpid2swlid(getIndex())<<"), INPORT: "<<i<<
                                        ", WIN VCID: "<<cur_vcid<<
                                        ", Win Msg: { "<<InputBuffer[i][cur_vcid][0]<<" }\n";
                            }
                        }
                    }
                }
                if(flag == false) {
                    SAInputWinVcid[i] = -1; //没有合适的虚通道
                }
            }
            //4.2. 输出端口仲裁
            for(int i = 0; i < PortNum; i++) {
                //内循环，对每一个输入端口判断是否请求该输出端口
                int last_inport = SAOutputWin[i] + 1; //Round Robin
                bool flag = false;
                for(int j = 0; j < PortNum && flag == false; j++, last_inport++) {
                    int cur_inport = last_inport % PortNum;
                    if(SAInputWinVcid[cur_inport] != -1) {
                        int inport_vcid = SAInputWinVcid[cur_inport];
                        int out_port = RCInputVCState[cur_inport][inport_vcid] / VC;
                        if(out_port == i) {
                            SAOutputWin[i] = cur_inport;
                            flag = true;
                            if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
                                EV<<"Step 4.2 Switch Allocation, Output Port Stage >> ROUTER: "<<getIndex()<<"("<<swpid2swlid(getIndex())<<"), OUTPORT: "<<i<<
                                        ", WIN INPORT: "<<cur_inport<<" }\n";
                            }
                        }

                    }
                }
                if(flag == false) {
                    SAOutputWin[i] = -1; //仲裁失败
                }
            }

            //Step 5. Switch Traversal
            //交叉开关传输，传输完数据后必须把相关的状态reset，传输完的标志是把输入buffer中的tail flit传输到输出buffer中
            //pkt从输入buffer到输出buffer需要改变pkt中的vcid，改为下一跳路由器输入buffer的vcid（和这一跳路由器输出的vcid相同）
            for(int i = 0; i < PortNum; i++) {
                if(SAOutputWin[i] == -1) continue;
                int inport = SAOutputWin[i];
                int inport_vcid = SAInputWinVcid[inport];
                int output_vcid = RCInputVCState[inport][inport_vcid] % VC;
                FatTreePkt* current_pkt = InputBuffer[inport][inport_vcid][0];

                //修改VCID
                //有可能body flit还没传输过来，导致current_pkt == nullptr
                //可能有bug
                if(current_pkt == nullptr) {
                    EV << "Error in Step 5. Switch Traversal in Router " << getIndex() << ", current_pkt is null" << endl;
                    continue;
                }
                current_pkt->setVc_id(output_vcid);

                //对输入缓存进行shift
                for(int j = 0; j < BufferDepth - 1; j++){
                    InputBuffer[inport][inport_vcid][j] = InputBuffer[inport][inport_vcid][j+1];
                }
                InputBuffer[inport][inport_vcid][BufferDepth-1] = nullptr;
                //将数据放到输出buffer
                OutputBuffer[i][output_vcid] = current_pkt;

                //每转发input buffer里面的一个flit，就产生一个流控信号，通知上游router，进行increment credit操作
                //先将bufferInfoMsg放入Queue中，由于和Data Pkt共用一个信道，容易发生阻塞，需要队列保存数据
                int from_port = getNextRouterPort(inport);
                BufferInfoMsg* bufferInfoMsg = new BufferInfoMsg("bufferInfoMsg");
                bufferInfoMsg->setFrom_port(from_port);
                bufferInfoMsg->setVcid(inport_vcid);
                bufTxQueue[inport].insert(bufferInfoMsg);



                //判断是否为tail flit，如果是，则重置寄存器状态
                if(current_pkt->getIsTail() == true) {
                    RCInputVCState[inport][inport_vcid] = -1;
                    VAInputVCState[inport][inport_vcid] = false;
                }

                if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
                    EV<<"Step 5. Switch Traversal >> ROUTER: "<<getIndex()<<"("<<swpid2swlid(getIndex())<<"), OUTPORT: "<<i<<
                    ", OUTPORT VCID: "<<output_vcid<<", INPORT: "<<inport<<
                    ", INPORT VCID: "<<inport_vcid<<", InputBuffer: { "<< current_pkt << " }" << '\n';
                }



            }

            //Step 6. Forward Data Message
            //发送数据，需要检查信道是否空闲以及
            for(int i = 0; i < PortNum; i++) {
                if(channelAvailTime(i) <= simTime()) {
                    bool flag = false;
                    for(int j = 0; j < VC && flag == false; j++) {
                        if(OutputBuffer[i][j] != nullptr && BufferConnectCredit[i][j] != 0) {
                            flag = true;
                            FatTreePkt* forward_msg = OutputBuffer[i][j];

                            //对状态寄存器进行重置
                            OutputBuffer[i][j] = nullptr;
                            if(!connectToProcessor(i)) { //与路由器相连
                                BufferConnectCredit[i][j]--;
                            }
                            if(forward_msg->getIsTail() == true) {
                                VAOutputVCState[i][j] = -1;
                            }

                            if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
                                EV<<"Step 6. Forward Message >> ROUTER: "<<getIndex()<<"("<<swpid2swlid(getIndex())<<"), OUTPORT: "<<i<<
                                ", OUTPORT VCID: "<<j<<", Forward Pkt: { " <<forward_msg<<" }" <<'\n';
                            }

                            //发送pkt数据
                            forwardMessage(forward_msg, i);
                        }
                    }
                }
            }

            //Step 7. Forward bufferInfoMsg Message
            for(int i = 0; i < PortNum; i++) {
                if(bufTxQueue[i].isEmpty() || channelAvailTime(i) > simTime()) continue;
                BufferInfoMsg* bufferInfoMsg = (BufferInfoMsg*) bufTxQueue[i].front();
                bufTxQueue[i].pop(); //注意，先pop再发送
                //发送bufferInfoMsg
                forwardBufferInfoMsg(bufferInfoMsg, i);

            }








//            //***********************************************************************************************************
//
//
//            // Step 3 虚通道仲裁 Virtual Channel Allocation
//            //虚通道仲裁采用Round Robin轮循调度算法
//            //采用VCAllocWinVCID来轮流指向VC个虚通道
//            //对每个端口都进行判决
//            for(int i=0;i<PortNum;i++){
//                //如果上一次仲裁胜利的Virtual Channel的Flit还没传输完，即VCAllocWinFlitCount不为0，那么跳过本次仲裁，沿用上一次仲裁结果
//                if(VCAllocWinFlitCount[i] == 0){
//                    int vcid=(VCAllocWinVCID[i]+1)%VC;//port i的VChannelMsg指针，从上一次仲裁成功的vcid的下一个vcid开始
//                    for(int j=0;j<VC;j++){
//                        int vcid_tmp=(vcid+j)%VC;
//                        if(InputBuffer[i][vcid_tmp][0]!=nullptr){ //一定是Head Flit，如果不考虑丢包的话
//                            if(InputBuffer[i][vcid_tmp][0]->getIsHead() == true){
//                                //VCAllocWinMsg[i]=InputBuffer[i][vcid_tmp][0];//先存着不删除，发出去后再删
//                                VCAllocWinVCID[i]=vcid_tmp;//存着仲裁胜利的vcid
//
//                                // Step 2 路由计算 Routing Computation, 在VC仲裁胜利后再做路由计算，节省存储资源
//                                // 如果flit一进入vc就进行路由计算，那1个vc里面如果有多个packet，那么就需要多个寄存器
//                                // 来保存路由计算的结果；相反，如果在vc的出口进行路由计算，那么只需要1个寄存器就能保存结果；
//                                // 如果虚通道的数量根据输出端口来确定，每个虚通道对应输出的端口号，那么需要一进入就进行路由计算，
//                                // 确定输出端口号
//                                VCAllocWinOutPort[i]=calRoutePort(InputBuffer[i][vcid_tmp][0]);//存储胜利的vc的msg的输出端口
//                                VCAllocWinFlitCount[i]=InputBuffer[i][vcid_tmp][0]->getFlitCount();//取得Flit Count
//                                if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
//                                    EV<<"VC Allocation >> ROUTER: "<<getIndex()<<"("<<swpid2swlid(getIndex())<<"), INPORT: "<<i<<
//                                            ", VCAllocWinVCID: "<<vcid_tmp<<", VCAllocWinOutPort: "<<VCAllocWinOutPort[i]<<
//                                            ", Flit Count: "<<VCAllocWinFlitCount[i]<<", Win Msg: { "<<InputBuffer[i][vcid_tmp][0]<<" }\n";
//                                }
//                                break;
//                            }
//
//                        }
//                    }
//                }
//            }
//
//
//
//
//            //Step 4 交叉开关仲裁 Switch Allocation
//            //第二阶段的仲裁主要是对从每个输入端口过来并对同一输出端口提出的请求进行仲裁。
//            //当然这些请求都是由输入端某一胜出的虚拟通道发出的。因为有P个输入端口，所以这样的仲裁需要P个输入。
//            //这里不用crossbar来实现，用for循环代替crossbar
//            for(int i=0;i<PortNum;i++){//对每一个输出端口SAlloc进行循环
//                if(SAllocFlitCount[i] == 0){//输出端口还未被占用
//                    int last_win_inport=(SAllocWinInPort[i]+1)%PortNum;//从上一次仲裁胜利的端口的下一个端口开始进行轮循仲裁
//                    for(int j=0;j<PortNum;j++){//PortNum个输入端口
//                        int inport_tmp=(last_win_inport+j)%PortNum;
//                        if(VCAllocWinOutPort[inport_tmp] == i
//                                && InputBuffer[inport_tmp][VCAllocWinVCID[inport_tmp]][0] != nullptr){
//                            //如果inport仲裁胜出的vc的输出port和outport的id一致，则仲裁胜出,同时需要判断VCAllocWinOutPort存的是
//                            //上一次仲裁胜利的package还是本次仲裁胜利的package，如果为上一次，那VCMsgBuffer对应的buffer为nullptr，跳过这个端口
//
//                            int vc_id=getNextRouterAvailVCID(i);//返回下一跳路由器有buffer的vcid，否则返回-1
//                            if(vc_id != -1){ //下个router相应端口的vc还有buffer的情况下
//                                //SAllocWinMsg[i]=VCAllocWinMsg[inport_tmp];
//                                InputBuffer[inport_tmp][VCAllocWinVCID[inport_tmp]][0]->setVc_id(vc_id);//设置下一个节点的vcid
//                                SAllocNextVCID[i] = vc_id;
//                                SAllocWinInPort[i]=inport_tmp;
//                                SAllocFlitCount[i]=VCAllocWinFlitCount[inport_tmp];
//                                if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
//                                    EV<<"Switch Allocation >> ROUTER: "<<getIndex()<<"("<<swpid2swlid(getIndex())<<"), OUTPORT: "<<i<<
//                                            ", SAllocWinInPort: "<<inport_tmp<<", VCAllocWinVCID: "<<VCAllocWinVCID[inport_tmp]<<
//                                            ", SAllocNextVCID: "<<SAllocNextVCID[i]<<", SAllocFlitCount: "<<SAllocFlitCount[i]<<
//                                            ", SAlloc Win Msg: { "<<InputBuffer[inport_tmp][VCAllocWinVCID[inport_tmp]][0]<<" }\n";
//                                }
//                                break;//break跳出循环
//                            }
//
//                        }
//                    }
//                }
//            }
//
//
//
//            //Step 5 交叉开关传输 Switch Traversal
//            //对每一个输出端口都转发出msg，同时对存有已转发的msg进行清空
//            for(int i=0;i<PortNum;i++){//对每一个输出端口进行转发
//                if(SAllocFlitCount[i] != 0){//输出端口有数据
//                    //先判断下一个Router对应端口对应vc是否有buffer，多个flit一起发时，有可能前面几个flit用完了buffer
//                    int input_port = SAllocWinInPort[i];
//                    int win_vcid = VCAllocWinVCID[input_port];
//                    int nextVCID = SAllocNextVCID[i];
//                    //判断下个路由器是否有空间接受此Flit
//                    if(BufferConnectCredit[i][nextVCID] != 0){
//                        FatTreeMsg* forward_msg=InputBuffer[input_port][win_vcid][0];
//                        int outport = VCAllocWinOutPort[input_port]; //output应该等于i
//                        if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
//                            EV<<"Switch Traversal >> ROUTER: "<<getIndex()<<"("<<swpid2swlid(getIndex())<<"), OUTPORT: "<<i<<
//                                    ", SAllocWinInPort: "<<input_port<<", VCAllocWinVCID: "<<win_vcid<<
//                                    ", nextVCID: "<<nextVCID<<", SAllocFlitCount: "<<SAllocFlitCount[i]<<
//                                    ", InputBuffer: { "
//                                    <<forward_msg<<" }, VCAllocWinOutPort: "<<outport<<'\n';
//                        }
//                        //转发msg
//                        forwardMessage(forward_msg,outport); //由于router不会sink message, 直接转发即可
//
//                        //清除相应buffer
//                        //SAllocWinMsg[i]=nullptr;
//                        //VCAllocWinMsg[inport]=nullptr;
//
//                        //更新寄存器信息
//                        VCAllocWinFlitCount[input_port]--;
//                        SAllocFlitCount[i]--;
//
//                        //对输入缓存Buffer进行shift
//                        for(int j=0;j<BufferDepth-1;j++){
//                            InputBuffer[input_port][win_vcid][j]=InputBuffer[input_port][win_vcid][j+1];
//                            //VCOutPort[inport][vcid][j]=VCOutPort[inport][vcid][j+1];
//                        }
//                        InputBuffer[input_port][win_vcid][BufferDepth-1]=nullptr;
//                        //VCOutPort[inport][vcid][BufferDepth-1]=-1;
//
//                        //判断路由器是否和processor相连，如果相连则不需要decrement credit，因为processor的接受能力是无限的
//                        if(!connectToProcessor(outport)){ //与路由器相连
//                            BufferConnectCredit[i][nextVCID]--;
//                        }
//                        if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
//                            EV<<"BufferConnectCredit["<<i<<"]["<<nextVCID<<"]="<<BufferConnectCredit[i][nextVCID]<<"\n";
//                        }
//
//
//                        //每转发input buffer里面的一个flit，就产生一个流控信号，通知上游router，进行increment credit操作
//                        int from_port = getNextRouterPort(input_port);
//                        BufferInfoMsg *bufferInfoMsg = new BufferInfoMsg("bufferInfoMsg");
//                        bufferInfoMsg->setFrom_port(from_port);
//                        bufferInfoMsg->setVcid(win_vcid);
//                        //发送bufferInfoMsg
//                        forwardBufferInfoMsg(bufferInfoMsg, input_port);
//
//                    }
//
//                }
//
//            }

        } // end of selfMsgAlloc

    } // end of selfMsg
    else{ //非自消息，即收到其他路由器的消息
        //*************************收到其他端口buffer更新消息************************
        if(strcmp("bufferInfoMsg", msg->getName()) == 0){
            //收到的消息为buffer状态消息，更新BufferConnectCredit[PortNum][VC]
            BufferInfoMsg* bufferInfoMsg = check_and_cast<BufferInfoMsg*>(msg);
            int from_port = bufferInfoMsg->getFrom_port();
            int vcid = bufferInfoMsg->getVcid();
            BufferConnectCredit[from_port][vcid]++;

            if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
                EV<<"Receiving bufferInfoMsg >> ROUTER: "<<getIndex()<<"("<<swpid2swlid(getIndex())<<"), INPORT: "<<from_port<<
                    ", Received MSG: { "<<bufferInfoMsg<<" }\n";
                EV<<"BufferConnectCredit["<<from_port<<"]["<<vcid<<"]="<<BufferConnectCredit[from_port][vcid]<<"\n";
            }
            delete bufferInfoMsg;


        }else{
            //**********************收到其他端口的FatTreePkt数据消息*******************

            FatTreePkt *ftmsg = check_and_cast<FatTreePkt*>(msg);
            flitReceived += 1;

            //Step 1. Input Buffer
            //决定放到哪个input port的virtual channel
            int input_port = ftmsg->getFrom_router_port();
            int vc_id = ftmsg->getVc_id();

            //由于有流控机制的存在，上一跳路由器向该路由器发送FatTreePkt时，buffer一定有容量来保存数据
            for(int i = 0; i < BufferDepth; i++){
                if(InputBuffer[input_port][vc_id][i] == nullptr){
                    InputBuffer[input_port][vc_id][i] = ftmsg;
                    break;
                }
            }
            if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
                EV<<"Step 1. Input Buffer >> ROUTER: "<<getIndex()<<"("<<swpid2swlid(getIndex())<<"), INPORT: "<<input_port<<
                    ", VCID: "<<vc_id<<", Received MSG: { "<<ftmsg<<" }\n";
            }


//            //判断是否为Head Flit
//            if(ftmsg->getIsHead() == true){
//                int input_port = ftmsg->getFrom_router_port();
//                int vc_id = ftmsg->getVc_id();
//                int flit_count = ftmsg->getFlitCount();
//                VCInportVCID[input_port] = vc_id;
//                VCInportFlitCount[input_port] = flit_count -1; //减去Head Flit，剩余的个数
//                if(InputBuffer[input_port][vc_id][BufferDepth-1]==nullptr){
//                    for(int i=0;i<BufferDepth;i++){
//                        if(InputBuffer[input_port][vc_id][i]==nullptr){
//                            InputBuffer[input_port][vc_id][i]=ftmsg;
//                            break;
//                        }
//                    }
//                    if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
//                        EV<<"Input Buffer >> ROUTER: "<<getIndex()<<"("<<swpid2swlid(getIndex())<<"), INPORT: "<<input_port<<
//                            ", VCID: "<<vc_id<<", Remaining Flit Count: "<<VCInportFlitCount[input_port]<<
//                            ", Received MSG: { "<<ftmsg<<" }\n";
//                    }
//                }else{
//                    //由于上一跳的router在往该router转发flit时已经确定有buffer，因此不会进入到下面这种情况
//                    if (Verbose >= VERBOSE_DEBUG_MESSAGES){
//                        EV<<"Input Buffer and Routing >> ROUTER: "<<getIndex()<<"("<<swpid2swlid(getIndex())<<"), INPORT: "<<input_port<<
//                            ", VCID: "<<vc_id<<", Received Head MSG: { "<<ftmsg<<" }, Buffer is full, dropping message\n";
//                    }
//
//                }
//
//            }else{ //Body/Tail Flit
//                int input_port = ftmsg->getFrom_router_port();
//                int vc_id = VCInportVCID[input_port];
//                VCInportFlitCount[input_port]--;
//
//                if(InputBuffer[input_port][vc_id][BufferDepth-1]==nullptr){
//                    for(int i=0;i<BufferDepth;i++){
//                        if(InputBuffer[input_port][vc_id][i]==nullptr){
//                            InputBuffer[input_port][vc_id][i]=ftmsg;
//                            break;
//                        }
//                    }
//                    if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
//                        EV<<"Input Buffer >> ROUTER: "<<getIndex()<<"("<<swpid2swlid(getIndex())<<"), INPORT: "<<input_port<<
//                            ", VCID: "<<vc_id<<", Remaining Flit Count: "<<VCInportFlitCount[input_port]<<
//                            ", Received MSG: { "<<ftmsg<<" }\n";
//                    }
//                }else{
//                    //由于上一跳的router在往该router转发flit时已经确定有buffer，因此不会进入到下面这种情况
//                    if (Verbose >= VERBOSE_DEBUG_MESSAGES){
//                        EV<<"Input Buffer and Routing >> ROUTER: "<<getIndex()<<"("<<swpid2swlid(getIndex())<<"), INPORT: "<<input_port<<
//                            ", VCID: "<<vc_id<<", Received Body/Tail MSG: { "<<ftmsg<<" }, Buffer is full, dropping message\n";
//                    }
//
//                }
//
//
//            }

        }//end of FatTreePkt

    } // end of Not self msg

}


//调用...计算路由端口
void Router::forwardMessage(FatTreePkt *msg, int out_port_id)
{

    // Increment hop count.
    msg->setHopCount(msg->getHopCount()+1);

    //int k=calRoutePort(msg);//计算发出msg的端口号
    int k = out_port_id;
    char str1[20]="port_";
    char str2[20];
    sprintf(str2, "%d", k);
    strcat(str1,str2);
    strcat(str1,"$o");
    //EV<<"k="<<k<<" str1="<<str1<<" str2="<<str2<<"\n";
    msg->setFrom_router_port(getNextRouterPort(k));//设置接受该msg的Router的port端口号
    send(msg,str1);
    int cur_swpid=getIndex();//当前路由器的id
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
    msg->setFrom_port(getNextRouterPort(k));//设置接受该msg的Router的port端口号
    send(msg,str1);
    int cur_swpid=getIndex();//当前路由器的id
    int cur_swlid=swpid2swlid(cur_swpid);
    if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
        EV << "Forwarding BufferInfoMsg { " << msg << " } from router "<<cur_swpid<<"("<<cur_swlid<<")"<< " through port "<<k<<"\n";
    }

}

//从ppid计算plid
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
//从plid计算ppid
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

//从swpid计算swlid
int Router::swpid2swlid(int swpid){
    //首先判断swpid在哪一层
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
    //已经找到switch所在层，接下来对其进行编码
    //先对非顶层的switch进行编码
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
        IDtmp=mul*level+IDtmp;//最前面加上它的层数
        return IDtmp;
    }
    //接下来对顶层的switch进行操作
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

//swlid转swpid
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

//根据当前router的swpid和msg的dst_ppid来计算转发的端口
int Router::calRoutePort(FatTreePkt *msg){
    int cur_swpid=getIndex();//当前路由器的id
    int cur_swlid=swpid2swlid(cur_swpid);
    int level=cur_swlid/pow(10,LevelNum-1);//Router的level
    int dst_ppid=msg->getDst_ppid();
    int dst_plid=ppid2plid(dst_ppid);
    //EV<<dst_ppid<<" "<<dst_plid<<"\n";
    //判断switch是否为祖先
    int ptmp=dst_plid/pow(10,level+1);
    int ctmp=(cur_swlid%((int)pow(10,LevelNum-1)))/pow(10,level);
    bool isAncestor=(ptmp==ctmp);
    int k;//转发的端口
    //EV<<cur_swpid<<" "<<cur_swlid<<" "<<level<<" "<<dst_ppid<<" "<<dst_plid<<" "<<ptmp<<" "<<ctmp<<"\n";
    //如果switch是dst_ppid的祖先，则向下端口转发，否则向上端口转发
    if(isAncestor){
        //向下转发
        k=(dst_plid/((int)pow(10,level)))%10;//通过端口pl’进行转发
        //EV<<"isAncestor, k="<<k<<"\n";
        return k;
    }else{
        //向上转发
        k=(dst_plid/((int)pow(10,level)))%10+PortNum/2;//k=pl’+m/2
        //EV<<"notAncestor, k="<<k<<" "<<dst_ppid<<" "<<dst_plid<<" "<<(int)pow(10,level)<<" "<<(dst_plid/((int)pow(10,level)))%10<<"\n";
        return k;
    }

}

int Router::getPortAndVCID(FatTreePkt* msg) {
    int port = calRoutePort(msg);
    int best_vcid = 0;
    for(int i = 0; i < VC; i++){
        if(BufferConnectCredit[port][i] > BufferConnectCredit[port][best_vcid]){
            best_vcid = i;
        }
    }
    //return value port * VC + vcid
    return port * VC + best_vcid;
}

//计算收到该msg的路由器的端口号
int Router::getNextRouterPort(int current_out_port){


    int cur_swpid=getIndex();//当前路由器的id
    int cur_swlid=swpid2swlid(cur_swpid);
    int level=cur_swlid/pow(10,LevelNum-1);//Router的level
    bool lowerRouter = (current_out_port>=(PortNum/2))&&(level!=LevelNum-1); //判断是否向上转发，向上转发则为下层router

    int ctmp=(cur_swlid%((int)pow(10,LevelNum-1)));//去除掉level的swlid
    int k;//下一个Router的接受端口
    if(!lowerRouter){//上层的Router
        if(level==0){
            k=0;//level==0时，为上端口，因此msg发到processor，而processor只有一个端口，默认processor的接受端口为0
        }else{
            int lowLevel=level-1;//下层的level
            k = (ctmp/((int) pow(10,lowLevel)))%10+PortNum/2;
        }

    }else{ //下层的Router
        k = (ctmp/((int) pow(10,level)))%10;
    }
    return k;
}

bool Router::connectToProcessor(int port_num) {
    int cur_swpid=getIndex();//当前路由器的id
    int cur_swlid=swpid2swlid(cur_swpid);
    int level=cur_swlid/pow(10,LevelNum-1);//Router的level
    if (level == 0 && port_num < PortNum/2)
        return true;
    return false;
}


int Router::getNextRouterAvailVCID(int port_num){
    int vc_id = intuniform(0,VC-1); //随机分配一个通道，以此vc开始循环判断是否有空的vc，增加随机分布，防止每次都从0开始
    for(int i=0;i<VC;i++){
        int vcid_tmp = (vc_id + i)%VC;
        if(BufferConnectCredit[port_num][vcid_tmp] != 0){
            return vcid_tmp;
        }
    }
    return -1;

}

simtime_t Router::channelAvailTime(int port_num) {
    int k = port_num;
    char str1[20] = "port_";
    char str2[20];
    sprintf(str2, "%d", k);
    strcat(str1, str2);
    strcat(str1, "$o");

    cChannel* txChannel = gate(str1)->getTransmissionChannel();
    simtime_t txFinishTime = txChannel->getTransmissionFinishTime();
    return txFinishTime;
}
double Router::getRouterPower() {

    double timeCount = simTime().dbl() - Sim_Start_Time;
    double clockCount = timeCount / CLK_CYCLE; //时钟周期数
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

