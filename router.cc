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
 * 1.Switch Allocator目前也是采用轮循方式，可否改进？
 *
 *
 * 2.虚通道的功能，一个Input channel有2个虚通道，其中一个仲裁胜利，但阻塞住了，另外一个应该可以通行。现在仲裁单位是package，会阻塞另外一个通道，需要改进
 * 输出端口仲裁，输入端口数据的数据以package为单位，否则会发生数据混淆。但是虚通道的仲裁不需要以package为单位，对每个虚通道设置一个寄存器
 * 用来保存该虚通道这个package的输出端口是哪个，然后虚通道仲裁用轮巡，可以防止一个虚通道阻塞时，其他虚通道可用
 *
 * 3.虚通道的问题，虚通道数量是否与输出端口数量一致，用来存放对应输出端口的数据
 *
 * Feature:
 * 1. Credit-based flow control
 * ***********************************************
 */


// 对Router进行建模
class Router : public cSimpleModule
{
  private:
    //long numSent;
    //long numReceived;
    cMessage *selfMsgAlloc; //message仲裁定时信号
    //cMessage *selfMsgBufferInfo; //buffer通知定时信号

    //cLongHistogram hopCountStats;
    //cOutVector hopCountVector;

    //每个Port的buffer状态
    //bool BufferAvailCurrent[PortNum][VC];//当前路由器的buffer状态
    //bool BufferAvailConnect[PortNum][VC];//相连路由器对应端口号的下一跳路由器buffer状态
    int BufferConnectCredit[PortNum][VC]; //连接路由器端口的buffer的credit，即空闲缓存大小

    //Input Buffer, Routing
    FatTreeMsg* VCMsgBuffer[PortNum][VC][BufferDepth]; //virtual channel的buffer,里面存放收到的Flit信息
    //越早到的数据放在ID小的那边，规定好,0表示buffer中第一个出去的数据
    //int VCOutPort[PortNum][VC][BufferDepth]; //对应VChannelMsg，存放该msg的输出端口号out_port_id

    int VCInportVCID[PortNum]; //存放每个端口进入的Flit的VCID，VCID由Head Flit确定
    //int VCOutport[PortNum][VC]; //存放每个输入端口的Virtual Channel的进入Crossbar的flit的输出端口
    int VCInportFlitCount[PortNum]; //存放每个输入端口Head Flit的Flit Count，用于计数，0表示剩余的flit为0个，已传完


    //用于VC仲裁
    int VCAllocWinVCID[PortNum];//存放每个port胜利的VChannelMsg的id号
    //FatTreeMsg* VCAllocWinMsg[PortNum];//用来存储每个port胜利的VChannelMsg的msg，注意，存在VChannelMsg的msg先不删，最后发出去了再删
    int VCAllocWinOutPort[PortNum];//用来存储每个port胜利的vc的msg的输出port
    int VCAllocWinFlitCount[PortNum];//用于存储每个Port胜利的VC的Flit Count

    //用于SA仲裁
    //FatTreeMsg* SAllocWinMsg[PortNum];//存储每个输出端口Switch Allocator仲裁中胜出输入端口号里的msg，PortNum个输出端口需要PortNum个仲裁
    int SAllocWinInPort[PortNum];//存储每个输出端口Switch Allocator仲裁中胜出输入端口号，看清楚，输入端口号，不是VC中胜出的vcid
    int SAllocFlitCount[PortNum];//存放输出端口一个packge的flit count，同时用于判断输出端口是否被占用
    int SAllocNextVCID[PortNum];//存放输出端口要输出Flit的下一个Router的VCID

    double RouterPower;

    double flitReceived; //用于计算toggle rate





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
    virtual int getNextRouterPort(int current_out_port); //计算下一个相连的router的端口
    virtual int getNextRouterAvailVCID(int port_num);//计算下一个节点相应端口可用的virtual channel
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
    // Initialize variables
    //numSent = 0;
    //numReceived = 0;
    //初始化行和列的参数
    //WATCH(numSent);
    //WATCH(numReceived);

    //hopCountStats.setName("hopCountStats");
    //hopCountStats.setRangeAutoUpper(0, 10, 1.5);
    //hopCountVector.setName("HopCount");

    //对Buffer进行初始化
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

    //对VC仲裁器进行初始化
    for(int i=0;i<PortNum;i++){
        VCAllocWinVCID[i]=-1;
        //VCAllocWinMsg[i]=nullptr;
        VCAllocWinOutPort[i]=-1;
        VCAllocWinFlitCount[i]=0;
    }

    //对SA仲裁器进行初始化
    for(int i=0;i<PortNum;i++){
        //SAllocWinMsg[i]=nullptr;
        SAllocWinInPort[i]=-1;
        SAllocFlitCount[i]=0;
        SAllocNextVCID[i]=-1;
    }

    //对selfMsg进行初始化
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
        //****************仲裁定时****************************

        if(msg==selfMsgAlloc){//自消息为仲裁定时消息

            scheduleAt(simTime()+CLK_CYCLE,selfMsgAlloc);


            // Step 3 虚通道仲裁 Virtual Channel Allocation
            //虚通道仲裁采用Round Robin轮循调度算法
            //采用VCAllocWinVCID来轮流指向VC个虚通道
            //对每个端口都进行判决
            for(int i=0;i<PortNum;i++){
                //如果上一次仲裁胜利的Virtual Channel的Flit还没传输完，即VCAllocWinFlitCount不为0，那么跳过本次仲裁，沿用上一次仲裁结果
                if(VCAllocWinFlitCount[i] == 0){
                    int vcid=(VCAllocWinVCID[i]+1)%VC;//port i的VChannelMsg指针，从上一次仲裁成功的vcid的下一个vcid开始
                    for(int j=0;j<VC;j++){
                        int vcid_tmp=(vcid+j)%VC;
                        if(VCMsgBuffer[i][vcid_tmp][0]!=nullptr){ //一定是Head Flit，如果不考虑丢包的话
                            if(VCMsgBuffer[i][vcid_tmp][0]->getIsHead() == true){
                                //VCAllocWinMsg[i]=VCMsgBuffer[i][vcid_tmp][0];//先存着不删除，发出去后再删
                                VCAllocWinVCID[i]=vcid_tmp;//存着仲裁胜利的vcid

                                // Step 2 路由计算 Routing Computation, 在VC仲裁胜利后再做路由计算，节省存储资源
                                // 如果flit一进入vc就进行路由计算，那1个vc里面如果有多个packet，那么就需要多个寄存器
                                // 来保存路由计算的结果；相反，如果在vc的出口进行路由计算，那么只需要1个寄存器就能保存结果；
                                // 如果虚通道的数量根据输出端口来确定，每个虚通道对应输出的端口号，那么需要一进入就进行路由计算，
                                // 确定输出端口号
                                VCAllocWinOutPort[i]=calRoutePort(VCMsgBuffer[i][vcid_tmp][0]);//存储胜利的vc的msg的输出端口
                                VCAllocWinFlitCount[i]=VCMsgBuffer[i][vcid_tmp][0]->getFlitCount();//取得Flit Count
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




            //Step 4 交叉开关仲裁 Switch Allocation
            //第二阶段的仲裁主要是对从每个输入端口过来并对同一输出端口提出的请求进行仲裁。
            //当然这些请求都是由输入端某一胜出的虚拟通道发出的。因为有P个输入端口，所以这样的仲裁需要P个输入。
            //这里不用crossbar来实现，用for循环代替crossbar
            for(int i=0;i<PortNum;i++){//对每一个输出端口SAlloc进行循环
                if(SAllocFlitCount[i] == 0){//输出端口还未被占用
                    int last_win_inport=(SAllocWinInPort[i]+1)%PortNum;//从上一次仲裁胜利的端口的下一个端口开始进行轮循仲裁
                    for(int j=0;j<PortNum;j++){//PortNum个输入端口
                        int inport_tmp=(last_win_inport+j)%PortNum;
                        if(VCAllocWinOutPort[inport_tmp] == i
                                && VCMsgBuffer[inport_tmp][VCAllocWinVCID[inport_tmp]][0] != nullptr){
                            //如果inport仲裁胜出的vc的输出port和outport的id一致，则仲裁胜出,同时需要判断VCAllocWinOutPort存的是
                            //上一次仲裁胜利的package还是本次仲裁胜利的package，如果为上一次，那VCMsgBuffer对应的buffer为nullptr，跳过这个端口

                            int vc_id=getNextRouterAvailVCID(i);//返回下一跳路由器有buffer的vcid，否则返回-1
                            if(vc_id != -1){ //下个router相应端口的vc还有buffer的情况下
                                //SAllocWinMsg[i]=VCAllocWinMsg[inport_tmp];
                                VCMsgBuffer[inport_tmp][VCAllocWinVCID[inport_tmp]][0]->setVc_id(vc_id);//设置下一个节点的vcid
                                SAllocNextVCID[i] = vc_id;
                                SAllocWinInPort[i]=inport_tmp;
                                SAllocFlitCount[i]=VCAllocWinFlitCount[inport_tmp];
                                if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
                                    EV<<"Switch Allocation >> ROUTER: "<<getIndex()<<"("<<swpid2swlid(getIndex())<<"), OUTPORT: "<<i<<
                                            ", SAllocWinInPort: "<<inport_tmp<<", VCAllocWinVCID: "<<VCAllocWinVCID[inport_tmp]<<
                                            ", SAllocNextVCID: "<<SAllocNextVCID[i]<<", SAllocFlitCount: "<<SAllocFlitCount[i]<<
                                            ", SAlloc Win Msg: { "<<VCMsgBuffer[inport_tmp][VCAllocWinVCID[inport_tmp]][0]<<" }\n";
                                }
                                break;//break跳出循环
                            }

                        }
                    }
                }
            }



            //Step 5 交叉开关传输 Switch Traversal
            //对每一个输出端口都转发出msg，同时对存有已转发的msg进行清空
            for(int i=0;i<PortNum;i++){//对每一个输出端口进行转发
                if(SAllocFlitCount[i] != 0){//输出端口有数据
                    //先判断下一个Router对应端口对应vc是否有buffer，多个flit一起发时，有可能前面几个flit用完了buffer
                    int input_port = SAllocWinInPort[i];
                    int win_vcid = VCAllocWinVCID[input_port];
                    int nextVCID = SAllocNextVCID[i];
                    //判断下个路由器是否有空间接受此Flit
                    if(BufferConnectCredit[i][nextVCID] != 0){
                        FatTreeMsg* forward_msg=VCMsgBuffer[input_port][win_vcid][0];
                        int outport = VCAllocWinOutPort[input_port]; //output应该等于i
                        if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
                            EV<<"Switch Traversal >> ROUTER: "<<getIndex()<<"("<<swpid2swlid(getIndex())<<"), OUTPORT: "<<i<<
                                    ", SAllocWinInPort: "<<input_port<<", VCAllocWinVCID: "<<win_vcid<<
                                    ", nextVCID: "<<nextVCID<<", SAllocFlitCount: "<<SAllocFlitCount[i]<<
                                    ", VCMsgBuffer: { "
                                    <<forward_msg<<" }, VCAllocWinOutPort: "<<outport<<'\n';
                        }
                        //转发msg
                        forwardMessage(forward_msg,outport); //由于router不会sink message, 直接转发即可

                        //清除相应buffer
                        //SAllocWinMsg[i]=nullptr;
                        //VCAllocWinMsg[inport]=nullptr;

                        //更新寄存器信息
                        VCAllocWinFlitCount[input_port]--;
                        SAllocFlitCount[i]--;

                        //对输入缓存Buffer进行shift
                        for(int j=0;j<BufferDepth-1;j++){
                            VCMsgBuffer[input_port][win_vcid][j]=VCMsgBuffer[input_port][win_vcid][j+1];
                            //VCOutPort[inport][vcid][j]=VCOutPort[inport][vcid][j+1];
                        }
                        VCMsgBuffer[input_port][win_vcid][BufferDepth-1]=nullptr;
                        //VCOutPort[inport][vcid][BufferDepth-1]=-1;

                        //判断路由器是否和processor相连，如果相连则不需要decrement credit，因为processor的接受能力是无限的
                        if(!connectToProcessor(outport)){ //与路由器相连
                            BufferConnectCredit[i][nextVCID]--;
                        }
                        if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
                            EV<<"BufferConnectCredit["<<i<<"]["<<nextVCID<<"]="<<BufferConnectCredit[i][nextVCID]<<"\n";
                        }


                        //每转发input buffer里面的一个flit，就产生一个流控信号，通知上游router，进行increment credit操作
                        int from_port = getNextRouterPort(input_port);
                        BufferInfoMsg *bufferInfoMsg = new BufferInfoMsg("bufferInfoMsg");
                        bufferInfoMsg->setFrom_port(from_port);
                        bufferInfoMsg->setVcid(win_vcid);
                        //发送bufferInfoMsg
                        forwardBufferInfoMsg(bufferInfoMsg, input_port);

                    }

                }

            }
        }


    }else{ //非自消息，即收到其他路由器的消息
        //*************************收到其他端口buffer更新消息************************
        if(strcmp("bufferInfoMsg", msg->getName()) == 0){
            //收到的消息为buffer状态消息，更新BufferAvailConnect[PortNum][VC]

            BufferInfoMsg *bufferInfoMsg = check_and_cast<BufferInfoMsg *>(msg);
            int from_port=bufferInfoMsg->getFrom_port();
            //更新BufferAvailConnect[PortNum][VC]
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
            //**********************收到其他端口的fatTreeMsg数据消息*******************
            //收到的消息为FatTreeMsg数据消息

            FatTreeMsg *ftmsg = check_and_cast<FatTreeMsg *>(msg);
            flitReceived += 1;

            //Step 1 输入存储 Input Buffer，决定放到哪个virtual channel
            //判断是否为Head Flit
            if(ftmsg->getIsHead() == true){
                int input_port = ftmsg->getFrom_router_port();
                int vc_id = ftmsg->getVc_id();
                int flit_count = ftmsg->getFlitCount();
                VCInportVCID[input_port] = vc_id;
                VCInportFlitCount[input_port] = flit_count -1; //减去Head Flit，剩余的个数
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
                    //由于上一跳的router在往该router转发flit时已经确定有buffer，因此不会进入到下面这种情况
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
                    //由于上一跳的router在往该router转发flit时已经确定有buffer，因此不会进入到下面这种情况
                    if (Verbose >= VERBOSE_DEBUG_MESSAGES){
                        EV<<"Input Buffer and Routing >> ROUTER: "<<getIndex()<<"("<<swpid2swlid(getIndex())<<"), INPORT: "<<input_port<<
                            ", VCID: "<<vc_id<<", Received Body/Tail MSG: { "<<ftmsg<<" }, Buffer is full, dropping message\n";
                    }

                }


            }

            //2 路由计算放到Virtual Channel Allocation里面进行
            /*
            int vc_id = ftmsg->getVc_id(); // 获取vc id， 静态方式设置vc id
            int port_id = ftmsg->getFrom_router_port();//获取收到msg的端口号，决定了放在哪个端口的vc下
            int avil_buff_id=-1;
            //判断buffer是否还有空间，如果buffer还有空间，将收到的信号存到buffer中去




            // 2 路由计算 Routing Computation
            int out_port_id=calRoutePort(ftmsg);//计算发出msg的端口号
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


//调用...计算路由端口
void Router::forwardMessage(FatTreeMsg *msg, int out_port_id)
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
int Router::calRoutePort(FatTreeMsg *msg){
    int cur_swpid=getIndex();//当前路由器的id
    int cur_swlid=swpid2swlid(cur_swpid);
    int level=cur_swlid/pow(10,LevelNum-1);//Router的level
    int dst_ppid=msg->getDst_ppid();
    int dst_plid=ppid2plid(dst_ppid);
    //EV<<dst_ppid<<" "<<dst_plid<<"\n";
    //判断switch是否为祖先
    int ptmp=dst_plid/pow(10,level+1);//
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

