/*
 * router.cc
 *
 *  Created on: 2016年7月30日
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
 * msg进来后通过静态的方式选择virtual channel，可以改进
 *
 * Switch Allocator目前也是采用轮循方式，可否改进？
 *
 * flit注入方式研究一下
 *
 * 目前是当个信号包，如果一个包多个flit如何设计？
 * ***********************************************
 */

// 对Router进行建模
class Router : public cSimpleModule
{
  private:
    //long numSent;
    //long numReceived;
    cMessage *selfMsgAlloc;
    cMessage *selfMsgBufferInfo;

    cLongHistogram hopCountStats;
    cOutVector hopCountVector;

    //每个Port的buffer状态
    bool BufferAvailCurrent[PortNum][VC];//当前路由器的buffer状态
    bool BufferAvailConnect[PortNum][VC];//相连路由器对应端口号的buffer状态

    //Input Buffer, Routing
    FatTreeMsg* VChannelMsg[PortNum][VC][BufferDepth]; //virtual channel 的buffer
    //越早到的数据放在ID小的那边，规定好,0表示buffer中第一个出去的数据
    int VCOutPort[PortNum][VC][BufferDepth]; //对应VChannelMsg，存放该msg的输出端口号out_port_id

    //用于VC仲裁
    int VCAllocWinVCID[PortNum];//存放每个port胜利的VChannelMsg的id号
    FatTreeMsg* VCAllocWinMsg[PortNum];//用来存储每个port胜利的VChannelMsg的msg，注意，存在VChannelMsg的msg先不删，最后发出去了再删
    int VCAllocWinOutPort[PortNum];//用来存储每个port胜利的vc的msg的输出port

    //用于SA仲裁
    FatTreeMsg* SAllocWinMsg[PortNum];//存储每个输出端口Switch Allocator仲裁中胜出输入端口号里的msg，PortNum个输出端口需要PortNum个仲裁
    int SAllocWinInPort[PortNum];//存储每个输出端口Switch Allocator仲裁中胜出输入端口号，看清楚，输入端口号，不是VC中胜出的vcid





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
    //初始化行和列的参数
    //WATCH(numSent);
    //WATCH(numReceived);

    hopCountStats.setName("hopCountStats");
    hopCountStats.setRangeAutoUpper(0, 10, 1.5);
    hopCountVector.setName("HopCount");

    //对Buffer进行初始化
    for(int i=0;i<PortNum;i++)
        for(int j=0;j<VC;j++)
            for(int k=0;k<BufferDepth;k++){
                VChannelMsg[i][j][k]=nullptr;
                VCOutPort[i][j][k]=-1;
            }
    //对VC仲裁器进行初始化
    for(int i=0;i<PortNum;i++){
        VCAllocWinVCID[i]=-1;
        VCAllocWinMsg[i]=nullptr;
        VCAllocWinOutPort[i]=-1;

    }

    //对SA仲裁器进行初始化
    for(int i=0;i<PortNum;i++){
        SAllocWinMsg[i]=nullptr;
        SAllocWinInPort[i]=-1;
    }

    //对selfMsg进行初始化
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
        //****************仲裁定时****************************

        if(msg==selfMsgAlloc){//自消息为仲裁定时消息

            scheduleAt(simTime()+CLK_CYCLE,selfMsgAlloc);

            // 3 虚通道仲裁 Virtual Channel Allocation
            //虚通道仲裁采用Round Robin轮循调度算法
            //采用VCAllocWinVCID来轮流指向VC个虚通道
            //对每个端口都进行判决
            for(int i=0;i<PortNum;i++){
                //存在某个输入端口在上一个周期仲裁胜利的vc在sa仲裁时没获得输出端口的仲裁胜出
                //那在这个回合就不要继续仲裁了，沿用上一次的结果
                if(VCAllocWinMsg[i]==nullptr){
                    int vcid=(VCAllocWinVCID[i]+1)%VC;//port i的VChannelMsg指针，从上一次仲裁成功的vcid的下一个vcid开始
                    for(int j=0;j<VC;j++){
                        int vcid_tmp=(vcid+j)%VC;
                        if(VChannelMsg[i][vcid_tmp][0]!=nullptr){
                            VCAllocWinMsg[i]=VChannelMsg[i][vcid_tmp][0];//先存着不删除，发出去后再删
                            VCAllocWinVCID[i]=vcid_tmp;//存着胜利的vcid
                            VCAllocWinOutPort[i]=VCOutPort[i][vcid_tmp][0];//存储胜利的vc的msg的输出端口
                            if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
                                EV<<"VC Allocation >> ROUTER: "<<getIndex()<<"("<<swpid2swlid(getIndex())<<"), INPORT: "<<i<<
                                        ", VCAllocWinVCID: "<<vcid_tmp<<", VCAllocWinMsg: { "<<VCAllocWinMsg[i]<<" }\n";
                            }
                            break;
                        }
                    }
                }
            }



            // 4 交叉开关仲裁 Switch Allocation
            //第二阶段的仲裁主要是对从每个输入端口过来并对同一输出端口提出的请求进行仲裁。
            //当然这些请求都是由输入端某一胜出的虚拟通道发出的。因为有P个输入端口，所以这样的仲裁需要P个输入。
            //这里不用crossbar来实现，用for循环代替crossbar
            for(int i=0;i<PortNum;i++){//对每一个输出端口SAlloc进行循环
                if(SAllocWinMsg[i]==nullptr){//输出端口还未被占用
                    int last_win_inport=(SAllocWinInPort[i]+1)%PortNum;//从上一次仲裁胜利的端口的下一个端口开始进行轮循仲裁
                    for(int j=0;j<PortNum;j++){//PortNum个输入端口
                        int inport_tmp=(last_win_inport+j)%PortNum;
                        if(VCAllocWinOutPort[inport_tmp]==i){//如果inport仲裁胜出的vc的输出port和outport的id一致，则仲裁胜出
                            int vc_id=VCAllocWinVCID[inport_tmp];
                            if(BufferAvailConnect[i][vc_id]==true){ //下个router相应端口的vc还有buffer的情况下
                                SAllocWinMsg[i]=VCAllocWinMsg[inport_tmp];
                                SAllocWinInPort[i]=inport_tmp;
                                if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
                                    EV<<"Switch Allocation >> ROUTER: "<<getIndex()<<"("<<swpid2swlid(getIndex())<<"), OUTPORT: "<<i<<
                                            ", SAllocWinInPort: "<<inport_tmp<<", SAllocWinMsg: { "<<SAllocWinMsg[i]<<" }\n";
                                }
                                break;//如果SAlloc为空，则把获胜的输入端口的msg赋值给它，不空就跳过，最后break跳出循环
                            }

                        }
                    }
                }
            }


            // 5 交叉开关传输 Switch Traversal
            // 对每一个输出端口都转发出msg，同时对存有已转发的msg进行清空
            for(int i=0;i<PortNum;i++){//对每一个输出端口进行转发
                if(SAllocWinMsg[i]!=nullptr){//输出端口有数据
                    int inport=SAllocWinInPort[i];
                    int vcid=VCAllocWinVCID[inport];
                    FatTreeMsg* forward_msg=VChannelMsg[inport][vcid][0];
                    int k=VCOutPort[inport][vcid][0];
                    if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
                        EV<<"Switch Traversal >> ROUTER: "<<getIndex()<<"("<<swpid2swlid(getIndex())<<"), OUTPORT: "<<i<<
                                ", SAllocWinInPort: "<<inport<<", VCAllocWinVCID: "<<vcid<<", VChannelMsg: { "
                                <<forward_msg<<" }, VCOutPort: "<<k<<'\n';
                    }
                    //转发msg
                    forwardMessage(forward_msg,k); //由于router不会sink message, 直接转发即可
                    //清除相应buffer
                    SAllocWinMsg[i]=nullptr;
                    VCAllocWinMsg[inport]=nullptr;
                    VCAllocWinOutPort[inport]=-1;
                    //对输入缓存Buffer进行shift
                    for(int j=0;j<BufferDepth-1;j++){
                        VChannelMsg[inport][vcid][j]=VChannelMsg[inport][vcid][j+1];
                        VCOutPort[inport][vcid][j]=VCOutPort[inport][vcid][j+1];
                    }
                    VChannelMsg[inport][vcid][BufferDepth-1]=nullptr;
                    VCOutPort[inport][vcid][BufferDepth-1]=-1;
                }

            }
        }else{//自消息为buffer更新定时消息
            //******************更新buffer信息定时**********************
            //更新buffer信息
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
            //产生bufferInfo信号并向相应端口发送信号
            for(int i=0;i<PortNum;i++){
                int from_port=getNextRouterPort(i);
                BufferInfoMsg *bufferInfoMsg = new BufferInfoMsg("bufferInfoMsg");
                bufferInfoMsg->setFrom_port(from_port);
                bufferInfoMsg->setBufferAvailArraySize(VC);//arraySize的大小为VC的数量
                //设置buffer信息
                for(int j=0;j<VC;j++){
                    bool avail=BufferAvailCurrent[i][j];
                    bufferInfoMsg->setBufferAvail(j, avail);
                }
                //发送bufferInfoMsg
                forwardBufferInfoMsg(bufferInfoMsg, i);


            }

        }


    }else{ //非自消息，即收到其他路由器的消息
        //*************************收到其他端口buffer更新消息************************
        if(strcmp("bufferInfoMsg", msg->getName()) == 0){
            //收到的消息为buffer状态消息，更新BufferAvailConnect[PortNum][VC]

            BufferInfoMsg *bufferInfoMsg = check_and_cast<BufferInfoMsg *>(msg);
            int from_port=bufferInfoMsg->getFrom_port();
            //更新BufferAvailConnect[PortNum][VC]
            for(int j=0;j<VC;j++){
                BufferAvailConnect[from_port][j]=bufferInfoMsg->getBufferAvail(j);
            }
            if (Verbose >= VERBOSE_BUFFER_INFO_MESSAGES) {
                EV<<"Receiving bufferInfoMsg, Updating buffer state >> ROUTER: "<<getIndex()<<"("<<swpid2swlid(getIndex())<<"), INPORT: "<<from_port<<
                    ", Received MSG: { "<<bufferInfoMsg<<" }\n";
            }
            delete bufferInfoMsg;


        }else{
            //**********************收到其他端口的fatTreeMsg数据消息*******************
            //收到的消息为FatTreeMsg数据消息

            FatTreeMsg *ftmsg = check_and_cast<FatTreeMsg *>(msg);

            // 1 输入存储 Input Buffer，决定放到哪个virtual channel
            int vc_id = ftmsg->getVc_id(); // 获取vc id， 静态方式设置vc id
            int port_id = ftmsg->getFrom_router_port();//获取收到msg的端口号，决定了放在哪个端口的vc下
            int avil_buff_id=-1;
            //判断buffer是否还有空间，如果buffer还有空间，将收到的信号存到buffer中去
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



            // 2 路由计算 Routing Computation
            int out_port_id=calRoutePort(ftmsg);//计算发出msg的端口号
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
    msg->setFrom_port(getNextRouterPort(k));//设置接受该msg的Router的port端口号
    send(msg,str1);
    int cur_swpid=getIndex();//当前路由器的id
    int cur_swlid=swpid2swlid(cur_swpid);
    if (Verbose >= VERBOSE_BUFFER_INFO_MESSAGES) {
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

