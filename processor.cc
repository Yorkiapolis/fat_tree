/*
 * processor.cc
 *
 *  Created on: 2016��7��30��
 *      Author: Vincent
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
    cMessage *selfMsgGenMsg;
    cMessage *selfMsgBufferInfoP;//Processor��bufer���¶�ʱ�źţ���������������router������vcͨ����Ϊavail

    long numSent;
    long numReceived;
    cLongHistogram hopCountStats;
    cOutVector hopCountVector;

    bool BufferAvailCurrentP[VC];//��ǰProcessor��buffer״̬��Ĭ�϶�Ϊavailable
    bool BufferAvailConnectP[VC];//����Processor�˿ڵ�Router��buffer״̬

  public:
    Processor();
    virtual ~Processor();
  protected:
    virtual FatTreeMsg *generateMessage();
    virtual void forwardMessage(FatTreeMsg *msg);
    virtual void forwardBufferInfoMsgP(BufferInfoMsg *msg);
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    virtual int ppid2plid(int ppid);
    virtual int plid2ppid(int plid);
    virtual int getNextRouterPortP(); //������processor������router�Ķ˿�


    // The finish() function is called by OMNeT++ at the end of the simulation:
    virtual void finish() override;
};

Define_Module(Processor);

Processor::Processor(){
    selfMsgGenMsg=nullptr;
    selfMsgBufferInfoP=nullptr;
}


Processor::~Processor(){
    cancelAndDelete(selfMsgGenMsg);
    cancelAndDelete(selfMsgBufferInfoP);
}

void Processor::initialize()
{
    // Initialize variables
    numSent = 0;
    numReceived = 0;
    //��ʼ���к��еĲ���
    WATCH(numSent);
    WATCH(numReceived);

    hopCountStats.setName("hopCountStats");
    hopCountStats.setRangeAutoUpper(0, 10, 1.5);
    hopCountVector.setName("HopCount");

    selfMsgGenMsg = new cMessage("selfMsgGenMsg");
    scheduleAt(Sim_Start_Time, selfMsgGenMsg);
    selfMsgBufferInfoP = new cMessage("selfMsgBufferInfoP");
    scheduleAt(Buffer_Info_Sim_Start, selfMsgBufferInfoP);

    for(int i=0;i<VC;i++){
        BufferAvailCurrentP[i]=true;
        BufferAvailConnectP[i]=false;
    }


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
        if(msg==selfMsgGenMsg){
            if(getIndex()==0){
                numSent++;
                EV << "Processor: "<<getIndex()<<"("<<ppid2plid(getIndex())<<") is generating the No."<<numSent<<" message\n";
                FatTreeMsg *newmsg = generateMessage();
                EV << newmsg << endl;
                int vc_id=newmsg->getVc_id();//��ȡvcid
                if(BufferAvailConnectP[vc_id]==true){//ת���µ�fatTreeMsg
                    forwardMessage(newmsg);
                    if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
                        EV<<"Processor Generating and Forwarding New FatTreeMsg >> PROCESSOR: "<<getIndex()<<"("<<ppid2plid(getIndex())<<"), VCID: "
                                <<vc_id<<"\n";
                    }
                    //bubble("Starting new one!");
                }else{//buffer������dropping msg
                    if (Verbose >= VERBOSE_DEBUG_MESSAGES) {
                        EV<<"Buffer is Full, Dropping FatTreeMsg >> PROCESSOR: "<<getIndex()<<"("<<ppid2plid(getIndex())<<"), VCID: "
                                <<vc_id<<"\n";
                    }
                    delete newmsg;

                }


                //simtime_t delay = par("delayTime");
                simtime_t sendNext = par("sendNext");
                //scheduleAt(simTime()+0.1, msg);
                scheduleAt(simTime()+sendNext,selfMsgGenMsg);

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
        }

    }
}

FatTreeMsg *Processor::generateMessage()
{


    // Produce source and destination addresse
    int current_ppid=getIndex();
    int n = getVectorSize();//processor������
    //EV<<n<<"\n";
    int dst_ppid = intuniform(0, n-2); //��������ģ��
    //EV<<dst_ppid<<"\n";
    if (dst_ppid >= getIndex())
        dst_ppid++;//��֤��ȡ��current_ppid

    int vc_id = intuniform(0,VC-1); //�������vcͨ��

    int current_plid=ppid2plid(current_ppid);
    int dst_plid=ppid2plid(dst_ppid);

    char msgname[100];//��ʼ����Ŀռ�̫С�������ݱ��ı�!!!!!!!
    sprintf(msgname, "From processor node %d(%d) to node %d(%d)", current_ppid,current_plid,dst_ppid,dst_plid);

    // Create message object and set source and destination field.
    FatTreeMsg *msg = new FatTreeMsg(msgname);
    msg->setSrc_ppid(current_ppid);//���÷�����processor���
    msg->setDst_ppid(dst_ppid);//���ý��յ�processor���
    msg->setFrom_router_port(getNextRouterPortP());//�����յ���msg��Router�˿�
    msg->setVc_id(vc_id);//����VC ID
    //EV<<current_ppid<<" "<<dst_ppid<<"\n";
    //EV<<msg->getSrc_ppid()<<" "<<msg->getDst_ppid()<<"\n";
    return msg;
}

//processorת����·���㷨,processorֻ��һ��prot,ֱ��ת����ȥ����
void Processor::forwardMessage(FatTreeMsg *msg)
{

    msg->setFrom_router_port(getNextRouterPortP());// �����յ�����Ϣ·�����Ķ˿ں�
    send(msg,"port$o");
    EV << "Forwarding message { " << msg << " } from processor to router.\n";

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
void Processor::finish()
{
    // This function is called by OMNeT++ at the end of the simulation.
    EV << "Sent:     " << numSent << endl;
    EV << "Received: " << numReceived << endl;
    EV << "Hop count, min:    " << hopCountStats.getMin() << endl;
    EV << "Hop count, max:    " << hopCountStats.getMax() << endl;
    EV << "Hop count, mean:   " << hopCountStats.getMean() << endl;
    EV << "Hop count, stddev: " << hopCountStats.getStddev() << endl;

    recordScalar("#sent", numSent);
    recordScalar("#received", numReceived);

    hopCountStats.recordAs("hop count");
}




