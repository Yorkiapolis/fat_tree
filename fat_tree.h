/*
 * fat_tree.h
 *
 *  Created on: 2016��8��14��
 *      Author: Vincent
 */

#ifndef FAT_TREE_H_
#define FAT_TREE_H_

//����ܹ��ղ���
#define LVT 1
#define NVT 2
#define HVT 3

//#define TR 0.2//toggle rate
#define VDD 1.0

#define PARM(x) PARM_ ## x
#define PARM_TECH_POINT 45
#define PARM_TRANSISTOR_TYPE NVT

//�������˲���
#define PortNum 4
#define LevelNum 3
#define ProcessorNum 16
#define SwitchNum 20
#define LinkNum 48 //˫����·����2
#define SwTop 4
#define SwLower 16
#define SwLowEach 8
#define VC 5 //virtual channel
#define BufferDepth 4 //virtual channel buffer depth, ���ڵ���2
#define BufferOut 0 //Router output buffer
#define FlitWidth 32
#define FixedFlitLength 4 //һ��package�����flit�ĳ���,Ҳ��processor��buffer�ĳ���,poisson�ֲ�Ϊ����


//ʱ����ز���
#define FREQ_Hz 1e9  //���ڹ��ķ���
#define CLK_CYCLE 0.1 //ʱ������
#define ROUND 10 //Ϊʱ�����ڵĵ���
//#define Buffer_Info_Update_Interval 0.1 //��ʱ�����ڱ���һ�£���������֮�����Ҫ��ʱ���ϵļ��
#define Sim_Start_Time 1 //1s ��ʼ����
//#define Buffer_Info_Sim_Start 0.98//��Sim_Start_Time��ǰ0.05��ͬʱ����channel�����ӳ�0.1s�������Buffer�����źŵĿ�ʼ����ʱ��


//Space Distribution
#define UNIFORM //�ռ���ȷֲ�

//Time Distribution
//#define SELF_SIMILARITY //�����Ʒֲ�
#define POISSON_DIST //���ò��ɷֲ�

//�����ƺͲ��ɷֲ��Ĳ���
//�����Ʒֲ�Pareto����
#define ALPHA_ON 4
#define ALPHA_OFF 2

//Poisson�ֲ�����
#define LAMBDA 7 //���ɷֲ������ڲ���ʱ������ָ���ֲ���lambda����ʾ��λʱ����(1s)�����֡�����䵹��Ϊʱ������ƽ��ֵ


//������Ϣ
#define Verbose 2
#define VERBOSE_DEBUG_MESSAGES 1
#define VERBOSE_BUFFER_INFO_MESSAGES 3
#define VERBOSE_DETAIL_DEBUG_MESSAGES 4

#endif /* FAT_TREE_H_ */

