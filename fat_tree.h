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
#define FlitWidth 32
#define FixedFlitLength 4 //һ��packet�����flit�ĳ���,Ҳ��processor��buffer�ĳ���,poisson�ֲ�Ϊ�ö���


//ʱ����ز���
#define FREQ 1e9  //���ڹ��ķ��棬��λhz
#define CLK_CYCLE 1/FREQ //ʱ������
#define Sim_Start_Time 1 //1s ��ʼ����
#define TimeScale 0.1 //���ģ����ڲ��ɷֲ��������Ʒֲ�����lambda=10����ʾ1s��10��flit���õ���ʱ��������TimeScale��roundȡ��


//Space Distribution
#define UNIFORM //�ռ���ȷֲ�

//Time Distribution
//#define SELF_SIMILARITY //�����Ʒֲ�
#define POISSON_DIST //���ò��ɷֲ�

//�����ƺͲ��ɷֲ���ȡֵ��ΧС��10����λʱ��Ϊ1s����TimeScale����л��㣬������ʱ������
//�����ƺͲ��ɷֲ��Ĳ���

//�����Ʒֲ�Pareto����
#define ALPHA_ON 4
#define ALPHA_OFF 2

//Poisson�ֲ�����
#define LAMBDA 7 //���ɷֲ������ڲ���ʱ������ָ���ֲ���lambda����ʾ��λʱ����(1s)�����֡�����䵹��Ϊʱ������ƽ��ֵ


//������Ϣ
#define Verbose 1
#define VERBOSE_DEBUG_MESSAGES 1
#define VERBOSE_DETAIL_DEBUG_MESSAGES 2

#endif /* FAT_TREE_H_ */

