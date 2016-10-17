/*
 * fat_tree.h
 *
 *  Created on: 2016��8��14��
 *      Author: Vincent
 */

#ifndef FAT_TREE_H_
#define FAT_TREE_H_

#define PortNum 4
#define LevelNum 3
#define ProcessorNum 16
#define SwitchNum 20
#define SwTop 4
#define SwLower 16
#define SwLowEach 8
#define VC 4 //virtual channel
#define BufferDepth 10 //virtual channel buffer depth

#define CLK_CYCLE 0.1 //ʱ������
#define ROUND 10 //Ϊʱ�����ڵĵ���
#define Buffer_Info_Update_Interval 0.1 //��ʱ�����ڱ���һ�£���������֮�����Ҫ��ʱ���ϵļ��
#define Sim_Start_Time 1 //1s ��ʼ����
#define Buffer_Info_Sim_Start 0.98//��Sim_Start_Time��ǰ0.05��ͬʱ����channel�����ӳ�0.1s�������Buffer�����źŵĿ�ʼ����ʱ��

#define FixedFlitLength 4 //һ��package�����flit�ĳ���,Ҳ��processor��buffer�ĳ���

//Space Distribution
#define UNIFORM //�ռ���ȷֲ�

//Time Distribution
#define SELF_SIMILARITY //�����Ʒֲ�
#define POISSON_DIST //���ò��ɷֲ�

//�����Ʒֲ�Pareto����
#define ALPHA_ON 3
#define ALPHA_OFF 2

//Poisson�ֲ�����
#define LAMBDA 5 //���ɷֲ������ڲ���ʱ������ָ���ֲ���lambda����ʾ��λʱ����(1s)�����֡�����䵹��Ϊʱ������ƽ��ֵ

//������Ϣ
#define Verbose 2
#define VERBOSE_DEBUG_MESSAGES 1
#define VERBOSE_BUFFER_INFO_MESSAGES 3
#define VERBOSE_DETAIL_DEBUG_MESSAGES 4

#endif /* FAT_TREE_H_ */

