/*
 * fat_tree.h
 *
 *  Created on: 2016年8月14日
 *      Author: Vincent
 */

#ifndef FAT_TREE_H_
#define FAT_TREE_H_

//晶体管工艺参数
#define LVT 1
#define NVT 2
#define HVT 3

//#define TR 0.2//toggle rate
#define VDD 1.0

#define PARM(x) PARM_ ## x
#define PARM_TECH_POINT 45
#define PARM_TRANSISTOR_TYPE NVT

//网络拓扑参数
#define PortNum 4
#define LevelNum 3
#define ProcessorNum 16
#define SwitchNum 20
#define LinkNum 48 //双向链路乘以2
#define SwTop 4
#define SwLower 16
#define SwLowEach 8
#define VC 5 //virtual channel
#define BufferDepth 4 //virtual channel buffer depth, 大于等于2
#define FlitWidth 32
#define FixedFlitLength 4 //一个packet中最大flit的长度,也是processor中buffer的长度,poisson分布为该定长


//时钟相关参数
#define FREQ 1e9  //用于功耗仿真，单位hz
#define CLK_CYCLE 1/FREQ //时钟周期
#define Sim_Start_Time 1 //1s 开始仿真
#define TimeScale 0.1 //不改，用于泊松分布和自相似分布，如lambda=10，表示1s内10个flit，得到的时间间隔除以TimeScale再round取整


//Space Distribution
#define UNIFORM //空间均匀分布

//Time Distribution
//#define SELF_SIMILARITY //自相似分布
#define POISSON_DIST //采用泊松分布

//自相似和泊松分布的取值范围小于10，单位时间为1s，但TimeScale会进行换算，最后乘以时钟周期
//自相似和泊松分布的参数

//自相似分布Pareto参数
#define ALPHA_ON 4
#define ALPHA_OFF 2

//Poisson分布参数
#define LAMBDA 7 //泊松分布中用于产生时间间隔的指数分布的lambda，表示单位时间内(1s)到达的帧数，其倒数为时间间隔的平均值


//调试信息
#define Verbose 1
#define VERBOSE_DEBUG_MESSAGES 1
#define VERBOSE_DETAIL_DEBUG_MESSAGES 2

#endif /* FAT_TREE_H_ */

