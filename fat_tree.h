/*
 * fat_tree.h
 *
 *  Created on: 2016年8月14日
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
#define BufferDepth 10//virtual channel buffer depth
#define CLK_CYCLE 0.2 //时钟周期
#define Buffer_Info_Update_Interval 0.2 //和时钟周期保持一致，但是两者之间必须要有时间上的间隔
#define Sim_Start_Time 1 //1s 开始仿真
#define Buffer_Info_Sim_Start 0.85//比Sim_Start_Time提前0.05，同时考虑channel传输延迟0.1s，这个是Buffer更新信号的开始仿真时间

#define Verbose 2
#define VERBOSE_DEBUG_MESSAGES 1
#define VERBOSE_BUFFER_INFO_MESSAGES 3

#endif /* FAT_TREE_H_ */
