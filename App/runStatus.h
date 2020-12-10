/*******************************************************************************
 * runStatus.h
 *
 * Created on: 2020年10月26日
 *
 * Author: ZhaoSir
 *******************************************************************************/
 #ifndef __RUNSTATUS_H__
 #define __RUNSTATUS_H__
 
 typedef enum _systemStatus
 {
    SleepStatus,                                                                // 休眠状态
    SampleStatus,                                                               // 休眠状态
    Send2BleStatus,                                                             // 给BLE发送数据状态
    HandleUsartStatus                                                           // 处理串口数据状态
 }systemStatus;

extern          void                initSystemStatus(void);
extern          void                setSystemCurrentStatus(systemStatus status);
extern          systemStatus        getSystemCurrentStatus(void);
 
 
 #endif


