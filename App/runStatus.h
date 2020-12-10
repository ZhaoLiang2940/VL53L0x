/*******************************************************************************
 * runStatus.h
 *
 * Created on: 2020��10��26��
 *
 * Author: ZhaoSir
 *******************************************************************************/
 #ifndef __RUNSTATUS_H__
 #define __RUNSTATUS_H__
 
 typedef enum _systemStatus
 {
    SleepStatus,                                                                // ����״̬
    SampleStatus,                                                               // ����״̬
    Send2BleStatus,                                                             // ��BLE��������״̬
    HandleUsartStatus                                                           // ����������״̬
 }systemStatus;

extern          void                initSystemStatus(void);
extern          void                setSystemCurrentStatus(systemStatus status);
extern          systemStatus        getSystemCurrentStatus(void);
 
 
 #endif


