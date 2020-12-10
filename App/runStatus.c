/*******************************************************************************
 * runStatus.c
 *
 * Created on: 2020年10月26日
 *
 * Author: ZhaoSir
 *******************************************************************************/
#include "runStatus.h"

systemStatus        SystemRunSatus[2];

/********************************************************************************
*               runStatus.c
*函数名称：	initSystemStatus()
*
*函数作用：	初始化系统运行状态
*
*参数说明：	无
*
*函数返回：	无
*
*函数作者：	ZhaoSir
********************************************************************************/
void initSystemStatus(void)
{
    SystemRunSatus[0] = SleepStatus;
    SystemRunSatus[1] = SystemRunSatus[0];
}

/********************************************************************************
*               runStatus.c
*函数名称：	setSystemStatus()
*
*函数作用：	设置系统运行状态
*
*参数说明：	status：需要为系统设定的接下来的状态
*
*函数返回：	无
*
*函数作者：	ZhaoSir
********************************************************************************/
void setSystemCurrentStatus(systemStatus status)
{
    SystemRunSatus[0] = SystemRunSatus[1];                                      // 记录上一次系统处于的状态
    SystemRunSatus[1] = status;                                                 // 记录接下来系统应该出的状态
}


/********************************************************************************
*               runStatus.c
*函数名称：	getSystemCurrentStatus()
*
*函数作用：	读取系统当前处于的状态
*
*参数说明：	无
*
*函数返回：	当前系统处于的状态
*
*函数作者：	ZhaoSir
********************************************************************************/
systemStatus getSystemCurrentStatus(void)
{
    return SystemRunSatus[1];
}



