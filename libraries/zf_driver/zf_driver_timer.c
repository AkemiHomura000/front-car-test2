/*********************************************************************************************************************
* TC377 Opensourec Library 即（TC377 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC377 开源库的一部分
*
* TC377 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          zf_driver_timer
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.10.2
* 适用平台          TC377TP
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-11-03       pudding            first version
* 2023-07-06       pudding            修复CPU2无法获取定时时间的bug
********************************************************************************************************************/

#include "IfxStm.h"
#include "IFXSTM_CFG.h"
#include "zf_driver_timer.h"

static uint32 systick_count[3];

//-------------------------------------------------------------------------------------------------------------------
//  函数简介      systick定时器启动
//  返回参数      void
//  使用示例      system_start(); // 启动定时器，记录下当前的时间
//-------------------------------------------------------------------------------------------------------------------
void system_start (void)
{
    systick_count[(IfxCpu_getCoreId())] = IfxStm_getLower(IfxStm_getAddress((IfxStm_Index)(IfxCpu_getCoreId())));
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介     获得当前System tick timer的值
//  返回参数     uint32           返回从开始到现在的时间(单位10ns)
//  使用示例     uint32 tim = system_getval();
//  备注信息     在核心0调用此函数则使用STM0模块  核心1则使用STM1模块
//-------------------------------------------------------------------------------------------------------------------
uint32 system_getval (void)
{
    uint32 time;
    uint32 stm_clk;

    stm_clk = IfxStm_getFrequency(IfxStm_getAddress((IfxStm_Index)(IfxCpu_getCoreId())));

    time = IfxStm_getLower(IfxStm_getAddress((IfxStm_Index)(IfxCpu_getCoreId())));
    time = time - systick_count[(IfxCpu_getCoreId())];
    time = (uint32)((uint64)time * 100000000 / stm_clk);

    return time;
}




