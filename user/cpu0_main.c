/*********************************************************************************************************************
 * TC377 Opensourec Library ����TC377 ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
 * Copyright (c) 2022 SEEKFREE ��ɿƼ�
 *
 * ���ļ��� TC377 ��Դ���һ����
 *
 * TC377 ��Դ�� ��������
 * �����Ը��������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù������֤��������
 * �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
 *
 * ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
 * ����û�������������Ի��ʺ��ض���;�ı�֤
 * ����ϸ����μ� GPL
 *
 * ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
 * ���û�У������<https://www.gnu.org/licenses/>
 *
 * ����ע����
 * ����Դ��ʹ�� GPL3.0 ��Դ���֤Э�� �����������Ϊ���İ汾
 * �������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
 * ���֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
 * ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
 *
 * �ļ�����          cpu0_main
 * ��˾����          �ɶ���ɿƼ����޹�˾
 * �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
 * ��������          ADS v1.10.2
 * ����ƽ̨          TC377TP
 * ��������          https://seekfree.taobao.com/
 *
 * �޸ļ�¼
 * ����              ����                ��ע
 * 2022-11-03       pudding            first version
 ********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "image.h"
#include "motor.h"
#pragma section all "cpu0_dsram"
// ���������#pragma section all restore���֮���ȫ�ֱ���������CPU0��RAM��

// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������

// **************************** �������� ****************************
void circle_run(void);
int core0_main(void) // �����Ӿ�����
{
    clock_init(); // ��ȡʱ��Ƶ��<��ر���>
    debug_init(); // ��ʼ��Ĭ�ϵ��Դ���
    // �˴���д�����йس�ʼ�������
    ips200_init(IPS200_TYPE_SPI);
    while (1)
    {
        if (mt9v03x_init())
            ips200_show_string(0, 80, "mt9v03x reinit.");
        else
            break;
        system_delay_ms(500); // ����ʱ�������Ʊ�ʾ�쳣
    }
    // pit_ms_init(CCU60_CH1, 100);
    // �˴���д�û����� ���������ʼ�������

    cpu_wait_event_ready(); // �ȴ����к��ĳ�ʼ�����
    while (TRUE)
    {
        // print_angle();
        image_process();
//        circle_run();
//        update_status();
        // �˴���д��Ҫѭ��ִ�еĴ���
    }
}
#pragma section all restore
// **************************** �������� ****************************
void circle_run(void)//�ߵ����Ӿ�
{
    //ͣ�͹ر��Ӿ�����

    //��ǰֱ��
//    while(!IfxCpu_acquireMutex(&dspeed_mutex));
    if (IfxCpu_acquireMutex(&dspeed_mutex))
    {
        d_speed = 0;
        IfxCpu_releaseMutex(&dspeed_mutex);
    }
    system_delay_ms(100);
    //��������
//    while(!IfxCpu_acquireMutex(&dspeed_mutex));
    if (IfxCpu_acquireMutex(&dspeed_mutex))
    {
        d_speed = -180;
        IfxCpu_releaseMutex(&dspeed_mutex);
    }
    float yaw_in = get_angle();
    system_delay_ms(1000);
    while((angle_yaw > (yaw_in+1)) || (angle_yaw < (yaw_in-1)));
    //����ֱ��
//    while(!IfxCpu_acquireMutex(&dspeed_mutex));
    if (IfxCpu_acquireMutex(&dspeed_mutex))
    {
        d_speed = 0;
        IfxCpu_releaseMutex(&dspeed_mutex);
    }
    system_delay_ms(1000);
    //�����Ӿ������ж�
}
