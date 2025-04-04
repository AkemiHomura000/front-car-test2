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
* �ļ�����          cpu1_main
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
#include "all_init.h"
#include "motor.h"
#include "image.h"
#pragma section all "cpu1_dsram"
// ���������#pragma section all restore���֮���ȫ�ֱ���������CPU1��RAM��

// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������

// **************************** �������� ****************************
void core1_main(void)//���������ƺ���λ������
{
    disable_Watchdog();                     // �رտ��Ź�
    interrupt_global_enable(0);             // ��ȫ���ж�
    // �˴���д�û����� ���������ʼ�������
    all_init();
    gpio_init(P21_5, GPO, GPIO_HIGH, GPO_PUSH_PULL);         // ��ʼ�� Ĭ�ϸߵ�ƽ �������ģʽ 5��Ӧ���ַ���
    gpio_init(P21_3, GPO, GPIO_LOW, GPO_PUSH_PULL);         // ��ʼ�� Ĭ�ϸߵ�ƽ �������ģʽ 3��Ӧ���ַ���
    pit_ms_init(CCU60_CH0, 10);
    wireless_uart_init();
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIRELESS_UART);
    // �˴���д�û����� ���������ʼ�������
    cpu_wait_event_ready();                 // �ȴ����к��ĳ�ʼ�����
    printf("init_uart\r\n");
    seekfree_assistant_oscilloscope_struct oscilloscope_data;
    oscilloscope_data.data[0] = speed_l;
    oscilloscope_data.data[1] = speed_r;
    oscilloscope_data.data[2] = target_speed - d_speed;
    oscilloscope_data.data[3] = target_speed + d_speed;
    oscilloscope_data.data[4] = error;
    oscilloscope_data.data[5] = d_speed;
    oscilloscope_data.channel_num = 6;
    while (TRUE)
    {
        // �δ�ͽ������յ�������
        seekfree_assistant_data_analysis();
        // ����
        for(uint8_t i = 0; i < SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT; i++)
        {
            // ���±�־λ
            if(seekfree_assistant_parameter_update_flag[i])
            {
                seekfree_assistant_parameter_update_flag[i] = 0;
                //--------------�����ڲ�����
                target_speed = seekfree_assistant_parameter[0];
                err_kp       = seekfree_assistant_parameter[1];
                err_kd       = seekfree_assistant_parameter[2];
                sptr_r.D     = seekfree_assistant_parameter[3];
                stop         = seekfree_assistant_parameter[4];
                //--------------ͨ��DEBBUG���ڷ�����Ϣ
//                printf("receive data channel : %d ", i);
//                printf("data : %f ", seekfree_assistant_parameter[i]);
//                printf("\r\n");
            }
        }
        oscilloscope_data.data[0] = speed_l;
        oscilloscope_data.data[1] = speed_r;
        oscilloscope_data.data[2] = target_speed - d_speed;
        oscilloscope_data.data[3] = target_speed + d_speed;
        oscilloscope_data.data[4] = error;
        oscilloscope_data.data[5] = d_speed;
        seekfree_assistant_oscilloscope_send(&oscilloscope_data);
        system_delay_ms(5);
        // �˴���д��Ҫѭ��ִ�еĴ���
    }
}
#pragma section all restore
