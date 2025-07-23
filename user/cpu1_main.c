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
void core1_main(void) // ���������ƺ���λ������
{
    disable_Watchdog();         // �رտ��Ź�
    interrupt_global_enable(0); // ��ȫ���ж�
    // �˴���д�û����� ���������ʼ�������
    all_init();
    gpio_init(P21_5, GPO, GPIO_HIGH, GPO_PUSH_PULL); // ��ʼ�� Ĭ�ϸߵ�ƽ �������ģʽ 5��Ӧ���ַ���
    gpio_init(P21_3, GPO, GPIO_LOW, GPO_PUSH_PULL);  // ��ʼ�� Ĭ�ϸߵ�ƽ �������ģʽ 3��Ӧ���ַ���

    pit_ms_init(CCU60_CH0, PIT_60_0_PERIOD);
    pit_ms_init(CCU60_CH1, PIT_60_1_PERIOD);
    pit_ms_init(CCU61_CH0, PIT_61_0_PERIOD);

    // �˴���д�û����� ���������ʼ�������
    cpu_wait_event_ready(); // �ȴ����к��ĳ�ʼ�����
    printf("init_uart\r\n");

    while (TRUE)
    {
        system_delay_ms(5);
    }
}
#pragma section all restore
