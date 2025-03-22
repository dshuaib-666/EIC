/**
 ****************************************************************************************************
 * @file        demo.c
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2022-06-21
 * @brief       ATK-MS53L0Mģ�����ʵ��
 * @license     Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:����ԭ�� ̽���� F407������
 * ������Ƶ:www.yuanzige.com
 * ������̳:www.openedv.com
 * ��˾��ַ:www.alientek.com
 * �����ַ:openedv.taobao.com
 *
 ****************************************************************************************************
 */
#include "ATKTDF.h"
#include "demo.h"
#include "ATKTDF_USA.h"
#include "usart.h"
 #include "freertos.h"
#include "Task.h"
#include "main.h"
#include <stdio.h>

/**
 * @brief       ��ʾ�豸��ַ
 * @param       ��
 * @retval      �
 */
//static void demo_show_id(uint16_t id)
//{
//    char buf[23];
//    
//    sprintf(buf, "ATK-MS53L0M ID: 0x%04x", id);
//    


//}

///**
// * @brief       ����0���ܣ���ȡATK-MS53L0M����ֵ
// * @param       is_normal: 0��Modbusģʽ
// *                         1��Normalģʽ
// *              device_id: ATK-MS53L0M�豸��ַ
// * @retval      ��
// */
//static void demo_key0_fun(uint8_t is_normal, uint16_t device_id)
//{
//    uint8_t ret;
//    uint16_t dat;
//    
//    if (is_normal == 0)
//    {
//        /* ATK-MS53L0M Modbus����ģʽ��ȡ����ֵ */
//        ret = atk_ms53l0m_modbus_get_data(device_id, &dat);
//        if (ret == 0)
//        {
// 
//        }
//        else
//        {
//           
//        }
//    }
//    else
//    {
//        /* ATK-MS53L0M Normal����ģʽ��ȡ����ֵ */
//        ret = atk_ms53l0m_normal_get_data(&dat);
//        if (ret == 0)
//        {
//          
//        }
//        else
//        {
//       
//        }
//    }
//}

///**
// * @brief       ����1���ܣ��л�ATK-MS53L0M����ģʽ
// * @param       is_normal: 0��Modbusģʽ
// *                         1��Normalģʽ
// *              device_id: ATK-MS53L0M�豸��ַ
// * @retval      ��
// */
//static void demo_key1_fun(uint8_t *is_normal, uint16_t device_id)
//{
//    uint8_t ret;
//    
//    if (*is_normal == 0)
//    {
//        /* ����ATK-MS53L0M�Ĺ���ģʽΪNormalģʽ */
//        ret = atk_ms53l0m_write_data(device_id, ATK_MS53L0M_FUNCODE_WORKMODE, ATK_MS53L0M_WORKMODE_NORMAL);
//        if (ret == 0)
//        {
//            /* ����ATK-MS53L0M�Ļش�����Ϊ5Hz */
//            atk_ms53l0m_write_data(device_id, ATK_MS53L0M_FUNCODE_BACKRATE, ATK_MS53L0M_BACKRATE_5HZ);
//            /* ����ATK-MS53L0M�Ĳ���ģʽΪ������ģʽ */
//            atk_ms53l0m_write_data(device_id, ATK_MS53L0M_FUNCODE_MEAUMODE, ATK_MS53L0M_MEAUMODE_LONG);
//            
//            *is_normal = 1;
//          
//        }
//        else
//        {
//            
//        }
//    }
//    else
//    {
//        /* ����ATK-MS53L0M�Ĺ���ģʽΪModbusģʽ */
//        ret = atk_ms53l0m_write_data(device_id, ATK_MS53L0M_FUNCODE_WORKMODE, ATK_MS53L0M_WORKMODE_MODBUS);
//        if (ret == 0)
//        {
//            /* ����ATK-MS53L0M�Ĳ���ģʽΪ���ٲ���ģʽ */
//            atk_ms53l0m_write_data(device_id, ATK_MS53L0M_FUNCODE_MEAUMODE, ATK_MS53L0M_MEAUMODE_HISPEED);
//            
//            *is_normal = 0;
//          
//        }
//        else
//        {
//         
//        }
//    }
//}

///**
// * @brief       ������ʾ��ں���
// * @param       ��
// * @retval      ��
// */
//void demo_run(void)
//{
//    uint8_t ret;
//    uint8_t key;
//    uint16_t id;
//    uint8_t is_normal = 0;
//    
//    /* ��ʼ��ATK-MS53L0M */
//    ret = atk_ms53l0m_init(115200, &id);
//    if (ret != 0)
//    {
//       
//        while (1)
//        {
//           
//        }
//    }
//    
//    /* ATK-MS53L0M��ʼ���ɹ�����ʾ�豸��ַ */
//    demo_show_id(id);
//    
//    while (1)
//    {
//        key = key_scan(0);
//        
//        switch (key)
//        {
//            case KEY0_PRES:
//            {
//                /* ��ȡATK-MS53L0M����ֵ */
//                demo_key0_fun(is_normal, id);
//                break;
//            }
//            case KEY1_PRES:
//            {
//                /* �л�ATK-MS53L0M����ģʽ */
//                demo_key1_fun(&is_normal, id);
//                break;
//            }
//            default:
//            {
//                break;
//            }
//        }
//        
//        delay_ms(10);
//    }
//}
