/**
 ****************************************************************************************************
 * @file        demo.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2022-06-21
 * @brief       ATK-MS53L0M模块测试实验
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 探索者 F407开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
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
 * @brief       显示设备地址
 * @param       无
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
// * @brief       按键0功能，获取ATK-MS53L0M测量值
// * @param       is_normal: 0，Modbus模式
// *                         1，Normal模式
// *              device_id: ATK-MS53L0M设备地址
// * @retval      无
// */
//static void demo_key0_fun(uint8_t is_normal, uint16_t device_id)
//{
//    uint8_t ret;
//    uint16_t dat;
//    
//    if (is_normal == 0)
//    {
//        /* ATK-MS53L0M Modbus工作模式获取测量值 */
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
//        /* ATK-MS53L0M Normal工作模式获取测量值 */
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
// * @brief       按键1功能，切换ATK-MS53L0M工作模式
// * @param       is_normal: 0，Modbus模式
// *                         1，Normal模式
// *              device_id: ATK-MS53L0M设备地址
// * @retval      无
// */
//static void demo_key1_fun(uint8_t *is_normal, uint16_t device_id)
//{
//    uint8_t ret;
//    
//    if (*is_normal == 0)
//    {
//        /* 设置ATK-MS53L0M的工作模式为Normal模式 */
//        ret = atk_ms53l0m_write_data(device_id, ATK_MS53L0M_FUNCODE_WORKMODE, ATK_MS53L0M_WORKMODE_NORMAL);
//        if (ret == 0)
//        {
//            /* 设置ATK-MS53L0M的回传速率为5Hz */
//            atk_ms53l0m_write_data(device_id, ATK_MS53L0M_FUNCODE_BACKRATE, ATK_MS53L0M_BACKRATE_5HZ);
//            /* 设置ATK-MS53L0M的测量模式为长距离模式 */
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
//        /* 设置ATK-MS53L0M的工作模式为Modbus模式 */
//        ret = atk_ms53l0m_write_data(device_id, ATK_MS53L0M_FUNCODE_WORKMODE, ATK_MS53L0M_WORKMODE_MODBUS);
//        if (ret == 0)
//        {
//            /* 设置ATK-MS53L0M的测量模式为高速测量模式 */
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
// * @brief       例程演示入口函数
// * @param       无
// * @retval      无
// */
//void demo_run(void)
//{
//    uint8_t ret;
//    uint8_t key;
//    uint16_t id;
//    uint8_t is_normal = 0;
//    
//    /* 初始化ATK-MS53L0M */
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
//    /* ATK-MS53L0M初始化成功，显示设备地址 */
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
//                /* 获取ATK-MS53L0M测量值 */
//                demo_key0_fun(is_normal, id);
//                break;
//            }
//            case KEY1_PRES:
//            {
//                /* 切换ATK-MS53L0M工作模式 */
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
