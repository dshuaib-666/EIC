/**
 ****************************************************************************************************
 * @file        demo.c
 * @author      ÕýµãÔ­×ÓÍÅ¶Ó(ALIENTEK)
 * @version     V1.0
 * @date        2022-06-21
 * @brief       ATK-MS53L0MÄ£¿é²âÊÔÊµÑé
 * @license     Copyright (c) 2020-2032, ¹ãÖÝÊÐÐÇÒíµç×Ó¿Æ¼¼ÓÐÏÞ¹«Ë¾
 ****************************************************************************************************
 * @attention
 *
 * ÊµÑéÆ½Ì¨:ÕýµãÔ­×Ó Ì½Ë÷Õß F407¿ª·¢°å
 * ÔÚÏßÊÓÆµ:www.yuanzige.com
 * ¼¼ÊõÂÛÌ³:www.openedv.com
 * ¹«Ë¾ÍøÖ·:www.alientek.com
 * ¹ºÂòµØÖ·:openedv.taobao.com
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
 * @brief       ÏÔÊ¾Éè±¸µØÖ·
 * @param       ÎÞ
 * @retval      Î
 */
//static void demo_show_id(uint16_t id)
//{
//    char buf[23];
//    
//    sprintf(buf, "ATK-MS53L0M ID: 0x%04x", id);
//    


//}

///**
// * @brief       °´¼ü0¹¦ÄÜ£¬»ñÈ¡ATK-MS53L0M²âÁ¿Öµ
// * @param       is_normal: 0£¬ModbusÄ£Ê½
// *                         1£¬NormalÄ£Ê½
// *              device_id: ATK-MS53L0MÉè±¸µØÖ·
// * @retval      ÎÞ
// */
//static void demo_key0_fun(uint8_t is_normal, uint16_t device_id)
//{
//    uint8_t ret;
//    uint16_t dat;
//    
//    if (is_normal == 0)
//    {
//        /* ATK-MS53L0M Modbus¹¤×÷Ä£Ê½»ñÈ¡²âÁ¿Öµ */
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
//        /* ATK-MS53L0M Normal¹¤×÷Ä£Ê½»ñÈ¡²âÁ¿Öµ */
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
// * @brief       °´¼ü1¹¦ÄÜ£¬ÇÐ»»ATK-MS53L0M¹¤×÷Ä£Ê½
// * @param       is_normal: 0£¬ModbusÄ£Ê½
// *                         1£¬NormalÄ£Ê½
// *              device_id: ATK-MS53L0MÉè±¸µØÖ·
// * @retval      ÎÞ
// */
//static void demo_key1_fun(uint8_t *is_normal, uint16_t device_id)
//{
//    uint8_t ret;
//    
//    if (*is_normal == 0)
//    {
//        /* ÉèÖÃATK-MS53L0MµÄ¹¤×÷Ä£Ê½ÎªNormalÄ£Ê½ */
//        ret = atk_ms53l0m_write_data(device_id, ATK_MS53L0M_FUNCODE_WORKMODE, ATK_MS53L0M_WORKMODE_NORMAL);
//        if (ret == 0)
//        {
//            /* ÉèÖÃATK-MS53L0MµÄ»Ø´«ËÙÂÊÎª5Hz */
//            atk_ms53l0m_write_data(device_id, ATK_MS53L0M_FUNCODE_BACKRATE, ATK_MS53L0M_BACKRATE_5HZ);
//            /* ÉèÖÃATK-MS53L0MµÄ²âÁ¿Ä£Ê½Îª³¤¾àÀëÄ£Ê½ */
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
//        /* ÉèÖÃATK-MS53L0MµÄ¹¤×÷Ä£Ê½ÎªModbusÄ£Ê½ */
//        ret = atk_ms53l0m_write_data(device_id, ATK_MS53L0M_FUNCODE_WORKMODE, ATK_MS53L0M_WORKMODE_MODBUS);
//        if (ret == 0)
//        {
//            /* ÉèÖÃATK-MS53L0MµÄ²âÁ¿Ä£Ê½Îª¸ßËÙ²âÁ¿Ä£Ê½ */
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
// * @brief       Àý³ÌÑÝÊ¾Èë¿Úº¯Êý
// * @param       ÎÞ
// * @retval      ÎÞ
// */
//void demo_run(void)
//{
//    uint8_t ret;
//    uint8_t key;
//    uint16_t id;
//    uint8_t is_normal = 0;
//    
//    /* ³õÊ¼»¯ATK-MS53L0M */
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
//    /* ATK-MS53L0M³õÊ¼»¯³É¹¦£¬ÏÔÊ¾Éè±¸µØÖ· */
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
//                /* »ñÈ¡ATK-MS53L0M²âÁ¿Öµ */
//                demo_key0_fun(is_normal, id);
//                break;
//            }
//            case KEY1_PRES:
//            {
//                /* ÇÐ»»ATK-MS53L0M¹¤×÷Ä£Ê½ */
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
