/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "board.h"
#include <machine/_stdint.h>
#include <stdio.h>
#include "bsp_mpu6050.h"
#include "inv_mpu.h"
#include "oled.h"
uint8_t status;
uint16_t time;
uint32_t count;
int main(void)
{
    uint8_t str[64];            //定义一个数组，OLED显示时，把东西全放进数组中，然后打印
	board_init();               //初始化了串口0，MPU6050中断，里面还有延迟函数，串口重定向，串口0的中断
	
	MPU6050_Init();             //初始化MPU6050
	
    uint8_t ret = 1;
          
    float pitch=0,roll=0,yaw=0;                     //定义三个角度

    printf("start\r\n");                            //串口打印start

    while( mpu_dmp_init() )
    {
        printf("dmp error\r\n");                    //如果MPU6050没初始化成功就串口一直打印dmp error
        delay_ms(200);
    }
    time = 15;
    count = 0;




    printf("Initialization Data Succeed \r\n");         //打印一次初始化成功
    OLED_Init();		//初始化OLED
    OLED_Clear();
    while (time)
    {
        delay_ms(20);
        time --;

    
    }

    while(1) 
    {

        status = mpu_dmp_get_data(&pitch,&roll,&yaw);           //标志位，如果为0就有数据
        if( status == 0 )                                       //如果为0就串口打印数据
        { 
            count ++;
            printf("pitch =%d\r\n", (int)pitch);
            printf("roll =%d\r\n", (int)roll);
            printf("yaw =%d\r\n\r\n", (int)yaw);
            printf("count =%d\r\n\r\n", (int)count);
            
        }
        delay_ms(20);



        sprintf((char *)str, "pitch:%.2f", pitch);                //把PITCH角放入数组中
        OLED_ShowString(0,0,(uint8_t *)str,16);         //OLED显示数组中的内容                          该函数格式：X，Y，字符串，大小

        sprintf((char *)str, "yaw:%.2f", yaw);                //把yaw角放入数组中
        OLED_ShowString(0,2,(uint8_t *)str,16);         //OLED显示数组中的内容                          该函数格式：X，Y，字符串，大小

        sprintf((char *)str, "roll:%.2f", roll);                //把ROLL角放入数组中
        OLED_ShowString(0,4,(uint8_t *)str,16);         //OLED显示数组中的内容                          该函数格式：X，Y，字符串，大小



    }

}

//OLED的显存
//存放格式如下.
//[0]0 1 2 3 ... 127	
//[1]0 1 2 3 ... 127	
//[2]0 1 2 3 ... 127	
//[3]0 1 2 3 ... 127	
//[4]0 1 2 3 ... 127	
//[5]0 1 2 3 ... 127	
//[6]0 1 2 3 ... 127	
//[7]0 1 2 3 ... 127 			   