/*!
    \file    gd32f4xx_it.c
    \brief   interrupt service routines

    \version 2023-06-25, V3.1.0, firmware for GD32F4xx
*/

/*
    Copyright (c) 2023, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#include "gd32f4xx_it.h"
#include "systick.h"
#include "hsp_gpio.h"

extern uint32_t sys_tick_counter;
extern uint16_t RES_value;
extern int16_t encoder_speed;
extern uint8_t rx_buffer[] ;
extern uint16_t rx_idx, tx_idx;
uint32_t encoder_pulse;
uint32_t encoder_pulse_t;
extern uint8_t buzz_flag;
extern uint16_t buzz_count;


/*!
    \brief      this function handles NMI exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void NMI_Handler(void)
{
}

/*!
    \brief      this function handles HardFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void HardFault_Handler(void)
{
    /* if Hard Fault exception occurs, go to infinite loop */
    while(1) {
    }
}

/*!
    \brief      this function handles MemManage exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void MemManage_Handler(void)
{
    /* if Memory Manage exception occurs, go to infinite loop */
    while(1) {
    }
}

/*!
    \brief      this function handles BusFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void BusFault_Handler(void)
{
    /* if Bus Fault exception occurs, go to infinite loop */
    while(1) {
    }
}

/*!
    \brief      this function handles UsageFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void UsageFault_Handler(void)
{
    /* if Usage Fault exception occurs, go to infinite loop */
    while(1) {
    }
}

/*!
    \brief      this function handles SVC exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SVC_Handler(void)
{
}

/*!
    \brief      this function handles DebugMon exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void DebugMon_Handler(void)
{
}

/*!
    \brief      this function handles PendSV exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void PendSV_Handler(void)
{
}

/*!
    \brief      this function handles SysTick exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SysTick_Handler(void)
{
	delay_decrement();
	sys_tick_counter++;

	if (!(sys_tick_counter % 5))
	{
		// LED_R_TOGGLE();
		encoder_pulse = timer_counter_read(TIMER3);
		encoder_speed =  30000 - encoder_pulse ;
		timer_counter_value_config(TIMER3, 30000);
	}
        
        if(buzz_count > 0)
        {
          buzz_count --;
          BUZZ_TOGGLE();
        }
        else
        {
          BUZZ_OFF();
        }
        
        if(buzz_flag)
        {
          buzz_count = 1000;
                      
        }

}

/*!
    \brief      this function handles USART2 RBNE interrupt request
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USART2_IRQHandler(void)
{
    uint8_t data;

    if((RESET != usart_interrupt_flag_get(USART2, USART_INT_FLAG_RBNE)) && 
       (RESET != usart_flag_get(USART2, USART_FLAG_RBNE)))
    {
        /* read one byte from the receive data register */
        data = (uint8_t)usart_data_receive(USART2);

        if(((rx_idx+1)%100) != tx_idx)
        {
            rx_buffer[rx_idx++] = data;
			rx_idx %= 100;      // winding back to the start of the ring buffer
        }
    }
}

/*!
    \brief      this function handles external line 14 interrupt request
    \param[in]  none
    \param[out] none
    \retval     none
*/
void EXTI10_15_IRQHandler(void)
{
    if(RESET != exti_interrupt_flag_get(EXTI_14))
    {
        if(RESET == PHB2())
        {
            if (RES_value < 65535)
                RES_value++;
        }
        else
        {
            if (RES_value > 0)
                RES_value--;
        }
    }
    exti_interrupt_flag_clear(EXTI_14);
}