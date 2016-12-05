/*
 * -------------------------------------------
 *    MSP432 DriverLib - v3_21_00_05 
 * -------------------------------------------
 *
 * --COPYRIGHT--,BSD,BSD
 * Copyright (c) 2016, Texas Instruments Incorporated
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
 * --/COPYRIGHT--*/
/******************************************************************************
 * MSP432 UART - Loopback with 24MHz DCO BRCLK
 *
 * Description: This demo connects TX to RX of the MSP432 UART
 * The example code shows proper initialization of registers
 * and interrupts to receive and transmit data. If data is incorrect P1.0 LED
 * is turned ON.
 *
 *  MCLK = HSMCLK = SMCLK = DCO of 24MHz
 *
 *               MSP432P401
 *             -----------------
 *            |                 |
 *       RST -|     P3.3/UCA0TXD|----|
 *            |                 |    |
 *           -|                 |    |
 *            |     P3.2/UCA0RXD|----|
 *            |                 |
 *            |             P1.0|---> LED
 *            |                 |
 *
 * Author: Timothy Logan
*******************************************************************************/
#include "msp.h"
#include <grlib.h>
#include <HAL_I2C.h>
#include <HAL_OPT3001.h>
#include "Crystalfontz128x128_ST7735.h"
#include "HAL_MSP_EXP432P401R_Crystalfontz128x128_ST7735.h"
#include <stdio.h>
/* DriverLib Includes */
#include "driverlib.h"

/* Standard Includes */
#include <stdint.h>

#include <stdbool.h>

//Data that is going to be sent and received through uart
volatile uint8_t TXData = 1;
volatile uint8_t RXData = 0;

/* Graphic library context */
Graphics_Context g_sContext;

/* Variable for storing lux value returned from OPT3001 */
float lux;

/* Timer_A Up Configuration Parameter */
const Timer_A_UpModeConfig upConfig =
{
        TIMER_A_CLOCKSOURCE_ACLK,               // ACLK Clock SOurce
        TIMER_A_CLOCKSOURCE_DIVIDER_1,          // ACLK/1 = 3MHz
        200,                                    // 200 tick period
        TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE,    // Disable CCR0 interrupt
        TIMER_A_DO_CLEAR                        // Clear value
};

/* Timer_A Compare Configuration Parameter  (PWM) */
Timer_A_CompareModeConfig compareConfig_PWM =
{
        TIMER_A_CAPTURECOMPARE_REGISTER_3,          // Use CCR3
        TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,   // Disable CCR interrupt
        TIMER_A_OUTPUTMODE_TOGGLE_SET,              // Toggle output but
        100                                         // 50% Duty Cycle
};

/* UART Configuration Parameter. These are the configuration parameters to
 * make the eUSCI A UART module to operate with a 115200 baud rate. These
 * values were calculated using the online calculator that TI provides
 * at:
 * http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
 */
const eUSCI_UART_Config uartConfig =
{
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
        13,                                      // BRDIV = 13
        0,                                       // UCxBRF = 0
        37,                                      // UCxBRS = 37
        EUSCI_A_UART_NO_PARITY,                  // No Parity
        EUSCI_A_UART_MSB_FIRST,                  // MSB First
        EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
        EUSCI_A_UART_MODE,                       // UART mode
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION  // Oversampling
};

int main(void)
{
    /* Halting WDT  */
    MAP_WDT_A_holdTimer();

    /* Selecting P3.2 and P3.3 in UART mode and P1.0 as output (LED) */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3,
             GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
    
    /* Set the core voltage level to VCORE1 */
    MAP_PCM_setCoreVoltageLevel(PCM_VCORE1);

    /* Setting DCO to 24MHz (upping Vcore) --> the loopback example */
    /* Set 2 flash wait states for Flash bank 0 and 1 */
    FlashCtl_setWaitState(FLASH_BANK0, 2);
    FlashCtl_setWaitState(FLASH_BANK1, 2);
    
    /* Initialize Clock System */
    MAP_CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_48);//48 was 24 in loopback example
    MAP_CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
    MAP_CS_initClockSignal(CS_HSMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
    MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
    MAP_CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    
    /* Initializes display */
    Crystalfontz128x128_Init();
    
    /* Set default screen orientation */
    Crystalfontz128x128_SetOrientation(0);
    
    /* Initializes graphics context */
    Graphics_initContext(&g_sContext, &g_sCrystalfontz128x128);
    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
    Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
    GrContextFontSet(&g_sContext, &g_sFontFixed6x8);
    Graphics_clearDisplay(&g_sContext);
    Graphics_drawStringCentered(&g_sContext,
                                    "Light Sensor:",
                                    AUTO_STRING_LENGTH,
                                    64,
                                    30,
                                    OPAQUE_TEXT);
    /* Configures P2.6 to PM_TA0.3 for using Timer PWM to control LCD backlight */
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN6,
            GPIO_PRIMARY_MODULE_FUNCTION);
            
    /* Configuring Timer_A0 for Up Mode and starting */
    MAP_Timer_A_configureUpMode(TIMER_A0_BASE, &upConfig);
    MAP_Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);
    
    /* Initialize compare registers to generate PWM */
    MAP_Timer_A_initCompare(TIMER_A0_BASE, &compareConfig_PWM);
    
    /* Initialize I2C communication */
    Init_I2C_GPIO();
    I2C_init();
    
    /* Initialize OPT3001 digital ambient light sensor */
    OPT3001_init();
    
    /* Configuring UART Module */
    MAP_UART_initModule(EUSCI_A2_BASE, &uartConfig);

    /* Enable UART module */
    MAP_UART_enableModule(EUSCI_A2_BASE);

    /* Enabling interrupts */
    MAP_UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA2);
    MAP_Interrupt_enableSleepOnIsrExit();

    //__delay_cycles(100000);
    
    while(1)
    {
        MAP_UART_transmitData(EUSCI_A2_BASE, TXData);
        
        
        MAP_Interrupt_enableSleepOnIsrExit();
        MAP_PCM_gotoLPM0InterruptSafe();
    }
}

/* EUSCI A0 UART ISR - Echos data back to PC host */
void EUSCIA2_IRQHandler(void)
{
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A2_BASE);
    lux = OPT3001_getLux();
    
        char string [20];
        sprintf(string, "%f", lux);
        Graphics_drawStringCentered(&g_sContext, 
                                    (int8_t *)string,
                                    6,
                                    48,
                                    50,
                                    OPAQUE_TEXT);
        sprintf(string, "lux");
        Graphics_drawStringCentered(&g_sContext,
                                    (int8_t *)string,
                                    3,
                                    86,
                                    50,
                                    OPAQUE_TEXT);
        sprintf(string, "RXData");
        Graphics_drawStringCentered(&g_sContext,
                                    (int8_t *)string,
                                    6,
                                    48,
                                    70,
                                    OPAQUE_TEXT);
        sprintf(string,"%5d", RXData);
        Graphics_drawStringCentered(&g_sContext,
                                    (int8_t *)string,
                                    6,
                                    86,
                                    70,
                                    OPAQUE_TEXT);
        sprintf(string, "TXData");
        Graphics_drawStringCentered(&g_sContext,
                                    (int8_t *)string,
                                    6,
                                    48,
                                    90,
                                    OPAQUE_TEXT);
        sprintf(string,"%5d", TXData);
        Graphics_drawStringCentered(&g_sContext,
                                    (int8_t *)string,
                                    6,
                                    86,
                                    90,
                                    OPAQUE_TEXT);
    if(lux < 150){
        TXData = 1;
    }else{
        TXData = 0;
    }
    MAP_UART_clearInterruptFlag(EUSCI_A2_BASE, status);

    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        RXData = MAP_UART_receiveData(EUSCI_A2_BASE);

        if(RXData == 1)              // Check value
        {
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
            //while(1);                       // Trap CPU
        }else{
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
        }
        //TXData++;
        MAP_Interrupt_disableSleepOnIsrExit();
    }

}
