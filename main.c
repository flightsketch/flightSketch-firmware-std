/* 
 * File:   main.c
 * Author: russ
 *
 * Created on April 4, 2018, 8:54 PM
 */
#include <stdio.h>
#include <stdlib.h>
#include "mcc_generated_files/mcc.h"
#define FCY _XTAL_FREQ/2
#include <libpic30.h>






/*
 * 
 */
int main(int argc, char** argv) {

    SYSTEM_Initialize();


    while(1){
        BMP280_CS_SetLow();
        UART1_Write(SPI1_Exchange8bit(0xD0));
        UART1_Write(SPI1_Exchange8bit(0xD0));
        BMP280_CS_SetHigh();
        __delay_ms(1000);
    }
    return (EXIT_SUCCESS);
}

