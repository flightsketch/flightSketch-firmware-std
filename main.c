/* 
 * File:   main.c
 * Author: russ
 *
 * Created on April 4, 2018, 8:54 PM
 */
#include "bmp280.h"
#include <stdio.h>
#include <stdlib.h>
#include "mcc_generated_files/mcc.h"
#define FCY _XTAL_FREQ/2
#include <libpic30.h>
#include <math.h>

#define SPI_BUFFER_LEN 30
#define BUFFER_LENGTH	0xFF
#define	SPI_READ	0x80
#define SPI_WRITE	0x7F
#define BMP280_DATA_INDEX	1
#define BMP280_ADDRESS_INDEX	2

#define START_PACKET 0xfd
#define STOP_PACKET 0xfe

int8_t BMP280_SPI_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);
int8_t BMP280_SPI_bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);
void BMP280_delay_msek(uint32_t msek);

struct bmp280_dev bmp;
bool sendData = false;

char update_due = 0;

int32_t alt = 0;
int32_t rawAlt = 0;
int32_t maxAlt = 0;
int32_t refAlt = 0;

int8_t  BMP280_SPI_bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
    __delay_ms(1);
	int32_t iError=BMP280_OK;
	int8_t array[SPI_BUFFER_LEN]={BUFFER_LENGTH};
	uint8_t stringpos;
	array[0] = reg_addr|SPI_READ;

    uint8_t i;
    BMP280_CS_SetLow();
    for (i=0; i<cnt+1; i++){
        array[i] = SPI1_Exchange8bit(array[i]);
    }
    while (SPI1STATbits.SRMPT == false);
    BMP280_CS_SetHigh();
	for (stringpos = 0; stringpos < cnt; stringpos++) {
		*(reg_data + stringpos) = array[stringpos+BMP280_DATA_INDEX];
	}
	return (int8_t)iError;
}

int8_t  BMP280_SPI_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
	int32_t iError=BMP280_OK;

    uint8_t i;

    BMP280_CS_SetLow();
    SPI1_Exchange8bit(reg_addr);
    for (i=0; i<cnt; i++){
        SPI1_Exchange8bit(*(reg_data + i));
    }
    while (SPI1STATbits.SRMPT == false);
    BMP280_CS_SetHigh();
	return (int8_t)iError;
}

void EEPROM_Read(){
//void EEPROM_Write(uint8_t start_address, uint8_t *data_read, uint8_t cnt){
    
    EEPROM_WP_SetHigh();
    EEPROM_HOLD_SetHigh();

    EEPROM_CS_SetLow();
    __delay_us(1);
    SPI1_Exchange8bit(0b00000011);
    __delay_us(1);
    SPI1_Exchange8bit(0b00000000);
    __delay_us(1);
    SPI1_Exchange8bit(0b00000000);
    __delay_us(1);
    SPI1_Exchange8bit(0b00000000);
    __delay_us(1);
    
//    SPI1_Exchange8bit(0b00000101);
    UART1_Write(SPI1_Exchange8bit(0xFF));
//    __delay_ms(5);
//    UART1_Write(SPI1_Exchange8bit(0xFF));
//    
//    __delay_ms(5);
//    UART1_Write(SPI1_Exchange8bit(0xFF));
//    
//    __delay_ms(5);
//    UART1_Write(SPI1_Exchange8bit(0xFF));
//    
//    __delay_ms(5);
    
    while (SPI1STATbits.SRMPT == false);
    EEPROM_CS_SetHigh();
    
}
void EEPROM_Write(){
//void EEPROM_Read(uint8_t start_address, uint8_t *data_read, uint8_t cnt){
    
    EEPROM_WP_SetHigh();
    EEPROM_HOLD_SetHigh();

    EEPROM_CS_SetLow();
    __delay_us(1);
    SPI1_Exchange8bit(0b00000110);
    while (SPI1STATbits.SRMPT == false);
    EEPROM_CS_SetHigh();
    __delay_us(1);
    EEPROM_CS_SetLow();
    __delay_us(1);
    SPI1_Exchange8bit(0b00000010);
    __delay_us(1);
    SPI1_Exchange8bit(0b00000000);
    __delay_us(1);
    SPI1_Exchange8bit(0b00000000);
    __delay_us(1);
    SPI1_Exchange8bit(0b00000000);
    __delay_us(1);
    
    SPI1_Exchange8bit(0x42);
    __delay_us(1);
    while (SPI1STATbits.SRMPT == false);
    EEPROM_CS_SetHigh();
    
    UART1_Write(0x43);
    
}

void  BMP280_delay_msek(uint32_t msek)
{
	__delay_ms(msek);
}

void sendDataPacket(){
    int8_t rslt;
    
    float altf;
    
    struct bmp280_uncomp_data ucomp_data;
    uint8_t meas_dur = bmp280_compute_meas_time(&bmp);
    
    UART1_Write(0x1E);
    //__delay_ms(10);
    UART1_Write(0xA5);
    //__delay_ms(10);
    UART1_Write(0xE1);

    rslt = bmp280_get_uncomp_data(&ucomp_data, &bmp);
    /* Check if rslt == BMP280_OK, if not, then handle accordingly */
//        LATBbits.LATB5 = 0;
    int32_t  temp32 = bmp280_comp_temp_32bit(ucomp_data.uncomp_temp, &bmp);
//        LATBbits.LATB5 = 1;
//        LATBbits.LATB5 = 0;
//        uint32_t pres32 = bmp280_comp_pres_32bit(ucomp_data.uncomp_press, &bmp);
//        LATBbits.LATB5 = 1;
//        LATBbits.LATB5 = 0;
    uint32_t pres64 = bmp280_comp_pres_64bit(ucomp_data.uncomp_press, &bmp);
//        LATBbits.LATB5 = 1;
//        LATBbits.LATB5 = 0;
//        double temp = bmp280_comp_temp_double(ucomp_data.uncomp_temp, &bmp);
//        LATBbits.LATB5 = 1;
//        LATBbits.LATB5 = 0;
//        double pres = bmp280_comp_pres_double(ucomp_data.uncomp_press, &bmp);
//        LATBbits.LATB5 = 1;
//        LATBbits.LATB5 = 0;
    altf = pres64;
    altf = altf/256.0;
    altf = altf/101325.0;
    altf = pow(altf,0.190284);
    altf = 1.0 - altf;
    altf = altf * 145366.45;
    altf = altf * 10;
    
    rawAlt = (int32_t) altf;
    alt = rawAlt - refAlt;
    
    if (alt>maxAlt){
        maxAlt = alt;
    }
    
    
    
//    alt = (uint32_t) ((1.0 - pow((((float) pres64)/((float) 25939200)),0.190284)) * 145366.45 *10.0 +1500.0);
    LATBbits.LATB5 = 1;
    //__delay_ms(10);
    UART1_Write((temp32 >> (0 * 8)) & 0xFF);
    //__delay_ms(10);
    UART1_Write((temp32 >> (1 * 8)) & 0xFF);
    //__delay_ms(10);
    UART1_Write((temp32 >> (2 * 8)) & 0xFF);
    //__delay_ms(10);
    UART1_Write((temp32 >> (3 * 8)) & 0xFF);
    //__delay_ms(10);
    UART1_Write((((alt+10000)) >> (0 * 8)) & 0xFF);
    //__delay_ms(10);
    UART1_Write((((alt+10000)) >> (1 * 8)) & 0xFF);
    //__delay_ms(10);
    UART1_Write((((alt+10000)) >> (2 * 8)) & 0xFF);
    //__delay_ms(10);
    UART1_Write((((alt+10000)) >> (3 * 8)) & 0xFF);
    __delay_ms(10);
    UART1_Write((((maxAlt+10000)) >> (0 * 8)) & 0xFF);
    //__delay_ms(10);
    UART1_Write((((maxAlt+10000)) >> (1 * 8)) & 0xFF);
    //__delay_ms(10);
    UART1_Write((((maxAlt+10000)) >> (2 * 8)) & 0xFF);
    //__delay_ms(10);
    UART1_Write((((maxAlt+10000)) >> (3 * 8)) & 0xFF);
}



void TMR1_CallBack(void){
    update_due = 1;
}

int main(int argc, char** argv) {

    
    
    SYSTEM_Initialize();
    
    UART1_STATUS u1_status;
    u1_status = UART1_StatusGet();
    char rx;
    char test = 0xfd;
    
    BMP280_CS_SetHigh();
    int8_t rslt;
    char tx[] = {0x60, 0xB6};
    BMP280_SPI_bus_write(0x00, 0x00, &tx, 2);
    __delay_ms(10);

    bmp.dev_id  =  0;
    bmp.intf = BMP280_SPI_INTF;
    bmp.read = BMP280_SPI_bus_read;
    bmp.write = BMP280_SPI_bus_write;
    bmp.delay_ms = BMP280_delay_msek;

    rslt = bmp280_init(&bmp);

    struct bmp280_config conf;

    rslt = bmp280_get_config(&conf, &bmp);

    conf.filter = BMP280_FILTER_COEFF_8;
    conf.os_pres = BMP280_OS_16X;
    conf.os_temp = BMP280_OS_1X;
    conf.odr = BMP280_ODR_0_5_MS;

    rslt = bmp280_set_config(&conf, &bmp);
    /* Check if rslt == BMP280_OK, if not, then handle accordingly */
    rslt = bmp280_get_config(&conf, &bmp);
    /* Always set the power mode after setting the configuration */
    rslt = bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp);
    /* Check if rslt == BMP280_OK, if not, then handle accordingly */
    rslt = bmp280_get_config(&conf, &bmp);

    TMR1_Start();
    
    while(1){
        if (update_due > 0){
            update_due = 0;
            if (sendData){
                sendDataPacket();
            }
        }
        
        if (UART1_TRANSFER_STATUS_RX_DATA_PRESENT & UART1_TransferStatusGet()){
            rx = UART1_Read();
            if (rx == test){
                sendData = true;
            }
        }
    }
    return (EXIT_SUCCESS);
}

