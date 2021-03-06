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
#include "mcc_generated_files/adc1.h"
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
bool sendData = true;
bool init = false;
bool pyro1Fire = false;
int  pyro1Count = 0;
int32_t deployAlt = 2500;
int32_t armAlt = 3000;
int pyro1Dur = 300;

char update_due = 0;
char send_due = 0;
bool inHead = false;
bool inData = false;
char headCount = 0;
char dataCount = 0;
char packetType = 0;
char dataLength = 0;
unsigned char checksum = 0;
unsigned char startByte = 0xF5;
unsigned char dataIn[16];

bool recordData = false;
int recordInterval = 0;
int recordCount = 0;
int debugCount = 0;

union dataAddress{
    unsigned long full;
    unsigned char bytes[4];
}da;

union altData {
    long full;
    unsigned char bytes[4];
}ad;

union accData {
    long full;
    unsigned char bytes[4];
    
}accd;

unsigned long fileLength;


int32_t currentAltInt = 0;
float rawAlt = 0;
int32_t maxAlt = 0;
float refAlt = 0;

float currentAlt = 0;
float currentAltMeas = 0;
float currentV = 0;
float maxV = 0;
float currentAcc = 0;

float E = 0;

float K1 = 0.097128094515918;
float K2 = 0.248046634941573;
float K3 = 0.316731906522445;

float dt = 0.020;
float t1;
float t2;

int32_t  temp;

union data {
    unsigned char dataBytes[12];
    struct dataPacket {
        int32_t temp;
        int32_t alt;
        int32_t maxAlt;
    }dp;
}du;

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

union altData EEPROM_Read(union dataAddress address){
//void EEPROM_Write(uint8_t start_address, uint8_t *data_read, uint8_t cnt){
    
    union altData data;
    EEPROM_WP_SetHigh();
//    __delay_us(1000);
    EEPROM_HOLD_SetHigh();
//    __delay_us(1000);

    EEPROM_CS_SetLow();
//    __delay_us(1000);
    SPI1_Exchange8bit(0b00000011);
//    __delay_us(1000);
    SPI1_Exchange8bit(address.bytes[2]);
//    __delay_us(1000);
    SPI1_Exchange8bit(address.bytes[1]);
//    __delay_us(1000);
    SPI1_Exchange8bit(address.bytes[0]);
//    __delay_us(1000);
    
//    SPI1_Exchange8bit(0b00000101);
    data.bytes[0] = SPI1_Exchange8bit(0xFF);
//    __delay_us(1000);
    data.bytes[1] = SPI1_Exchange8bit(0xFF);
//    __delay_us(1000);
    data.bytes[2] = SPI1_Exchange8bit(0xFF);
//    __delay_us(1000);
    data.bytes[3] = SPI1_Exchange8bit(0xFF);
//    __delay_us(1000);
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
//    __delay_ms(1);
    EEPROM_CS_SetHigh();
    
    return(data);
    
}
void EEPROM_Write(union dataAddress address, unsigned char bytes[]){
//void EEPROM_Read(uint8_t start_address, uint8_t *data_read, uint8_t cnt){
    int i;
    EEPROM_CS_SetLow();
//    __delay_us(10000);
    EEPROM_WP_SetHigh();   // remove write protect
//    __delay_ms(10);
    EEPROM_HOLD_SetHigh(); // remove hold condition
//    __delay_ms(10);
    
    SPI1_Exchange8bit(0b00000110); // enable writes
    while (SPI1STATbits.SRMPT == false);
//    __delay_ms(10);
    EEPROM_CS_SetHigh();
//    __delay_us(10000);
    EEPROM_CS_SetLow();
//    __delay_us(10000);
    SPI1_Exchange8bit(0b00000010); // write command
//    __delay_us(10000);
    SPI1_Exchange8bit(address.bytes[2]); // address high
//    __delay_us(10000);
    SPI1_Exchange8bit(address.bytes[1]); // address mid
//    __delay_us(10000);
    SPI1_Exchange8bit(address.bytes[0]); // address low
//    __delay_us(10000);
    
    for (i=0; i<4; i++){
        SPI1_Exchange8bit(bytes[i]); // data
//        __delay_us(10000);
    }
    
    while (SPI1STATbits.SRMPT == false);
//    __delay_ms(10);
    EEPROM_CS_SetHigh();
    
}

void writeData(){
    union altData altWrite;
    altWrite.full = currentAltMeas*10 + 10000;
    union dataAddress altWriteAddress;
    altWriteAddress.full = fileLength;
    EEPROM_Write(altWriteAddress, altWrite.bytes);
    fileLength = fileLength + 4;
    __delay_ms(10);
    altWrite.full = currentAlt*10 + 10000;
    altWriteAddress.full = fileLength;
    EEPROM_Write(altWriteAddress, altWrite.bytes);
    fileLength = fileLength + 4;

}

void  BMP280_delay_msek(uint32_t msek)
{
	__delay_ms(msek);
}

void sendDataPacket(){
    
    unsigned char dataByte = 0x00;
    unsigned char checksum = 0x00;
    unsigned char i = 0;
    
    UART1_Write(0xF5);  // FS packet start
    UART1_Write(0x01);  // Packet type, data type 1
    __delay_us(500);
    UART1_Write(0x0C);  // Payload length
    UART1_Write(0x02);  // Header checksum
    
    du.dp.alt = currentAltInt + 10000;
    du.dp.maxAlt = maxAlt + 10000;
    du.dp.temp = temp;

    for (i=0; i<8; i++){
        UART1_Write(du.dataBytes[i]);
        checksum = checksum + du.dataBytes[i];
    }
    
//    dataByte = (temp >> (0 * 8)) & 0xFF;
//    UART1_Write(dataByte);
//    checksum = dataByte;
//    dataByte = (temp >> (1 * 8)) & 0xFF;
//    UART1_Write(dataByte);
//    checksum = checksum + dataByte;
//    dataByte = (temp >> (2 * 8)) & 0xFF;
//    UART1_Write(dataByte);
//    checksum = checksum + dataByte;
//    dataByte = (temp >> (3 * 8)) & 0xFF;
//    UART1_Write(dataByte);
//    checksum = checksum + dataByte;
//    
//    dataByte = (((alt+10000)) >> (0 * 8)) & 0xFF;
//    UART1_Write((((alt+10000)) >> (0 * 8)) & 0xFF);
//    UART1_Write((((alt+10000)) >> (1 * 8)) & 0xFF);
//    UART1_Write((((alt+10000)) >> (2 * 8)) & 0xFF);
//    UART1_Write((((alt+10000)) >> (3 * 8)) & 0xFF);
    __delay_us(500);
    
    for (i=8; i<12; i++){
        UART1_Write(du.dataBytes[i]);
        checksum = checksum + du.dataBytes[i];
    }
    UART1_Write(checksum);
    
//    UART1_Write((((maxAlt+10000)) >> (0 * 8)) & 0xFF);
//    UART1_Write((((maxAlt+10000)) >> (1 * 8)) & 0xFF);
//    UART1_Write((((maxAlt+10000)) >> (2 * 8)) & 0xFF);
//    UART1_Write((((maxAlt+10000)) >> (3 * 8)) & 0xFF);
//    UART1_Write(0x99);
}



void TMR1_CallBack(void){
    update_due++;
}

void getAlt(){
    int8_t rslt;
    
    float altf;
    
    struct bmp280_uncomp_data ucomp_data;
    uint8_t meas_dur = bmp280_compute_meas_time(&bmp);
    
    rslt = bmp280_get_uncomp_data(&ucomp_data, &bmp);
    
    temp = bmp280_comp_temp_32bit(ucomp_data.uncomp_temp, &bmp);
    uint32_t pres64 = bmp280_comp_pres_64bit(ucomp_data.uncomp_press, &bmp);
    
    altf = pres64;
    altf = altf/256.0;
    altf = altf/101325.0;
    altf = pow(altf,0.190284);
    altf = 1.0 - altf;
    altf = altf * 145366.45;
    
    rawAlt = altf;
    currentAltMeas = rawAlt - refAlt;
    
    currentAlt = currentAlt + t1*currentV + t2*currentAcc;
    currentV = currentV + t1*currentAcc;
    
    E = currentAltMeas - currentAlt;
    
    currentAlt = currentAlt + K1 * E;
    currentV = currentV + K2 * E;
    currentAcc = currentAcc + K3 * E;
    
    currentAltInt = (currentAlt*10);
    
    
    if (currentAltInt>maxAlt){
        maxAlt = currentAltInt;
    }
}

void accInit(){
    union accData dataRx1;
    union accData p11;
    unsigned char accSum = 0;
    __delay_ms(1000);
    LATBbits.LATB14 = 0;
    p11.bytes[3] = SPI1_Exchange8bit(0x50);
    p11.bytes[2] = SPI1_Exchange8bit(0x00);
    p11.bytes[1] = SPI1_Exchange8bit(0x20);
    p11.bytes[0] = SPI1_Exchange8bit(0x7F);
    while (SPI1STATbits.SRMPT == false);
    LATBbits.LATB14 = 1;
    __delay_us(1);
    LATBbits.LATB14 = 0;
    dataRx1.bytes[3] = SPI1_Exchange8bit(0x50);
    dataRx1.bytes[2] = SPI1_Exchange8bit(0x00);
    dataRx1.bytes[1] = SPI1_Exchange8bit(0x20);
    dataRx1.bytes[0] = SPI1_Exchange8bit(0x7F);
    while (SPI1STATbits.SRMPT == false);
    LATBbits.LATB14 = 1;
    
}

void getAcc(){
    
    union accData dataRx;
    union accData p1;
    unsigned char accSum = 0;
    LATBbits.LATB14 = 0;
    //__delay_us(100);
    p1.bytes[3] = SPI1_Exchange8bit(0x20);
    //__delay_us(100);
    p1.bytes[2] = SPI1_Exchange8bit(0x00);
    //__delay_us(100);
    p1.bytes[1] = SPI1_Exchange8bit(0x00);
    //__delay_us(100);
    p1.bytes[0] = SPI1_Exchange8bit(0x58);
    //__delay_us(100);
    while (SPI1STATbits.SRMPT == false);
    LATBbits.LATB14 = 1;
    __delay_us(1);
    
    LATBbits.LATB14 = 0;
    //__delay_us(100);
    dataRx.bytes[3] = SPI1_Exchange8bit(0x20);
    //__delay_us(100);
    dataRx.bytes[2] = SPI1_Exchange8bit(0x00);
    //__delay_us(100);
    dataRx.bytes[1] = SPI1_Exchange8bit(0x00);
    //__delay_us(100);
    dataRx.bytes[0] = SPI1_Exchange8bit(0x58);
    //__delay_us(100);
    while (SPI1STATbits.SRMPT == false);
    LATBbits.LATB14 = 1;
    
//    UART1_Write(0xF5);
//    UART1_Write(0x06);
//    UART1_Write(0x04);
//    UART1_Write(0xFF);
//    __delay_ms(10);
//    UART1_Write(dataRx.bytes[3]);
//    accSum = accSum + dataRx.bytes[3];
//    UART1_Write(dataRx.bytes[2]);
//    accSum = accSum + dataRx.bytes[2];
//    UART1_Write(dataRx.bytes[1]);
//    accSum = accSum + dataRx.bytes[1];
//    UART1_Write(dataRx.bytes[0]);
//    accSum = accSum + dataRx.bytes[0];
//    UART1_Write(accSum);
    
}

void parseByte(char rx){
    
    if (!inHead || !inData){ //Not in header or data, waiting for start of frame
        
        if (rx == ((char) 0xF5)){ //Start of packet
            //print("start of packet...................")
            
            inHead = true;
            headCount = 0;
            return;
        }
    }
    if (inHead && headCount<3){ //Reading header bytes
        headCount = headCount + 1;
        switch (headCount){
        case 1: packetType = rx;
                break;
        case 2: dataLength = rx;
                break;
        case 3: 
            checksum = (startByte + packetType + dataLength);
            //print("Head checksum = ")
            //print(checksum)
            if (((unsigned char)rx) != checksum){
                inHead = false;
                return;
            }
            else { //Checksum valid, start reading data packet
                
                if (dataLength == (unsigned char) 0){
                    inHead = false; //reset flags to wait for new frame
                    inData = false;
                    switch ((unsigned char) packetType){
                    case 0xF1: parsePacket_typeF1();
                    break;
                    case 0xF2: parsePacket_typeF2();
                    break;
                    case 0xF3: parsePacket_typeF3();
                    break;
                    case 0xF4: parsePacket_typeF4();
                    break;
                    default: maxAlt = 999;
                    break;
                    }
                    return;
                }
                else {
                    inData = true;
                    dataCount = 0;
//                dataArray = Array(repeating: 0, count: Int(dataLength));
                    return;
                }
            }
           
            
        }
    }
    if (inData){ //In data packet
        
        if (dataCount<(dataLength)){ //Read data to array
            dataIn[dataCount] = rx;
            dataCount = dataCount + 1;
            return;
        }

        if (dataCount == dataLength){ //Last byte = checksum
            if (rx != doChecksumIn()){
                inHead = false;
                return;
            }
            else { //Checksum valid, start reading data packet
                inHead = false; //reset flags to wait for new frame
                inData = false;
                switch (packetType){
                case 0xF1: parsePacket_typeF1();
                break;
                case 0xF2: parsePacket_typeF2();
                break;
                }
                return;
            }
        }
    }
}

int doChecksumIn(){
    return 0;
}

void parsePacket_typeF1(){
    maxAlt = 0;
    refAlt = rawAlt;
    init = true;
    pyro1Fire = false;
    pyro1Count = 0;
    currentAlt = 0.0;
    currentV = 0.0;
    currentAcc = 0.0;
    recordData = false;
    fileLength = 0;
}

void parsePacket_typeF2(){ // start data recording 
    fileLength = 0;
    recordData = true;
//    writeData();
//    union altData dataSend;
//    dataSend.bytes[0] = debugCount;
//    dataSend.bytes[1] = debugCount;
//    dataSend.bytes[2] = debugCount;
//    dataSend.bytes[3] = debugCount;
//    debugCount++;
//    
//    union dataAddress add;
//    add.full = 0;
//
//    
//    
//    EEPROM_Write(add, dataSend.bytes);

    
}

void parsePacket_typeF3(){ // stop data recording
    
}

void parsePacket_typeF4(){ // download data
    union altData dataRx;
    unsigned char checksumTest = 0x00;
    union dataAddress readAddress;
    int i;
    int numPackets = fileLength/4;
    
    sendFileHeader();
    
    readAddress.full = 0;
    
    for (i=0; i<numPackets; i++){
        __delay_us(100);
        UART1_Write(0xF5);
        UART1_Write(0x04);
        UART1_Write(0x04);
        UART1_Write(0xFD);
        __delay_us(500);

        dataRx = EEPROM_Read(readAddress);;
        UART1_Write(dataRx.bytes[0]);
        checksumTest = checksumTest + dataRx.bytes[0];
        UART1_Write(dataRx.bytes[1]);
        checksumTest = checksumTest + dataRx.bytes[1];
        UART1_Write(dataRx.bytes[2]);
        checksumTest = checksumTest + dataRx.bytes[2];
        UART1_Write(dataRx.bytes[3]);
        checksumTest = checksumTest + dataRx.bytes[3];
//        __delay_us(100);
//        __delay_ms(10);
        UART1_Write(checksumTest);
        __delay_us(500);
        checksumTest = 0;
        readAddress.full = readAddress.full + 4;
    }
    
    sendFileEOF();
}

void sendFileHeader(){
    unsigned char fileByte = 0x00;
    unsigned char headerChecksum = 0x00;
        UART1_Write(0xF5);
        UART1_Write(0x03);
        UART1_Write(0x04);
        UART1_Write(0xFC);
        fileByte = (fileLength >> (0 * 8)) & 0xFF;
        UART1_Write(fileByte);
        headerChecksum = headerChecksum + fileByte;
        fileByte = (fileLength >> (1 * 8)) & 0xFF;
        UART1_Write(fileByte);
        headerChecksum = headerChecksum + fileByte;
        fileByte = (fileLength >> (2 * 8)) & 0xFF;
        UART1_Write(fileByte);
        headerChecksum = headerChecksum + fileByte;
        fileByte = (fileLength >> (3 * 8)) & 0xFF;
        UART1_Write(fileByte);
        headerChecksum = headerChecksum + fileByte;
        UART1_Write(headerChecksum);
        __delay_us(100);
}

void sendFileEOF(){
        UART1_Write(0xF5);
        UART1_Write(0x05);
        UART1_Write(0x00);
        UART1_Write(0xFA);
        __delay_us(100);
}

int main(int argc, char** argv) {
    
    unsigned char checksum = 0x00;
    
    SYSTEM_Initialize();
    
    t1 = dt;
    t2 = 0.5*dt*dt;
    
    UART1_STATUS u1_status;
    u1_status = UART1_StatusGet();
    unsigned char rx;
    char test = 0xf5;
    int batt = 0;
    
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

    conf.filter = BMP280_FILTER_OFF;
    conf.os_pres = BMP280_OS_8X;
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
    ANSELBbits.ANSB0 = 0;
    TRISBbits.TRISB0 = 0;
    LATBbits.LATB0 = 0;
    

    TRISBbits.TRISB5 = 0;
    LATBbits.LATB5 = 0;
    
    TRISBbits.TRISB14 = 0;
    LATBbits.LATB14 = 1;
    
    accInit();
    
    while(1){
        
        if (update_due > 0){
            LATBbits.LATB5 = 1;
            update_due = 0;
            send_due++;
            getAlt();
            
            getAcc();
            if (pyro1Fire && (pyro1Count <= pyro1Dur)) {
                pyro1Count++;
                if (pyro1Count > pyro1Dur){
                    LATBbits.LATB0 = 0;
                }
            }
            
            if (init && !pyro1Fire) {
                if (maxAlt > armAlt) {
                    if (currentAltInt < deployAlt){
                        pyro1Fire = true;
                        LATBbits.LATB0 = 1;
                    }
                }
            }
            
            if (currentV > 5.0){
                recordData = true;
                
            }
            
            if (fileLength > 2097152){
                recordData = false;
            }
            
            if (fileLength > 24000){
                recordData = false;
            }
            
            if (recordData){
                recordCount = recordCount + 1;
                if (recordCount >= recordInterval){
                    recordCount = 0;
                    writeData();
                }
            }
          
        }
        
        
        
        
        
        
        if (send_due > 5){
            send_due = 0;
            if (sendData){
                sendDataPacket();
            }
            ADC1_SamplingStart();
            __delay_us(50);
            ADC1_SamplingStop();
            batt = ADC1_Channel1ConversionResultGet();
            UART1_Write(0xF5);
            UART1_Write(0x02);
            __delay_us(500);
            UART1_Write(0x02);
            UART1_Write(0xF9);
            UART1_Write(batt);
            UART1_Write(batt>>8);
            checksum = batt;
            checksum = checksum + (batt>>8);
            UART1_Write(checksum);
            checksum = 0x00;
        }
        
        if (UART1_TRANSFER_STATUS_RX_DATA_PRESENT & UART1_TransferStatusGet()){
            
            rx = UART1_Read();
            parseByte(rx);
            if (rx == test){
                
            }
        }
        LATBbits.LATB5 = 0;
    }
    return (EXIT_SUCCESS);
}

