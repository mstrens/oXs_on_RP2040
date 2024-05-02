
//#include "hardware/watchdog.h"
#include "param.h"
#include "tools.h"
#include "config.h"
#include "I2Cdev.h"
#include "helper_3dmath.h"
#include "kx134.h"
#include "pico/util/queue.h"


extern CONFIG config;
//extern queue_t qSendCmdToCore1;



#define KX134_CTL1_VAL 0XD8 // = 1101 1000 = enabled, high perf, no use data ready , 64g (11), no tap/tilt function
#define KX134_CTL1_ADR 0X1B  // address of ctl1 register (enable and set range)

#define KX134_ODCTL_VAL 0X07  // = 100hz (frequence of refresh)
#define KX134_ODCTL_ADR 0X21  // address of ODctl register (frequence of refresh)

#define KX134_XOUT_L_ADR 0X08  // address of X low register

KX134::KX134(int a) {}

void KX134::begin()  // initialise KX134 
{
    if ( config.pinScl == 255 or config.pinSda == 255) return; // skip if pins are not defined
    #ifdef DEBUG  
    printf("Trying to detect KX134 sensor at I2C Addr=%X\n", KX134_DEFAULT_ADDRESS);
    #endif
    uint8_t bufCtl1[2] = {KX134_CTL1_ADR ,0x00};  // set KX134 on pause to allow to change registers
    // test the 4 I2C address  0X1C, 0X1D, 0X1E, 0X1F 
    i2cAdr = 0X1C;
    if ( i2c_write_timeout_us(i2c1, i2cAdr , bufCtl1, 2, false, 1000) < 0) {
        i2cAdr = 0X1D;
        if ( i2c_write_timeout_us(i2c1, i2cAdr , bufCtl1, 2, false, 1000) < 0) {
            i2cAdr = 0X1E;
            if ( i2c_write_timeout_us(i2c1, i2cAdr , bufCtl1, 2, false, 1000) < 0) {
                i2cAdr = 0X1F;
                if ( i2c_write_timeout_us(i2c1, i2cAdr , bufCtl1, 2, false, 1000) < 0) {
                    printf("Write error for first KX134 command\n");
                    return ;
                }
            }
        }            
    }
    printf("KX134 accepts first command on i2c address = %x\n", i2cAdr);
    uint8_t bufOdCtl[2] = {KX134_ODCTL_ADR ,KX134_ODCTL_VAL};  // set refresh rate at 100hz
        if ( i2c_write_timeout_us(i2c1, i2cAdr , bufOdCtl, 2, false, 1000) <0) {
        printf("Write error for KX134 ODCTL reg\n");
        return ;
    }
    bufCtl1[1] = KX134_CTL1_VAL;  // enable KX134 and set range
    if ( i2c_write_timeout_us(i2c1, i2cAdr , bufCtl1, 2, false, 1000) <0) {
        printf("Write error for KX134 CTL1 reg\n");
        return ;
    }
    sleep_us(10000);
    kx134Installed = true;
}


void KX134::getAcc(){
    if (! kx134Installed) return; // skip when not installed
    if ((microsRp() - lastKx134Us) < 2000 ) return; // get the data only once per 20 sec 
    int16_t ax,ay,az ;
    uint8_t accReg = KX134_XOUT_L_ADR ;
    uint8_t buffer[6]; 
    if (i2c_write_timeout_us(i2c1, i2cAdr, &accReg, 1, true,1000)<0) { // true to keep master control of bus
        printf("Write error for KX134 at 0X08\n");
        return ;
    }
    if ( i2c_read_timeout_us(i2c1, i2cAdr, buffer, 6, false, 3500) <0){
        printf("Read error for KX134\n");
        return ;
    } 
    lastKx134Us = microsRp();    
    ax = (int16_t) (buffer[1] << 8 | buffer[0]) ;
    ay = (int16_t) (buffer[3] << 8 | buffer[2]) ;
    az = (int16_t) (buffer[5] << 8 | buffer[4]) ;
    
    sumAx  += ax;
    sumAy  += ay;
    sumAz  += az;
    countSumAcc++;

    uint32_t now_ms = millisRp(); 
    if (now_ms - lastSendAccXYZMs >= 200) {
        lastSendAccXYZMs = now_ms;
        // * 1000 because sport is in mg; mpu is 16 bits = + or - 32768; when ACC max is +/-64g, it gives 32768/64 =  512 steps / g 
        sent2Core0( ACC_X , (int32_t) (sumAx / countSumAcc * 1000 / (int) 512) ) ;  
        sent2Core0( ACC_Y , (int32_t) (sumAy / countSumAcc * 1000 / (int) 512) ) ;  
        sent2Core0( ACC_Z , (int32_t) (sumAz / countSumAcc * 1000 / (int) 512) ) ;
        sumAx  = 0;
        sumAy  = 0;
        sumAz  = 0;
        countSumAcc = 0;
        printf("Acc has been sent to core0\n"); 
    }
}
