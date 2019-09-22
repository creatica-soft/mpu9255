//MPU-9250 - 3.3V 3-axis accelerometer, 3-axis gyroscope and 3-axis magnetometer, internal temperature sensor plus DMP processor in a very small form factor
//Has I2C and SPI interfaces as well as EDA and ECL pins for auxilary i2c bus, where MPU acts as a master or the bus works in pass-through mode
//Also, has INT pin for programmable interrupts
//This sketch uses I2C (ports 20 - SDA and 21 - SCL on Arduino Mega via level shifter)

#include <Wire.h>
#include <math.h>

//to enable SPI and disable i2c, CSB (chip select) -> GND
#define MPU_I2C_ADDR_PRIM 0x68 //AD0 low
#define MPU_I2C_ADDR_SEC 0x69 //AD0 high
#define AKM_I2C_ADDR 0xC //Magnetometer i2c address to connect in pass-through mode

#define MPU_DEVICE_ID 0x73 //decimal 115
#define AKM_DEVICE_ID 0x48 //decimal 72

//Register names of gyroscope and accelerometer

#define SELF_TEST_X_GYRO 0
#define SELF_TEST_Y_GYRO 1
#define SELF_TEST_Z_GYRO 2
#define SELF_TEST_X_ACCEL 13
#define SELF_TEST_Y_ACCEL 14
#define SELF_TEST_Z_ACCEL 15

//Offset in LSB = X_OFFS_USR * 4 / 2^FS_SEL (for FS_SEL values see GYRO_FS_SEL_XXXX: 0 - for 250, 1 - for 500, 2 - for 1000 and 4 - for 2000)
//Offset in DPS = X_OFFS_USR * 4 / 2^FS_SEL / Gyro_Sensitivity
//Nominal Conditions: FS_SEL = 0, Gyro_Sensitivity = 2^16 LSB / 500dps = 131
//Max 999.969 dps, Min -1000 dps, Step 0.0305 dps
//All gyro offset registers are 0 at boot
#define XG_OFFSET_H 19
#define XG_OFFSET_L 20
#define YG_OFFSET_H 21
#define YG_OFFSET_L 22
#define ZG_OFFSET_H 23
#define ZG_OFFSET_L 24

#define SMPLRT_DIV 25 //used for 1kHz internal sampling rate only (CONFIG_DLPF_1 through CONFIG_DLPF_6, A_CONFIG_DLPF_0 through A_CONFIG_DLPF_7 and [A_]FCHOICE_B 0)
#define CONFIG 26
#define GYRO_CONFIG 27
#define ACCEL_CONFIG 28
#define ACCEL_CONFIG_2 29
#define LP_ACCEL_ODR 30
#define WOM_THR 31
#define FIFO_EN 35
#define I2C_MST_CTRL 36

#define I2C_SLV0_ADDR 37
#define I2C_SLV0_REG 38
#define I2C_SLV0_CTRL 39
#define I2C_SLV1_ADDR 40
#define I2C_SLV1_REG 41
#define I2C_SLV1_CTRL 42
#define I2C_SLV2_ADDR 43
#define I2C_SLV2_REG 44
#define I2C_SLV2_CTRL 45
#define I2C_SLV3_ADDR 46
#define I2C_SLV3_REG 47
#define I2C_SLV3_CTRL 48
#define I2C_SLV4_ADDR 49
#define I2C_SLV4_REG 50
#define I2C_SLV4_DO 51
#define I2C_SLV4_CTRL 52
#define I2C_SLV4_DI 53

#define I2C_MST_STATUS 54

#define INT_PIN_CFG 55 //also used to set bypass mode for i2c (bit 1)
#define INT_ENABLE 56 
#define INT_STATUS 58
#define ACCEL_XOUT_H 59
#define ACCEL_XOUT_L 60
#define ACCEL_YOUT_H 61
#define ACCEL_YOUT_L 62
#define ACCEL_ZOUT_H 63
#define ACCEL_ZOUT_L 64
#define TEMP_OUT_H 65
#define TEMP_OUT_L 66
#define GYRO_XOUT_H 67
#define GYRO_XOUT_L 68
#define GYRO_YOUT_H 69
#define GYRO_YOUT_L 70
#define GYRO_ZOUT_H 71
#define GYRO_ZOUT_L 72

#define EXT_SENS_DATA_00 73
#define EXT_SENS_DATA_01 74
#define EXT_SENS_DATA_02 75
#define EXT_SENS_DATA_03 76
#define EXT_SENS_DATA_04 77
#define EXT_SENS_DATA_05 78
#define EXT_SENS_DATA_06 79
#define EXT_SENS_DATA_07 80
#define EXT_SENS_DATA_08 81
#define EXT_SENS_DATA_09 82
#define EXT_SENS_DATA_10 83
#define EXT_SENS_DATA_11 84
#define EXT_SENS_DATA_12 85
#define EXT_SENS_DATA_13 86
#define EXT_SENS_DATA_14 87
#define EXT_SENS_DATA_15 88
#define EXT_SENS_DATA_16 89
#define EXT_SENS_DATA_17 90
#define EXT_SENS_DATA_18 91
#define EXT_SENS_DATA_19 92
#define EXT_SENS_DATA_20 93
#define EXT_SENS_DATA_21 94
#define EXT_SENS_DATA_22 95
#define EXT_SENS_DATA_23 96
#define I2C_SLV0_DO 99
#define I2C_SLV1_DO 100
#define I2C_SLV2_DO 101
#define I2C_SLV3_DO 102
#define I2C_MST_DELAY_CTRL 103

#define SIGNAL_PATH_RESET 104
#define MOT_DETECT_CTRL 105
#define USER_CTRL 106
#define PWR_MGMT_1 107
#define PWR_MGMT_2 108
#define FIFO_COUNTH 114
#define FIFO_COUNTL 115
#define FIFO_R_W 116
#define WHO_AM_I 117

//Accelerometer offsets. +/-16g offset cancellation in all Full Scale modes, 15 bit 0.98-mg steps (1mg = 2048 LSB)
//The lowest bit in *_L accel offset registers is reserved for temperature compensation
//At boot accel offset registers contain factory trim values
#define XA_OFFSET_H 119
#define XA_OFFSET_L 120
#define YA_OFFSET_H 122
#define YA_OFFSET_L 123
#define ZA_OFFSET_H 125
#define ZA_OFFSET_L 126

#define PWR_MGMT_1_H_RESET 128 //Reset the internal registers and restores the default settings. Write a 1 to set the reset, the bit will auto clear
#define PWR_MGMT_1_SLEEP 64 //When set, the chip is set to sleep mode (After OTP loads, the PU_SLEEP_MODE bit will be written here)
#define PWR_MGMT_1_CYCLE 32 //When set, and SLEEP and STANDBY are not set, the chip will cycle between sleep and taking a single sample at a rate determined by LP_ACCEL_ODR register
                            //NOTE: When all accelerometer axis are disabled via PWR_MGMT_2 register bits and cycle is enabled, 
                            //the chip will wake up at the rate determined by the respective registers above, but will not take any samples
#define PWR_MGMT_1_GYRO_STANDBY 16 //When set, the gyro drive and pll circuitry are enabled, but the sense paths are disabled. This is a low power mode that allows quick enabling of the gyros
#define PWR_MGMT_1_PD_PTAT 8 //Power down internal PTAT voltage generator and PTAT ADC
//CLKSEL bits are 0, 1, 2
#define PWR_MGMT_1_CLKSEL_STOP 7 //Stops the clock and keeps timing generator in reset
#define PWR_MGMT_1_CLKSEL_AUTO 1 //Auto selects the best available clock source – PLL if ready, else use the Internal oscillator, same as 2,3,4 and 5
#define PWR_MGMT_1_CLKSEL_20MHZ 0 //Internal 20MHz oscillator, same as 6
//NOTE: After OTP loads, the inverse of PU_SLEEP_MODE bit will be written to CLKSEL[0], so that during sleep Internal clock is enabled
//and when not asleep, the best clock will be chosen

#define PWR_MGMT_2_DISABLE_XA 32 //X accelerometer is disabled
#define PWR_MGMT_2_DISABLE_YA 16 //Y accelerometer is disabled
#define PWR_MGMT_2_DISABLE_ZA 8 //Z accelerometer is disabled
#define PWR_MGMT_2_DISABLE_XG 4 //X gyro is disabled
#define PWR_MGMT_2_DISABLE_YG 2 //Y gyro is disabled
#define PWR_MGMT_2_DISABLE_ZG 1 //Z gyro is disabled
#define PWR_MGMT_2_ENABLE_6_AXIS 0 //enable all axises of gyro and accel

//bits 0..2 of CONFIG register - FCHOICE_B must be 0 for these values to take effect
#define CONFIG_DLPF_0 0 //GYRO bandwidth 250Hz, delay 0.97ms, Sampling rate 8kHz, temperature bandwidth 4000Hz, delay 0.04ms
#define CONFIG_DLPF_1 1 //GYRO bandwidth 184Hz, delay 2.9ms, Sampling rate 1kHz, temperature bandwidth 188Hz, delay 1.9ms
#define CONFIG_DLPF_2 2 //GYRO bandwidth 92Hz, delay 3.9ms, Sampling rate 1kHz, temperature bandwidth 98Hz, delay 2.8ms
#define CONFIG_DLPF_3 3 //GYRO bandwidth 41Hz, delay 5.9ms, Sampling rate 1kHz, temperature bandwidth 42Hz, delay 4.8ms
#define CONFIG_DLPF_4 4 //GYRO bandwidth 20Hz, delay 9.9ms, Sampling 1kHz, temperature bandwidth 20Hz, delay 8.3ms
#define CONFIG_DLPF_5 5 //GYRO bandwidth 10Hz, delay 17.85ms, Sampling 1kHz, temperature bandwidth 10Hz, delay 13.4ms
#define CONFIG_DLPF_6 6 //GYRO bandwidth 5Hz, delay 33.48ms, Sampling 1kHz, temperature bandwidth 5Hz, delay 18.6ms
#define CONFIG_DLPF_7 7 //GYRO bandwidth 3600Hz, delay 0.17ms, Sampling 8kHz, temperature bandwidth 4000Hz, delay 0.04ms

//bit 6 of CONFIG register
#define CONFIG_FIFO_NO_OVERWRITE 64 //when FIFO is full, no more data will be written to it

//Fchoice_b - bits 0 and 1 of GYRO_CONFIG register
#define FCHOICE_B_0 0 //inverted value of FCHOICE 11 
#define FCHOICE_B_1 1 //inverted value of FCHOICE 0 - GYRO bandwidth 8800Hz, delay 0.064ms, Sampling 32kHz, temperature bandwidth 4000Hz, delay 0.04ms
#define FCHOICE_B_2 2 //inverted value of FCHOICE 1 - Gyro bandwidth 3600Hz, delay 0.11ms, Sampling 32kHz, temperature bandwidth 4000Hz, delay 0.04ms

//Gyro full scale in dps - bits 3 and 4 of GYRO_CONFIG register
#define GYRO_FS_SEL_250 0 //131 LSB/(º/s) - sensivity scale factor, Gyro_Sensitivity = 2^16 LSB / 500dps
#define GYRO_FS_SEL_500 8 //65.5 LSB/(º/s)
#define GYRO_FS_SEL_1000 16 //32.8 LSB/(º/s)
#define GYRO_FS_SEL_2000 24 //16.4 LSB/(º/s)
#define GYRO_GZ_ST_EN 32 //Z Gyro self-test
#define GYRO_GY_ST_EN 64 //Y Gyro self-test
#define GYRO_GX_ST_EN 128 //X Gyro self-test

//Accelerometer full scale - bits 3 and 4 of ACCEL_CONFIG register
#define ACCEL_FS_SEL_2G 0 //+/-2g - 16,384 LSB/g - sensivity scale factor
#define ACCEL_FS_SEL_4G 8 //+/-4g - 8,192 LSB/g
#define ACCEL_FS_SEL_8G 16 //+/-8g - 4,096 LSB/g
#define ACCEL_FS_SEL_16G 24 //+/-16g - 2,048 LSB/g
#define ACCEL_AZ_ST_EN 32 //Z Accel self-test
#define ACCEL_AY_ST_EN 64 //Y Accel self-test
#define ACCEL_AX_ST_EN 128 //X Accel self-test

//A_Fchoice_b - bit 3 of ACCEL_CONFIG_2 register
#define A_FCHOICE_B_0 0 //inverted value of A_FCHOICE 11 
#define A_FCHOICE_B_1 1 //inverted value of A_FCHOICE 0 - Accel 3dB bandwidth 1,046Hz, Rate 4kHz, filter block Dec1, delay 0.503ms

//bits 0..2 of ACCEL_CONFIG_2 register - A_FCHOICE_B must be 0 for these values to take effect
#define A_CONFIG_DLPF_0 0 //Accel bandwidth 460Hz, Sampling Rate 1kHz, delay 1.94ms
#define A_CONFIG_DLPF_1 1 //Accel bandwidth 184Hz, Sampling Rate 1kHz, delay 5.8ms
#define A_CONFIG_DLPF_2 2 //Accel bandwidth 92Hz, Sampling Rate 1kHz, delay 7.8ms
#define A_CONFIG_DLPF_3 3 //Accel bandwidth 41Hz, Sampling Rate 1kHz, delay 11.8ms
#define A_CONFIG_DLPF_4 4 //Accel bandwidth 20Hz, Sampling Rate 1kHz, delay 19.8ms
#define A_CONFIG_DLPF_5 5 //Accel bandwidth 10Hz, Sampling Rate 1kHz, delay 35.7ms
#define A_CONFIG_DLPF_6 6 //Accel bandwidth 5Hz, Sampling Rate 1kHz, delay 66.96ms
#define A_CONFIG_DLPF_7 7 //Accel bandwidth 460Hz, Sampling Rate 1kHz, delay 1.94ms - same as A_CONFIG_DLPF_0

//low power accelerometer data rate (Hz) - register LP_ACCEL_ODR
#define LP_ACCEL_ODR_0_24 0
#define LP_ACCEL_ODR_0_49 1
#define LP_ACCEL_ODR_0_98 2
#define LP_ACCEL_ODR_1_95 3
#define LP_ACCEL_ODR_3_91 4
#define LP_ACCEL_ODR_7_81 5
#define LP_ACCEL_ODR_15_63 6
#define LP_ACCEL_ODR_31_25 7
#define LP_ACCEL_ODR_62_5 8
#define LP_ACCEL_ODR_125 9
#define LP_ACCEL_ODR_250 10
#define LP_ACCEL_ODR_500 11

#define FIFO_EN_TEMP_OUT 128 //Write TEMP_OUT_H and TEMP_OUT_L to the FIFO at the sample rate; If enabled, buffering of data occurs even if data path is in standby
#define FIFO_EN_GYRO_XOUT 64 //Write GYRO_XOUT_H and GYRO_XOUT_L to the FIFO at the sample rate; If enabled, buffering of data occurs even if data path is in standby
#define FIFO_EN_GYRO_YOUT 32 //Write GYRO_YOUT_H and GYRO_YOUT_L to the FIFO at the sample rate; If enabled, buffering of data occurs even if data path is in standby
#define FIFO_EN_GYRO_ZOUT 16 //Write GYRO_ZOUT_H and GYRO_ZOUT_L to the FIFO at the sample rate; If enabled, buffering of data occurs even if data path is in standby
#define FIFO_EN_ACCEL 8 //write ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L, ACCEL_ZOUT_H, and ACCEL_ZOUT_L to the FIFO at the sample rate;
#define FIFO_EN_SLV_2 4 //write EXT_SENS_DATA registers associated to SLV_2 (as determined by I2C_SL20_CTRL) to the FIFO at the sample rate;
#define FIFO_EN_SLV_1 2 //write EXT_SENS_DATA registers associated to SLV_1 (as determined by I2C_SLV1_CTRL) to the FIFO at the sample rate;
#define FIFO_EN_SLV_0 1 //write EXT_SENS_DATA registers associated to SLV_0 (as determined by I2C_SLV0_CTRL) to the FIFO at the sample rate;
#define FIFO_EN_DISABLE 0 //disable FIFO

#define SIGNAL_PATH_RESET_GYRO_RST 4 //Reset gyro digital signal path. Note: Sensor registers are not cleared. Use SIG_COND_RST to clear sensor registers
#define SIGNAL_PATH_RESET_ACCEL_RST 2 //Reset accel digital signal path. Note: Sensor registers are not cleared. Use SIG_COND_RST to clear sensor registers
#define SIGNAL_PATH_RESET_TEMP_RST 1 //Reset temp digital signal path. Note: Sensor registers are not cleared. Use SIG_COND_RST to clear sensor registers

#define USER_CTRL_FIFO_EN 64 //Enable FIFO operation mode
#define USER_CTRL_I2C_MST_EN 32 //Enable the I2C Master I/F module; pins ES_DA and ES_SCL are isolated from pins SDA/SDI and SCL/ SCLK
#define USER_CTRL_I2C_IF_DIS 16 //Disable I2C Slave module and put the serial interface in SPI mode only
#define USER_CTRL_FIFO_RST 4 //Reset FIFO module. Reset is asynchronous. This bit auto clears after one clock cycle
#define USER_CTRL_DMP_RST 8 //Reset DMP
#define USER_CTRL_I2C_MST_RST 2 //Reset I2C Master module. Reset is asynchronous. This bit auto clears after one clock cycle
                                //NOTE: This bit should only be set when the I2C master has hung. 
                                //If this bit is set during an active I2C master transaction, the I2C slave will hang, 
                                //which will require the host to reset the slave
#define USER_CTRL_SIG_COND_RST 1 //Reset all gyro digital signal path, accel digital signal path, and temp digital signal path. 
                                 //This bit also clears all the sensor registers. SIG_COND_RST is a pulse of one clk8M wide
#define USER_CTRL_DISABLE_FIFO_I2C_MASTER 0 //disables FIFO and i2c master interface

//INT_ENABLE bits
#define WOM_EN 64 //Enable interrupt for wake on motion to propagate to interrupt pin
#define FIFO_OVERFLOW_EN 16 //Enable interrupt for fifo overflow to propagate to interrupt pin
#define FSYNC_INT_EN 8 //Enable Fsync interrupt to propagate to interrupt pin
#define RAW_RDY_EN 1 //Enable Raw Sensor Data Ready interrupt to propagate to interrupt pin. 
                     //The timing of the interrupt can vary depending on the setting in register 36 I2C_MST_CTRL, bit [6] WAIT_FOR_ES
#define DISABLE_INT 0 //disable INT

//I2C_MST_CNTL bits
#define MULT_MST_EN 128 //Enables multi-master capability. When not set, 
                        //clocking to the I2C_MST_IF can be disabled when not in use and the logic to detect lost arbitration is disabled.
#define WAIT_FOR_ES 64 //Delays the data ready interrupt until external sensor data is loaded. If I2C_MST_IF is disabled via USER_CTRL register,
                       //the interrupt will still occur
#define SLV_3_FIFO_EN 32 //write EXT_SENS_DATA registers associated to SLV_3 (as determined by I2C_SLV0_CTRL and I2C_SLV1_CTRL and 
                         //I2C_SLV2_CTRL) to the FIFO at the sample rate
#define I2C_MST_P_NSR 16 //This bit controls the I2C Master’s transition from one slave read to the next slave read. 
                         //If not set, there is a restart between reads. If set, there is a stop between reads.
#define I2C_MST_CLK_364KHZ 15 //i2c master clock speed
#define I2C_MST_CLK_381KHZ 14 //i2c master clock speed
#define I2C_MST_CLK_400KHZ 13 //i2c master clock speed
#define I2C_MST_CLK_421KHZ 12 //i2c master clock speed
#define I2C_MST_CLK_444KHZ 11 //i2c master clock speed
#define I2C_MST_CLK_471KHZ 10 //i2c master clock speed
#define I2C_MST_CLK_500KHZ 9 //i2c master clock speed
#define I2C_MST_CLK_258KHZ 8 //i2c master clock speed
#define I2C_MST_CLK_267KHZ 7 //i2c master clock speed
#define I2C_MST_CLK_276KHZ 6 //i2c master clock speed
#define I2C_MST_CLK_286KHZ 5 //i2c master clock speed
#define I2C_MST_CLK_296KHZ 4 //i2c master clock speed
#define I2C_MST_CLK_308KHZ 3 //i2c master clock speed
#define I2C_MST_CLK_320KHZ 2 //i2c master clock speed
#define I2C_MST_CLK_333KHZ 1 //i2c master clock speed
#define I2C_MST_CLK_348KHZ 0 //i2c master clock speed


//Magnetometer (AK8963) registers
//Addresses from 0 to 13 and from 16 to 18 are compliant with automatic increment function of serial
//interface respectively. Values of addresses from 16 to 18 can be read only in Fuse access mode. In other
//modes, read data is not correct.
#define WIA 0 //DeviceId
#define INFO 1 //Info
#define ST1 2 //Status
#define HXL 3 //x-axis data
#define HXH 4 //x-axis data
#define HYL 5 //y-axis data
#define HYH 6 //y-axis data
#define HZL 7 //z-axis data
#define HZH 8 //z-axis data
#define ST2 9 //Status
#define CNTL 10 //Control
#define ASTC 12 //Self-test
#define ASAX 16 //X-axis sensitivity adjustment value - accessible only in Fuse ROM mode
#define ASAY 17 //Y-axis sensitivity adjustment value - accessible only in Fuse ROM mode
#define ASAZ 18 //Z-axis sensitivity adjustment value - accessible only in Fuse ROM mode

#define ASTC_SELF 64

//MODE[3:0]: Operation mode setting - CNTL register
//Other code settings are prohibited
#define AKM_CNTL_MODE_POWER_DOWN 0 //Power-down mode
#define AKM_CNTL_MODE_SINGLE_MEAS 1 //Single measurement mode
#define AKM_CNTL_MODE_CONT_MEAS_1 2 //Continuous measurement mode 1 (8Hz)
#define AKM_CNTL_MODE_CONT_MEAS_2 6 //Continuous measurement mode 2 (100Hz)
#define AKM_CNTL_MODE_EXT_TRIG_MEAS 4 //External trigger measurement mode
#define AKM_CNTL_MODE_SELF_TEST 8 //Self-test mode
#define AKM_CNTL_MODE_FUSE_ROM_ACCESS 15 //Fuse ROM access mode
#define AKM_CNTL_OUTPUT_16BIT 16 //16-bit output (15μT/LSB), otherwise 14-bit (0.6μT/LSB, bit 5 of CNTL register set to 0). Full scale ±4912μT

//globals: default sensivities
float gyroSensivity = 131, accelSensivity = 16384, temperatureSensivity = 333.87;

struct MagnetSensAdj {
  float x;
  float y;
  float z;
} magnetSensAdj;

struct XYZ {
  int16_t x;
  int16_t y;
  int16_t z;
};

uint8_t readRegister(uint8_t dev, uint8_t addr, uint16_t len, uint8_t * buf) {
  Wire.beginTransmission(dev);
  Wire.write(addr);
  Wire.endTransmission();
  Wire.requestFrom(dev, len);
  uint8_t i = 0;
  while (Wire.available()) {
    buf[i++] = Wire.read();
    if (i >= len) break;
  }
  return i;
}

//return values of writeRegister()
//0:success
//1:data too long to fit in transmit buffer
//2:received NACK on transmit of address
//3:received NACK on transmit of data
//4:other error
uint8_t writeRegister(uint8_t dev, uint8_t addr, uint8_t val) {
  Wire.beginTransmission(dev);
  Wire.write(addr);
  Wire.write(val);
  return Wire.endTransmission();
}

uint8_t getChipId(uint8_t addr, uint8_t reg) {
  uint8_t chipId = 0;
  if (readRegister(addr, reg, 1, &chipId) != 1) {
    Serial.println(F("getChipId() readRegister error"));
    return 0;    
  }
  return chipId;
}

void i2cBypass(bool enable) {
  uint8_t intReg = 0, err;
  uint8_t enableBypass = 2;
  if (readRegister(MPU_I2C_ADDR_PRIM, INT_PIN_CFG, 1, &intReg) != 1) {
    Serial.println(F("i2cBypass() readRegister error"));
    return;    
  }
  intReg = (enable) ? (intReg & (~enableBypass)) | enableBypass : intReg & (~enableBypass);
  if ((err = writeRegister(MPU_I2C_ADDR_PRIM, INT_PIN_CFG, intReg)) != 0)
    Serial.println(String(F("i2cBypass() writeRegister error ")) + err);
}

void setAkmPowerMode(uint8_t mode) {
  uint8_t err;
  if ((err = writeRegister(AKM_I2C_ADDR, CNTL, mode)) != 0)
    Serial.println(String(F("setAkmPowerMode() writeRegister error ")) + err);
  //return writeRegister(AKM_I2C_ADDR, CNTL, mode);
}

void setMpuPowerMode(uint8_t mode1, uint8_t mode2) {
  uint8_t err;
  if ((err = writeRegister(MPU_I2C_ADDR_PRIM, PWR_MGMT_1, mode1)) != 0)
    Serial.println(String(F("setMpuPowerMode() writeRegister error ")) + err);
  if ((err = writeRegister(MPU_I2C_ADDR_PRIM, PWR_MGMT_2, mode2)) != 0)
    Serial.println(String(F("setMpuPowerMode() writeRegister2 error ")) + err);
}

uint8_t getMagnetSensAdj() {
  uint8_t err, xyz[3];
  setAkmPowerMode(AKM_CNTL_MODE_FUSE_ROM_ACCESS);
  delay(100);
  if ((err = readRegister(AKM_I2C_ADDR, ASAX, 3, &xyz[0])) != 3) {
    Serial.println(String(F("compassSelfTest() read data register error ")) + err);
    return err;    
  }
  magnetSensAdj.x = ((int16_t)xyz[0] - 128)/256.0 + 1;
  magnetSensAdj.y = ((int16_t)xyz[1] - 128)/256.0 + 1;
  magnetSensAdj.z = ((int16_t)xyz[2] - 128)/256.0 + 1;
  setAkmPowerMode(AKM_CNTL_MODE_POWER_DOWN);
  delay(100);
  return 0;
}

uint8_t magnetometerSelfTest() {
  uint8_t err, dataReady, xyz[7];
  int8_t bitm;
  float data[3];
  if (getMagnetSensAdj() != 0) Serial.println(F("getMagnetSensAdj() failed"));
  if ((err = writeRegister(AKM_I2C_ADDR, ASTC, ASTC_SELF)) != 0) {
    Serial.println(String(F("compassSelfTest() writeRegister error ")) + err);
    return err;
  }
  setAkmPowerMode(AKM_CNTL_MODE_SELF_TEST);
  delay(100);
  if ((err = readRegister(AKM_I2C_ADDR, ST1, 1, &dataReady)) != 1) {
    Serial.println(String(F("compassSelfTest() read data register error ")) + err);
    return err;    
  }
  while (!(dataReady & 1)) delay(1);
  if ((err = readRegister(AKM_I2C_ADDR, HXL, 7, &xyz[0])) != 7) {
    Serial.println(String(F("compassSelfTest() read data register 2 error ")) + err);
    return err;    
  }
  if ((err = writeRegister(AKM_I2C_ADDR, ASTC, 0)) != 0) { //cancel self-teset
    Serial.println(String(F("compassSelfTest() writeRegister2 error ")) + err);
    return err;
  }
  setAkmPowerMode(AKM_CNTL_MODE_POWER_DOWN);
  data[0] = ((((int16_t)xyz[1]) << 8) | xyz[0]) * magnetSensAdj.x;
  data[1] = ((((int16_t)xyz[3]) << 8) | xyz[2]) * magnetSensAdj.y;
  data[2] = ((((int16_t)xyz[5]) << 8) | xyz[4]) * magnetSensAdj.z;
  
  bitm = (xyz[6] >> 4) & 1; //0 - 14-bit mode, 1 - 16-bit mode
  /*if (bitm == 0) Serial.println(F("compass mode is 14-bit"));
  else Serial.println(F("compass mode is 16-bit"));
  sprintf(s, String(F("compassSelfTest: x = %f, y = %f, z = %f")).c_str(), data[0], data[1], data[2]);
  Serial.println(s);*/
  if ((data[0] < (-50 * (1 + 3 * bitm)) || data[0] > (50 * (1 + 3 * bitm))) || (data[1] < (-50 * (1 + 3 * bitm)) || data[1] > (50 * (1 + 3 * bitm))) || (data[2] < (-800 * (1 + 3 * bitm)) || data[2] > (-200 * (1 + 3 * bitm)))) {
    char s[128];
    sprintf(s, String(F("compassSelfTest failed: x = %f, y = %f, z = %f")).c_str(), data[0], data[1], data[2]);
    Serial.println(s);
    return 1;
  }
  return 0;
}

void calibrateMpu() {
  uint8_t data[12];
  uint16_t samplesNumber;
  int32_t gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};
  setMpuPowerMode(PWR_MGMT_1_H_RESET, 0); //reset registers
  delay(100);
  setMpuPowerMode(PWR_MGMT_1_CLKSEL_AUTO, PWR_MGMT_2_ENABLE_6_AXIS); //auto select clock and enabled all axis of gyro and accel
  delay(200);
  enableInt(DISABLE_INT); //disable INT
  delay(100);
  enableFifo(FIFO_EN_DISABLE); //disable FIFO writes by DMA
  delay(100);
  setMpuPowerMode(PWR_MGMT_1_CLKSEL_20MHZ, PWR_MGMT_2_ENABLE_6_AXIS); //select internal clock
  delay(100);
  i2cMasterControl(0); //resets i2c master control registers
  delay(100);
  userControl(USER_CTRL_DISABLE_FIFO_I2C_MASTER); //disables FIFO access from serial interface and disables i2c master interface
  delay(100);
  userControl(USER_CTRL_DMP_RST | USER_CTRL_FIFO_RST); //resets FIFO and DMP
  delay(100);
  confMpu(CONFIG_DLPF_1); //set LPF 184Hz, sampling rate 1kHz, delay 2.9ms
  delay(100);
  sampleRateDivider(0);
  delay(100);
  confGyroscope(GYRO_FS_SEL_250); //set gyroscope  full scale to +/- 250 deg/s
  delay(100);
  confAccel(ACCEL_FS_SEL_2G, A_CONFIG_DLPF_1); //set accelerometer full scale to 2g and LPF to 184Hz, sampling rate 1kHz, delay 5.8ms
  delay(100);
  userControl(USER_CTRL_FIFO_EN); //enable FIFO access from serial interface
  delay(100);
  enableFifo(FIFO_EN_GYRO_XOUT | FIFO_EN_GYRO_YOUT | FIFO_EN_GYRO_ZOUT | FIFO_EN_ACCEL); //allow writing to FIFO gyro and accel data
  delay(40); // collect 40 samples in 40 milliseconds = 480 bytes
  enableFifo(FIFO_EN_DISABLE); //disable FIFO writes by DMA
  samplesNumber = availableFifo() / 12; //one sample is 12 bytes: 6 for gyro and 6 for accel - 40 samples
  for (uint16_t i = 0; i < samplesNumber; i++) {
    readFifo(12, &data[0]);
    accelBias[0] += ((int16_t)data[0] << 8) | data[1];
    accelBias[1] += ((int16_t)data[2] << 8) | data[3];
    accelBias[2] += ((int16_t)data[4] << 8) | data[5];
    gyroBias[0] += ((int16_t)data[6] << 8) | data[7];
    gyroBias[1] += ((int16_t)data[8] << 8) | data[9];
    gyroBias[2] += ((int16_t)data[10] << 8) | data[11];
  }
  accelBias[0] /= samplesNumber;
  accelBias[1] /= samplesNumber;
  accelBias[2] /= samplesNumber;
  gyroBias[0] /= samplesNumber;
  gyroBias[1] /= samplesNumber;
  gyroBias[2] /= samplesNumber;
  
  //remove gravity from the accel Z <= this does not apply to MPU9255 - at rest it supposed to show 0,0,0 for acceleration, not 0,0,+1g
  //as the manual says for MPU9250
  //accelBias[2] = accelBias[2] > 0 ? accelBias[2] - 16384 : accelBias[2] + 16384;

  char s[255];
  sprintf(s, String(F("accelBias: x = %ld, y = %ld, z = %ld; gyroBias: x = %ld, y = %ld, z = %ld")).c_str(), accelBias[0], accelBias[1], accelBias[2], gyroBias[0], gyroBias[1], gyroBias[2]); 
  Serial.println(s);
  
  // Divide by 4 to get 32.8 LSB per deg/s to conform to expected bias input format.
  data[0] = ((-gyroBias[0] / 4) >> 8) & 255;
  data[1] = (-gyroBias[0] / 4) & 255;
  data[2] = ((-gyroBias[1] / 4) >> 8) & 255;
  data[3] = (-gyroBias[1] / 4) & 255;
  data[4] = ((-gyroBias[2] / 4) >> 8) & 255;
  data[5] = (-gyroBias[2] / 4) & 255;

  writeOffset(XG_OFFSET_H, data[0]);
  writeOffset(XG_OFFSET_L, data[1]);
  writeOffset(YG_OFFSET_H, data[2]);
  writeOffset(YG_OFFSET_L, data[3]);
  writeOffset(ZG_OFFSET_H, data[4]);
  writeOffset(ZG_OFFSET_L, data[5]);

  sprintf(s, String(F("gyroOffsets (must be <= 20dps to pass self-test): x = %.1fdps, y = %.1fdps, z = %.1fdps")).c_str(), gyroBias[0]/131.0, gyroBias[1]/131.0, gyroBias[2]/131.0);
  Serial.println(s);
  sprintf(s, String(F("accelOffsets: x = %.3fG, y = %.3fG, z = %.3fG")).c_str(), accelBias[0]/16384.0, accelBias[1]/16384.0, accelBias[2]/16384.0);
  Serial.println(s);

  int32_t accelBiasReg[3] = {0, 0, 0};
  readOffset(XA_OFFSET_H, 2, &data[0]);
  accelBiasReg[0] = ((int16_t)data[0] << 8) | data[1];
  readOffset(YA_OFFSET_H, 2, &data[0]);
  accelBiasReg[1] = ((int16_t)data[0] << 8) | data[1];
  readOffset(ZA_OFFSET_H, 2, &data[0]);
  accelBiasReg[2] = ((int16_t)data[0] << 8) | data[1];

  sprintf(s, String(F("accelBiasReg: x = %ld, y = %ld, z = %ld")).c_str(), accelBiasReg[0], accelBiasReg[1], accelBiasReg[2]); 
  Serial.println(s);

  //need to preserve the lowest bit in accelBiasReg

  for (uint8_t i = 0; i < 3; i++) {
    accelBiasReg[i] -= ((accelBias[i] / 8) & ~1);
  }
  data[0] = (accelBiasReg[0] >> 8) & 255;
  data[1] = (accelBiasReg[0] & 255);
  data[2] = (accelBiasReg[1] >> 8) & 255;
  data[3] = (accelBiasReg[1] & 255);
  data[4] = (accelBiasReg[2] >> 8) & 255;
  data[5] = (accelBiasReg[2] & 255);

  writeOffset(XA_OFFSET_H, data[0]);
  writeOffset(XA_OFFSET_L, data[1]);
  writeOffset(YA_OFFSET_H, data[2]);
  writeOffset(YA_OFFSET_L, data[3]);
  writeOffset(ZA_OFFSET_H, data[4]);
  writeOffset(ZA_OFFSET_L, data[5]);

  sprintf(s, String(F("accelBiasRegCalibrated: x = %.3fG, y = %.3fG, z = %.3fG")).c_str(), accelBiasReg[0]/16384.0, accelBiasReg[1]/16384.0, accelBiasReg[2]/16384.0);
  Serial.println(s);
}

uint8_t mpuSelfTest() {
  uint8_t data[6], selfTest[3], err, error = 0;
  int32_t accelAvg[3] = {0, 0, 0}, gyroAvg[3] = {0, 0, 0}, accelST[3] = {0, 0, 0}, gyroST[3] = {0, 0, 0};
  sampleRateDivider(0); //set 1kHz rate
  confMpu(CONFIG_DLPF_2);
  delay(100);
  confGyroscope(GYRO_FS_SEL_250);
  delay(100);
  confAccel(ACCEL_FS_SEL_2G, A_CONFIG_DLPF_2);
  delay(100);

  for (uint8_t i = 0; i < 200; i++) {
    if ((err = readRegister(MPU_I2C_ADDR_PRIM, ACCEL_XOUT_H, 6, &data[0])) != 6) {
      Serial.println(String(F("mpuSelfTest() read data register error ")) + err);
      return err;
    }
    accelAvg[0] += ((int16_t)data[0] << 8) | data[1];
    accelAvg[1] += ((int16_t)data[2] << 8) | data[3];
    accelAvg[2] += ((int16_t)data[4] << 8) | data[5];
    if ((err = readRegister(MPU_I2C_ADDR_PRIM, GYRO_XOUT_H, 6, &data[0])) != 6) {
      Serial.println(String(F("mpuSelfTest() read data register2 error ")) + err);
      return err;
    }
    gyroAvg[0] += ((int16_t)data[0] << 8) | data[1];
    gyroAvg[1] += ((int16_t)data[2] << 8) | data[3];
    gyroAvg[2] += ((int16_t)data[4] << 8) | data[5];
  }
  for (uint8_t i = 0; i < 3; i++) {
    accelAvg[i] /= 200;
    gyroAvg[i] /= 200;
  }
  confAccel(ACCEL_AX_ST_EN | ACCEL_AY_ST_EN | ACCEL_AZ_ST_EN, A_CONFIG_DLPF_2);
  delay(100);
  confGyroscope(GYRO_GX_ST_EN | GYRO_GY_ST_EN | GYRO_GZ_ST_EN);
  delay(100);
  for (uint8_t i = 0; i < 200; i++) {
    if ((err = readRegister(MPU_I2C_ADDR_PRIM, ACCEL_XOUT_H, 6, &data[0])) != 6) {
      Serial.println(String(F("mpuSelfTest() read data register3 error ")) + err);
      return err;
    }
    accelST[0] += ((int16_t)data[0] << 8) | data[1];
    accelST[1] += ((int16_t)data[2] << 8) | data[3];
    accelST[2] += ((int16_t)data[4] << 8) | data[5];
    if ((err = readRegister(MPU_I2C_ADDR_PRIM, GYRO_XOUT_H, 6, &data[0])) != 6) {
      Serial.println(String(F("mpuSelfTest() read data register4 error ")) + err);
      return err;
    }
    gyroST[0] += ((int16_t)data[0] << 8) | data[1];
    gyroST[1] += ((int16_t)data[2] << 8) | data[3];
    gyroST[2] += ((int16_t)data[4] << 8) | data[5];
  }
  for (uint8_t i = 0; i < 3; i++) {
    accelST[i] /= 200;
    gyroST[i] /= 200;
  }
  confGyroscope(GYRO_FS_SEL_250);
  delay(100);
  confAccel(ACCEL_FS_SEL_2G, A_CONFIG_DLPF_2);
  delay(100);
  if ((err = readRegister(MPU_I2C_ADDR_PRIM, SELF_TEST_X_ACCEL, 3, &selfTest[0])) != 3) {
    Serial.println(String(F("mpuSelfTest() read data register5 error ")) + err);
    return err;
  }
  err = 0;
  float f;
  char s[255], axis;
  for (uint8_t i = 0; i < 3; i++) {
    f = accelST[i] - accelAvg[i];
    switch (i) {
      case 0: axis = 'x'; break;
      case 1: axis = 'y'; break;
      case 2: axis = 'z'; break;
    }
    if (selfTest[i] != 0) {
      f /= 2620 * (pow(1.01, selfTest[i] - 1));
      if (f <= 0.5 || f >= 1.5) {
        sprintf(s, String(F("accelerometer self-test failed (values must be between 0.5 and 1.5): %c = %f")).c_str(), axis, f);
        Serial.println(s); 
        error = 250;       
      }
    }
    else {
      f = abs(f) / 16384;
      if (f < 0.225 || f > 0.675) {
        sprintf(s, String(F("accelerometer self-test failed (absolute values must be between 0.225G and 0.675G): %c = %f")).c_str(), axis, f);
        Serial.println(s);
        error = 251;                
      }
    }
  }

  if ((err = readRegister(MPU_I2C_ADDR_PRIM, SELF_TEST_X_GYRO, 3, &selfTest[0])) != 3) {
    Serial.println(String(F("mpuSelfTest() read data register6 error")) + err);
    return err;
  }
  for (uint8_t i = 0; i < 3; i++) {
    f = gyroST[i] - gyroAvg[i];
    switch (i) {
      case 0: axis = 'x'; break;
      case 1: axis = 'y'; break;
      case 2: axis = 'z'; break;
    }
    if (selfTest[i] != 0) {
      f /= 2620 * (pow(1.01, selfTest[i] - 1));
      if (f <= 0.5) {
        sprintf(s, String(F("gyroscope self-test failed (values must be > 0.5): %c = %f")).c_str(), axis, f);
        Serial.println(s);
        error = 252;        
      }
    }
    else {
      f = abs(f) / 131;
      if (f < 60) {
        sprintf(s, String(F("gyroscope self-test failed (absolute values must be > 60dps): %c = %f")).c_str(), axis, f);
        Serial.println(s);
        error = 253;                
      }
    }
  }
  return error;
}

uint8_t confMpu(uint8_t conf) {
  uint8_t err;
  if ((err = writeRegister(MPU_I2C_ADDR_PRIM, CONFIG, conf)) != 0)
    Serial.println(String(F("confMpu() writeRegister error ")) + err);
  return err;
}

uint8_t confGyroscope(uint8_t conf) {
  uint8_t err, c;
  if ((err = readRegister(MPU_I2C_ADDR_PRIM, GYRO_CONFIG, 1, &c)) != 1) {
    Serial.println(String(F("confGyroscope() read data Registers error ")) + err);
    return err;        
  }
  c = c & ~2; c = c & ~24; c |= conf; //clear bits 0, 1, 3, 4 but preserve the self-test (5,6,7) and reserved bit 2, then add conf
  if ((err = writeRegister(MPU_I2C_ADDR_PRIM, GYRO_CONFIG, c)) != 0)
    Serial.println(String(F("confGyroscope() write register error ")) + err);
  return err;
}

//slaveNum: 0 - 4
//addr: <i2c dev addr> & 127 | (I2C_SLV_RNW << 7), where I2C_SLV_RNW = 0 for write transfer and 1 - for read
//cntl[3:0] - data length to read from this slave, [4] - byte grouping, [5] - would not write reg value, [6] - byte swap, [7] - enable reading from this slave
uint8_t confSlave(uint8_t slaveNum, uint8_t addr, uint8_t reg, uint8_t cntl) {
  if (slaveNum > 4) { Serial.println(F("slaveNum must be <= 4")); return 255; }
  uint8_t err = 0;
  if ((err = writeRegister(MPU_I2C_ADDR_PRIM, I2C_SLV0_ADDR + (3 * slaveNum), addr)) != 0) {
    Serial.println(String(F("confSlave() write register error ")) + err);
    return err;
  }
  if ((err = writeRegister(MPU_I2C_ADDR_PRIM, I2C_SLV0_REG + (3 * slaveNum), reg)) != 0) {
    Serial.println(String(F("confSlave() write register 2 error ")) + err);
    return err;
  }
  switch (slaveNum) {
    case 4:
      if ((err = writeRegister(MPU_I2C_ADDR_PRIM, I2C_SLV4_CTRL, cntl)) != 0) {
        Serial.println(String(F("confSlave() write register 3 error ")) + err);
        return err;
      }
    break;
    default:
      if ((err = writeRegister(MPU_I2C_ADDR_PRIM, I2C_SLV0_CTRL + (3 * slaveNum), cntl)) != 0) {
        Serial.println(String(F("confSlave() write register 3 error ")) + err);
        return err;
      }
    break;
  }
  return err;
}

void confAccel(uint8_t conf1, uint8_t conf2) {
  uint8_t err, c;
  if (readRegister(MPU_I2C_ADDR_PRIM, ACCEL_CONFIG, 1, &c) != 1) {
    Serial.println(F("confAccel() read data register error"));
    return;        
  }
  c = c & ~24; c |= conf1; //clear bits 3 and 4 (ACCEL_FS_SEL) but preserve self-test bits (5,6,7), then add conf1
  if ((err = writeRegister(MPU_I2C_ADDR_PRIM, ACCEL_CONFIG, c)) != 0)
    Serial.println(String(F("confAccel() writeRegister error ")) + err);
  if (readRegister(MPU_I2C_ADDR_PRIM, ACCEL_CONFIG_2, 1, &c) != 1) {
    Serial.println(F("confAccel() read data register2 error"));
    return;        
  }
  c = c & ~16; c |= conf2; //clear bits [0..2] (A_DLPFCFG) and 3 (accel_fchoice_b), then add conf2

  if ((err = writeRegister(MPU_I2C_ADDR_PRIM, ACCEL_CONFIG_2, c)) != 0)
    Serial.println(String(F("confAccel() write register2 error ")) + err);
}

uint8_t getGyroSensivity(float * s) {
  uint8_t err, c;
  if ((err = readRegister(MPU_I2C_ADDR_PRIM, GYRO_CONFIG, 1, &c)) != 1) {
    Serial.println(String(F("getGyroSensivity() read data registers error ")) + err);
    return err;        
  }
  *s = (float)(131.0/(1 + ((c >> 3) & 3)));
  return 0;
}

uint8_t getAccelSensivity(float * s) {
  uint8_t err, c;
  if ((err = readRegister(MPU_I2C_ADDR_PRIM, ACCEL_CONFIG, 1, &c)) != 1) {
    Serial.println(String(F("getAccelSensivity() read data registers error ")) + err);
    return err;        
  }
  *s = (float)(16384.0/(1 + ((c >> 3) & 3)));
  return 0;  
}

//0 means no divider, 1 - means divided by 2, 2 - by 3, 3 - by 4, etc
//SMPLRT_DIV is only used for 1kHz internal sampling
void sampleRateDivider(uint8_t divider) {
  uint8_t err;
  if ((err = writeRegister(MPU_I2C_ADDR_PRIM, SMPLRT_DIV, divider)) != 0)
    Serial.println(String(F("sampleRateDividerccel() writeRegister error ")) + err);  
}

void resetSignalPath(uint8_t path) {
  uint8_t err;
  if ((err = writeRegister(MPU_I2C_ADDR_PRIM, SIGNAL_PATH_RESET, path)) != 0)
    Serial.println(String(F("resetSignalPath() writeRegister error ")) + err);    
}

void enableFifo(uint8_t mode) {
  uint8_t err;
  if ((err = writeRegister(MPU_I2C_ADDR_PRIM, FIFO_EN, mode)) != 0)
    Serial.println(String(F("enableFifo() writeRegister error ")) + err);    
}

uint16_t availableFifo() {
  uint8_t data[2];
  if (readRegister(MPU_I2C_ADDR_PRIM, FIFO_COUNTH, 2, &data[0]) != 2) {
    Serial.println(F("availableFifo() read data register error"));
    return 0;
  }  
  return ((uint16_t)(data[0] & 31) << 8) | data[1];
}

uint16_t readFifo(uint16_t len, uint8_t * buffer) {
  uint8_t l;
  if ((l = readRegister(MPU_I2C_ADDR_PRIM, FIFO_R_W, len, buffer)) != len) {
    Serial.println(F("readFifo() read data register error"));
    return l;
  }
  return len;
}

void writeOffset(uint8_t reg, uint8_t data) {
  uint8_t err;
  if ((err = writeRegister(MPU_I2C_ADDR_PRIM, reg, data)) != 0) {
    Serial.println(String(F("writeOffset() writeRegister error ")) + err);
    return;
  }  
}

uint8_t readOffset(uint8_t reg, uint8_t len, uint8_t * data) {
  uint8_t l;
  if ((l = readRegister(MPU_I2C_ADDR_PRIM, reg, len, data)) != len) {
    Serial.println(String(F("readOffset() read data register error ")) + l);
    return l;
  }
  return len;
}

void enableInt(uint8_t mode) {
  uint8_t err;
  if ((err = writeRegister(MPU_I2C_ADDR_PRIM, INT_ENABLE, mode)) != 0) {
    Serial.println(String(F("enableInt() writeRegister error ")) + err);
    return;
  }  
}

void userControl(uint8_t control) {
  uint8_t err;
  if ((err = writeRegister(MPU_I2C_ADDR_PRIM, USER_CTRL, control)) != 0)
    Serial.println(String(F("userControl() writeRegister error ")) + err);    
}

void i2cMasterControl(uint8_t control) {
  uint8_t err;
  if ((err = writeRegister(MPU_I2C_ADDR_PRIM, I2C_MST_CTRL, control)) != 0) { 
    Serial.println(String(F("i2cMasterControl() writeRegister3 error ")) + err);
    return;
  }  
}

uint8_t checkI2cMaster(uint8_t * status) {
  uint8_t err;
  if ((err = readRegister(MPU_I2C_ADDR_PRIM, I2C_MST_STATUS, 1, status)) != 1) {
    Serial.println(String(F("checkI2cMaster() read data register error ")) + err);
    return err;
  }
  return 0;  
}

uint8_t readGyroscope(XYZ * g) {
  uint8_t xyz[6], err;
  if ((err = readRegister(MPU_I2C_ADDR_PRIM, GYRO_XOUT_H, 6, &xyz[0])) != 6) {
    Serial.println(String(F("readGyroscope() read data Registers error ")) + err);
    return err;
  }
  g->x = (((int16_t)xyz[0]) << 8) | xyz[1];
  g->y = (((int16_t)xyz[2]) << 8) | xyz[3];
  g->z = (((int16_t)xyz[4]) << 8) | xyz[5];
  return 0;
}

void setAccelDataRate(uint8_t rate) {
  uint8_t err;
  if ((err = writeRegister(MPU_I2C_ADDR_PRIM, LP_ACCEL_ODR, rate)) != 0)
    Serial.println(String(F("setAccelDataRate() writeRegister error ")) + err);  
}

uint8_t readAccelerometer(XYZ * a) {
  uint8_t xyz[6], err;
  if ((err = readRegister(MPU_I2C_ADDR_PRIM, ACCEL_XOUT_H, 6, &xyz[0])) != 6) {
    Serial.println(String(F("readAccelerometer() read data Registers error ")) + err);
    return err;    
  }
  a->x = (((int16_t)xyz[0]) << 8) | xyz[1]; //x 
  a->y = (((int16_t)xyz[2]) << 8) | xyz[3]; //y
  a->z = (((int16_t)xyz[4]) << 8) | xyz[5]; //z
  return 0;
}

uint8_t readTemperature(int16_t * t) {
  uint8_t temp[2], err;
  if ((err = readRegister(MPU_I2C_ADDR_PRIM, TEMP_OUT_H, 2, &temp[0])) != 2) {
    Serial.println(String(F("readAccelerometer() read data Registers error ")) + err);
    return err;    
  }
  *t = (((int16_t)temp[0]) << 8) | temp[1];
  return 0;
}

uint8_t readMagnetometer(XYZ * m) {
  uint8_t xyz[7], err = 0, ready;
  //check if data is ready
  if ((err = readRegister(AKM_I2C_ADDR, ST1, 1, &ready)) != 1) {
    Serial.println(String(F("readCompass() read data register error ")) + err);
    return err;    
  }
  while (!(ready & 1)) delay(1);
  if ((err = readRegister(AKM_I2C_ADDR, HXL, 7, &xyz[0])) != 7) {
    Serial.println(String(F("readCompass() read data register error ")) + err);
    return err;    
  }
  /*if (checkI2cMaster(&err) != 0) Serial.println(F("checkI2cMaster() failed"));
  if (err & 1) Serial.println(F("Slave0 receives nack"));
  if ((err = readRegister(MPU_I2C_ADDR_PRIM, EXT_SENS_DATA_00, 7, &xyz[0])) != 7) {
    Serial.println(String(F("readCompass() read data register error ")) + err);
    return err;    
  }*/
  
  //must read ST2 (xyz[6]) to signal the end of data reading
  if ((xyz[6] >> 3) & 1) { Serial.println(F("HOFL: magnetic sensor overflow")); return 255;}
  m->x = (((int16_t)xyz[1] << 8) | xyz[0]) * magnetSensAdj.x;
  m->y = (((int16_t)xyz[3] << 8) | xyz[2]) * magnetSensAdj.y;
  m->z = (((int16_t)xyz[5] << 8) | xyz[4]) * magnetSensAdj.z;
  return 0;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println(F("ready"));
  Wire.begin();
  //Wire.setClock(400000);
  //Just to verify that we talk to the right devices
  Serial.println(String(F("MPU-9250 (gyro+acc) Id: ")) + getChipId(MPU_I2C_ADDR_PRIM, WHO_AM_I));
  calibrateMpu(); //should be called before mpuSelfTest(); otherwise, the later would likely fail
  if (mpuSelfTest() != 0) Serial.println(F("mpuSelfTest failed")); else Serial.println(F("mpuSelfTest passed"));
  calibrateMpu(); //need to call it again - something in mpuSelfTest() resets the offset registers
  i2cBypass(true); //necessary to talk to magnetometer
  delay(100);
  Serial.println(String(F("AK8963 (magnetometer) Id: ")) + getChipId(AKM_I2C_ADDR, WIA));
  magnetSensAdj.x = 1; magnetSensAdj.y = 1; magnetSensAdj.z = 1;
  if (magnetometerSelfTest() == 0) Serial.println(F("magnetometerSelfTest passed"));
  setMpuPowerMode(PWR_MGMT_1_CLKSEL_AUTO, PWR_MGMT_2_ENABLE_6_AXIS); //clear sleep bit
  delay(100);
  confMpu(CONFIG_DLPF_6); //low-pass filter 5Hz
  delay(100);
  confGyroscope(GYRO_FS_SEL_250); // +/- 250 deg/s full scale
  delay(100);
  confAccel(ACCEL_FS_SEL_2G, A_CONFIG_DLPF_6); //+/-2G full scale, low-pass filter 5Hz
  delay(100);
  sampleRateDivider(0); //divides 1kHz internal sampling (FIFO, output data rate ODR) rate = 1kHz / (1 + <divider>)
  if (getMagnetSensAdj() != 0) Serial.println(F("getMagnetSensAdj() failed"));
  /*char s[64];
  sprintf(s, String(F("magnetSensAdj x = %.2f, y = %.2f, z = %.2f")).c_str(), magnetSensAdj.x, magnetSensAdj.y, magnetSensAdj.z);
  Serial.println(s);*/
  if (getGyroSensivity(&gyroSensivity) != 0) Serial.println(F("getGyroSensivity() error"));
  if (getAccelSensivity(&accelSensivity) != 0) Serial.println(F("getAccelSensivity() error"));
  setAkmPowerMode(AKM_CNTL_MODE_CONT_MEAS_1); //14-bit default 8Hz output
  delay(100);


  //configure magnetormeter as a slave 0
  /*i2cBypass(false); //turn the bypass off
  userControl(USER_CTRL_I2C_MST_EN); //turn on i2c master interface
  //i2cMasterControl(MULT_MST_EN | WAIT_FOR_ES | I2C_MST_P_NSR | I2C_MST_CLK_258KHZ);
  i2cMasterControl(MULT_MST_EN | WAIT_FOR_ES | I2C_MST_CLK_258KHZ);
  confSlave(4, AKM_I2C_ADDR, ST1, 1 | 128); 
  confSlave(0, AKM_I2C_ADDR, HXL, 7 | 128); */
}

void loop() {
  char s[128];
  int16_t t = 0;
  uint8_t deg[3] = { 0xc2, 0xb0 }; //unicode degree symbol
  XYZ g, a, m;
  memset((uint8_t *)&g, 0, sizeof(XYZ));
  memset((uint8_t *)&a, 0, sizeof(XYZ));
  memset((uint8_t *)&m, 0, sizeof(XYZ));
  if (readMagnetometer(&m) == 0) {
    //One gauss equals 1×10−4 tesla (100 μT), so 1 tesla = 10,000 gauss.
    int16_t d = atan2(m.x, m.y) * 180 / M_PI - 90; // -90 is needed to align North with x-axis (we use arctan, not arccatan); otherwise, y-axis points to North
    if (d < 0) d = 360 + d;
    sprintf(s, String(F("COG %d%.2sM")).c_str(), (uint16_t)d, deg);
    Serial.println(s);
  } else Serial.println(F("error reading magnetometer"));
  if (readTemperature(&t) == 0) {
    sprintf(s, String(F("Temperature: %.1f%.2sC")).c_str(), t / temperatureSensivity + 21.0, deg);;
    Serial.println(s);
  } else Serial.println(F("error reading temperature"));
  if (readGyroscope(&g) == 0) {
    sprintf(s, String(F("Gyroscope: x(pitching) = %.1f%.2s/s, y(rolling) = %.1f%.2s/s, z(yawning) = %.1f%.2s/s")).c_str(), g.x/gyroSensivity, deg, g.y/gyroSensivity, deg, g.z/gyroSensivity, deg);
    Serial.println(s);
  } else Serial.println(F("error reading gyroscope"));
  if (readAccelerometer(&a) == 0) {
    sprintf(s, String(F("Accelerometer: x = %.3fG, y = %.3fG, z = %.3fG")).c_str(), a.x/accelSensivity, a.y/accelSensivity, a.z/accelSensivity);
    Serial.println(s);
  } else Serial.println(F("error reading accelerometer"));
  delay(1000);
}
