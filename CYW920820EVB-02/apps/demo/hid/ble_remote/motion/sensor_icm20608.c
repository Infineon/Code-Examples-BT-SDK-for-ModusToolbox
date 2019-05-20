/* Copyright Â© 2014-2015 InvenSense Inc. All rights reserved.
This software, related documentation and any modifications thereto (collectively â€œSoftwareâ€�)
is subject to InvenSense and its licensors' intellectual property rights under U.S. and international copyright and other intellectual property rights laws.
InvenSense and its licensors retain all intellectual property and proprietary rights in
and to the Software and any use, reproduction, disclosure or distribution of the
Software without an express license agreement from InvenSense is strictly prohibited. */

#ifdef SUPPORT_MOTION

/* Includes ------------------------------------------------------------------*/
#include "wiced_bt_trace.h"
#include "sensor_icm20608.h"
#include "wiced_hal_i2c.h"


/* Private define ------------------------------------------------------------*/

// Sensor I2C address
#define ICM20608_I2C_ADDR             0x68
#define ICM20608_I2C_ADDR_AD0         0x0 // 0x1

#define ICM20628_I2C_WRITE_BUF_SIZE   12

// Registers
#define ICM20608_REG_SMPLRT_DIV       0x19
#define ICM20608_REG_CONFIG           0x1A
#define ICM20608_REG_GYRO_CONFIG      0x1B
#define ICM20608_REG_ACCEL_CONFIG     0x1C
#define ICM20608_REG_ACCEL_CONFIG2    0x1D
#define ICM20608_REG_LP_MODE_CFG      0x1E
#define ICM20608_REG_ACCEL_WOM_THR    0x1F

#define ICM20608_REG_INT_PIN_CFG      0x37
#define ICM20608_REG_INT_ENABLE       0x38

#define ICM20608_REG_INT_STATUS       0x3A
#define ICM20608_REG_ACCEL_XOUT_H     0x3B

#define ICM20608_REG_GYRO_XOUT_H      0x43

#define ICM20608_REG_ACCEL_INTEL_CTRL 0x69

#define ICM20608_REG_PWR_MGMT_1       0x6B
#define ICM20608_REG_PWR_MGMT_2       0x6C

#define ICM20608_REG_WHO_AM_I         0x75

// Config
#define ICM20628_DLPF_CFG_6           0x06

// Gyro Config
#define ICM20608_GYRO_FS_2000DPS      0x18

// Accel Config
#define ICM20628_ACCEL_FS_8G          0x10
#define ICM20628_ACCEL_FS_16G         0x18

// Accel Config 2
#define ICM20628_ACCEL_DLPF_CFG_1     0x01
#define ICM20628_ACCEL_DLPF_CFG_6     0x06

// Low Power Mode Configuration
#define ICM20628_LPOSC_CLKSEL_8       0x08 // 62.5Hz, 16ms
#define ICM20628_LPOSC_CLKSEL_9       0x09 // 125Hz,  8ms

// INT/DRDY Pin / Bypass Enable Configuration
#define ICM20608_LATCH_INT_EN         0x20
#define ICM20608_INT_RD_CLEAR         0x10

// Interrupt Enable
#define ICM20628_WOM_INT_EN           0xE0

// Accelerometer Intelligence Control
#define ICM20628_ACCEL_INTEL_EN       0x80
#define ICM20628_ACCEL_INTEL_MODE     0x40

// Power Management 1
#define ICM20608_ACCEL_CYCLE          0x20
#define ICM20608_CLKSEL_AUTO          0x01

// Who Am I Value
#define ICM20608_WHOAMI               0xAF

// Pwr Mgmt 2
#define ICM20608_PWR_MGMT_2_ACCEL_OFF 0x38
#define ICM20608_PWR_MGMT_2_GYRO_OFF  0x07

/* Private typedef -----------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

#define REVERT_SIGN(value)   (((value) == -32768) ? 32767 : -(value))

/* Private variables ---------------------------------------------------------*/

// I2C write buffer
/*static uint8_t ICM20608_I2CWriteBuf[ICM20628_I2C_WRITE_BUF_SIZE];
*/
// Power Management 2 reg value
static uint8_t ICM20608_pwrMgmt2Val = (ICM20608_PWR_MGMT_2_ACCEL_OFF | ICM20608_PWR_MGMT_2_GYRO_OFF);

/* Public variables ----------------------------------------------------------*/

/* Function code -------------------------------------------------------------*/

/**
* @brief Register Read
* @param [in] regAddr : which register to read
* @param [out] buf : pointer to buffer to place data
* @param [in] cnt : number of bytes to read
* @return false if error.
*/
/*static void ICM20608_RegRead(uint8_t regAddr, uint8_t *buf, uint8_t cnt)
{
  I2C_Transaction i2cTransaction;

  // I2C transaction
  i2cTransaction.writeCount   = 1;
  i2cTransaction.writeBuf     = &regAddr;
  i2cTransaction.readCount    = cnt;
  i2cTransaction.readBuf      = buf;
  i2cTransaction.slaveAddress = ICM20608_I2C_ADDR;
  I2C_transfer(Board_I2CHandle, &i2cTransaction);
}*/

static void ICM20608_RegRead(uint8_t regAddr, uint8_t *buf, uint8_t cnt)
{
    wiced_hal_i2c_combined_read(buf, cnt, &regAddr, 1, ICM20608_I2C_ADDR | ICM20608_I2C_ADDR_AD0);
}


/**
* @brief Register Write
* @param [in] regAddr : which register to read
* @param [in] buf : pointer to buffer containing data to be written
* @param [in] cnt : number of bytes to write
* @return false if error.
*/
/*static void ICM20608_RegWrite(uint8_t regAddr, const uint8_t *buf, uint8_t cnt)
{
  I2C_Transaction i2cTransaction

  // avoid overflow
  ASSERT((cnt + 1) <= sizeof(ICM20608_I2CWriteBuf));

  // copy in buffer
  ICM20608_I2CWriteBuf[0] = regAddr;
  memcpy(&ICM20608_I2CWriteBuf[1], buf, cnt);

  // I2C transaction
  i2cTransaction.writeCount   = cnt + 1;
  i2cTransaction.writeBuf     = ICM20608_I2CWriteBuf;
  i2cTransaction.readCount    = 0;
  i2cTransaction.readBuf      = NULL;
  i2cTransaction.slaveAddress = ICM20608_I2C_ADDR;
  I2C_transfer(Board_I2CHandle, &i2cTransaction);
}*/

////////////////////////////////////////////////////////////////////////////////
/// Write a byte of data to ICM-20608
///   sub_address is the register address.
///   data is the byte to write
////////////////////////////////////////////////////////////////////////////////
void ICM20608_RegWrite(uint8_t regAddr, const uint8_t *buf, uint8_t cnt)
{
    uint8_t data_array[2];

    data_array[0] = regAddr;
    data_array[1] = buf[0];
    wiced_hal_i2c_write(data_array, 2, ICM20608_I2C_ADDR | ICM20608_I2C_ADDR_AD0);
}


/**
* @brief Configure Gyro
* @return None
*/
static void ICM20608_GYRO_Config(void)
{
  uint8_t buf[1];

  // Noise BW (Hz) 8 Hz, Rate (kHz) 1
  buf[0] = ICM20628_DLPF_CFG_6;
    ICM20608_RegWrite(ICM20608_REG_CONFIG, buf, 1);

  // Sample rate 100 Hz, SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)
  buf[0] = 9;
    ICM20608_RegWrite(ICM20608_REG_SMPLRT_DIV, buf, 1);

  // Gyro 2000 DPS
  buf[0] = ICM20608_GYRO_FS_2000DPS;
    ICM20608_RegWrite(ICM20608_REG_GYRO_CONFIG, buf, 1);

}

/**
* @brief Configure Accel
* @return None
*/
static void ICM20608_ACC_Config(void)
{
  uint8_t buf[1];

  // Full scale
  buf[0] = ICM20628_ACCEL_FS_8G;
    ICM20608_RegWrite(ICM20608_REG_ACCEL_CONFIG, buf, 1);

  // Accel DLPF
  buf[0] = ICM20628_ACCEL_DLPF_CFG_6;
    ICM20608_RegWrite(ICM20608_REG_ACCEL_CONFIG2, buf, 1);
}

/**
* @brief Hardware Initialization for ICM20608
* @return None
*/
//static void ICM20608_HAL_Init(void)
//{
//
//}

/**
* @brief Initialise GPIO for regulator of GYRO
* @return None
*/
void ICM20608_Init(void)
{
  uint8_t buf[1];
WICED_BT_TRACE("ICM20608_Init \n");

//  ICM20608_HAL_Init();
  // Read WHO AM I
  do
  {
    ICM20608_RegRead(ICM20608_REG_WHO_AM_I, buf, 1);
  }
  while (buf[0] != ICM20608_WHOAMI);

  // Auto selects the best available clock source
  buf[0] = ICM20608_CLKSEL_AUTO;
  ICM20608_RegWrite(ICM20608_REG_PWR_MGMT_1, buf, 1);

  ICM20608_GYRO_Config();
  ICM20608_ACC_Config();

  // Check if config is correct
  // Noise BW (Hz) 8 Hz, Rate (kHz) 1
  buf[0] = ICM20628_DLPF_CFG_6;
  ICM20608_RegRead(ICM20608_REG_GYRO_CONFIG, buf, 1);
  ICM20608_RegRead(ICM20608_REG_ACCEL_CONFIG, buf, 1);

  // disable Gyro and Accel
  ICM20608_pwrMgmt2Val = (ICM20608_PWR_MGMT_2_ACCEL_OFF | ICM20608_PWR_MGMT_2_GYRO_OFF);
  ICM20608_RegWrite(ICM20608_REG_PWR_MGMT_2, &ICM20608_pwrMgmt2Val, 1);
}

/**
* @brief Put ICM20608 GYRO only in Power down.
* @return None.
*/
void ICM20608_GYRO_Stop(void)
{
  ICM20608_pwrMgmt2Val |= ICM20608_PWR_MGMT_2_GYRO_OFF;
  ICM20608_RegWrite(ICM20608_REG_PWR_MGMT_2, &ICM20608_pwrMgmt2Val, 1);
}

/**
* @brief Put ICM20608 ACC in Power down
* @return None
*/
void ICM20608_ACC_Stop(void)
{
  ICM20608_pwrMgmt2Val |= ICM20608_PWR_MGMT_2_ACCEL_OFF;
  ICM20608_RegWrite(ICM20608_REG_PWR_MGMT_2, &ICM20608_pwrMgmt2Val, 1);
}

/**
* @brief Put ICM20608 ACC and GYRO in Normal mode.
* @return None.
*/
void ICM20608_GYROACC_Start(void)
{
  // start sensor
WICED_BT_TRACE("gyroacc: start sensor\n");
  ICM20608_pwrMgmt2Val &= ~(ICM20608_PWR_MGMT_2_ACCEL_OFF | ICM20608_PWR_MGMT_2_GYRO_OFF);
  ICM20608_RegWrite(ICM20608_REG_PWR_MGMT_2, &ICM20608_pwrMgmt2Val, 1);
}


/**
* @brief Start accelerometer in motion detection mode
* @return None
*/
void ICM20608_StartMotionDetection(void)
{
    uint8_t buf[1];

    //buf[0] = ICM20608_LATCH_INT_EN;
    buf[0] = ICM20608_LATCH_INT_EN | ICM20608_INT_RD_CLEAR;
    ICM20608_RegWrite(ICM20608_REG_INT_PIN_CFG, buf, 1);

    // Ensure that Accelerometer is running
    // In PWR_MGMT_1 register (0x6B) set CYCLE = 0, SLEEP = 0, and GYRO_STANDBY = 0
    // already done in Init
    // buf[0] = ICM20608_CLKSEL_AUTO;
    // ICM20608_RegWrite(ICM20608_REG_PWR_MGMT_1, buf, 1);

    // In PWR_MGMT_2 register (0x6C) set STBY_XA = STBY_YA = STBY_ZA = 0, and STBY_XG = STBY_YG = STBY_ZG = 1
    // BLTH03513877 Need to keep accelerometer and gyro enabled for motion to operate.
    //ICM20608_pwrMgmt2Val &= (~ICM20608_PWR_MGMT_2_ACCEL_OFF) | ICM20608_PWR_MGMT_2_GYRO_OFF;
    ICM20608_pwrMgmt2Val = 0;
    ICM20608_RegWrite(ICM20608_REG_PWR_MGMT_2, &ICM20608_pwrMgmt2Val, 1);

    // Accelerometer Configuration
    // In ACCEL_CONFIG2 register (0x1D) set ACCEL_FCHOICE_B = 0 and A_DLPF_CFG[2:0] = 1 (b001)
    buf[0] = ICM20628_ACCEL_DLPF_CFG_1;
    ICM20608_RegWrite(ICM20608_REG_ACCEL_CONFIG2, buf, 1);

    // Enable Motion Interrupt
    // In INT_ENABLE register (0x38) set WOM_INT_EN = 111 to enable motion interrupt
    buf[0] = ICM20628_WOM_INT_EN;
    ICM20608_RegWrite(ICM20608_REG_INT_ENABLE, buf, 1);

    // Set Motion Threshold
    // Set the motion threshold in ACCEL_WOM_THR register (0x1F)
    buf[0] = 0x20; // fine tune this
    ICM20608_RegWrite(ICM20608_REG_ACCEL_WOM_THR, buf, 1);

    // Enable Accelerometer Hardware Intelligence
    // In ACCEL_INTEL_CTRL register (0x69) set ACCEL_INTEL_EN = ACCEL_INTEL_MODE = 1; Ensure that bit 0 is set to 0.
#if 0
    // This needs to be fine tuned together with ICM20608_REG_ACCEL_WOM_THR
    buf[0] = ICM20628_ACCEL_INTEL_EN | ICM20628_ACCEL_INTEL_MODE;
#else
    buf[0] = ICM20628_ACCEL_INTEL_EN;// | ICM20628_ACCEL_INTEL_MODE;
#endif
    ICM20608_RegWrite(ICM20608_REG_ACCEL_INTEL_CTRL, buf, 1);

    // Set Frequency of Wake-Up
    // In Low Power Mode Configuration register (0x1E) set LPOSC_CLKSEL[3:0] for a sample rate as indicated in the register map
    buf[0] = ICM20628_LPOSC_CLKSEL_8;
    ICM20608_RegWrite(ICM20608_REG_LP_MODE_CFG, buf, 1);

    // BLTH03513877 motion will not operate with this enabled
    // Enable Cycle Mode (Accelerometer Low-Power Mode)
    // In PWR_MGMT_1 register (0x6B) set CYCLE = 1
    //buf[0] = ICM20608_ACCEL_CYCLE | ICM20608_CLKSEL_AUTO;
    //ICM20608_RegWrite(ICM20608_REG_PWR_MGMT_1, buf, 1);
}

/**
* @brief Disable motion detection mode
* @return None
*/
void ICM20608_StopMotionDetection(void)
{
  uint8_t buf[1];

  // Accel DLPF
  buf[0] = ICM20628_ACCEL_DLPF_CFG_6;
    ICM20608_RegWrite(ICM20608_REG_ACCEL_CONFIG2, buf, 1);

  // Disable Motion interrupt
  buf[0] = 0;
    ICM20608_RegWrite(ICM20608_REG_INT_ENABLE, buf, 1);

  // Disable Cycle Mode (Accelerometer Low-Power Mode)
  buf[0] = ICM20608_CLKSEL_AUTO;
  ICM20608_RegWrite(ICM20608_REG_PWR_MGMT_1, buf, 1);

  // clear it
    ICM20608_RegRead(ICM20608_REG_INT_STATUS, buf, 1);
}


/**
* @brief Get X, Y and Z GYRO informations
* @param [in] BufferData : Buffer for storing data
* @return None
*/
void ICM20608_GYRO_GetSample(int16_t Sample[3])
{
  uint8_t buffer[6];

  ICM20608_RegRead(ICM20608_REG_GYRO_XOUT_H, buffer, sizeof(buffer));

  Sample[0] = ((buffer[0] << 8) | (buffer[1] & 0xFF));
  Sample[1] = ((buffer[2] << 8) | (buffer[3] & 0xFF));
  Sample[2] = ((buffer[4] << 8) | (buffer[5] & 0xFF));
}


/**
* @brief Get X, Y and Z ACC informations
* @param [out] BufferData : Buffer for storing data
* @return None
*/
void ICM20608_ACC_GetSample(int16_t Sample[3])
{
  uint8_t buffer[6];

  ICM20608_RegRead(ICM20608_REG_ACCEL_XOUT_H, buffer, sizeof(buffer));

  Sample[0] = ((buffer[0] << 8) | (buffer[1] & 0xFF));
  Sample[1] = ((buffer[2] << 8) | (buffer[3] & 0xFF));
  Sample[2] = ((buffer[4] << 8) | (buffer[5] & 0xFF));
}

/**
* Change referential
* @param [in/out] Sample : 3 axes gyro sample
* @return None
*/
void ICM20608_GYRO_ChangeRef(int16_t Sample[3])
{
  int16_t memVal;

  memVal = Sample[0];
  Sample[0] = Sample[1];
  Sample[1] = REVERT_SIGN(memVal);
  //Sample[2] = Sample[2];
}

/**
* Change referential
* @param [in/out] Sample : 3 axes accel sample
* @return None
*/
void ICM20608_ACC_ChangeRef(int16_t Sample[3])
{
  int16_t memVal;

  // convert 16 bit to 12 bits
  memVal = Sample[0];
  Sample[0] = (REVERT_SIGN(Sample[1])) >> 4;
  Sample[1] = memVal >> 4;
  Sample[2] = (REVERT_SIGN(Sample[2])) >> 4;
}

/**
* Manage interrupt
* @return None
*/
void ICM20608_ProcessIt(void)
{
  // Motion_EnqueueMsg(MOTION_EVT_MOTION_DETECTED);
}

#endif // SUPPORT_MOTION
