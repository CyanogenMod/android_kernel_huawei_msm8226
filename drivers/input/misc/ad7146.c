/**
*\mainpage
* AD7146 Controller Driver
\n
* @copyright 2014 Analog Devices Inc.
\n
* Licensed under the GPL version 2 or later.
* \date      January-2014
* \version   Driver 1.4.1
* \version   Android Jelly Bean 4.1.2
* \version   Linux 3.4.0
*/

/**
* \file ad7146.c
* This file is the core driver part of AD7146 with Event interface.
 It also has routines for interrupt handling for
* Sensor active and Convertion complete interrupt modes,
* suspend, resume, initialization routines etc.
* AD7146 Controller Driver
* Copyright 2014 Analog Devices Inc.
* Licensed under the GPL version 2 or later.
*/

#include <linux/input/ad7146.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/sched.h>
#include <linux/switch.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <linux/workqueue.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#include <linux/suspend.h>
#include <linux/wakelock.h>
#endif
#include <linux/wakelock.h>

#define AD7146_DEBUG
#define CONFIG_AD7146_DEBUG

#ifdef CONFIG_AD7146_DEBUG
#define AD7146_Driver_Dbg(format, arg...)	pr_info("[AD7146] :"\
						       format , ## arg)
#else
#define AD7146_Driver_Dbg(format, arg...)	if (0)
#endif

#ifdef CONFIG_DEVICE_LIST
/**
  LIST for the created devices by the driver
  */
static struct list_head ad7146_device_list = LIST_HEAD_INIT(ad7146_device_list);
EXPORT_SYMBOL(ad7146_device_list);
#endif

typedef int (*ad7146_read_t)(struct device *, unsigned short, \
			     unsigned short *, unsigned short);
typedef int (*ad7146_write_t)(struct device *, unsigned short, unsigned short);

static struct wake_lock cap_prox_wake_lock;

/*
 REGISTER BASED DEFINES associated with the driver
 */

/**
  Power control Register
 */
#define PWR_CTRL_REG			(0x0)
/**
  Calibration and Control Register
 */
#define STG_CAL_EN_REG			(0x1)
/**
  Device Control Register 0
 */
#define AMB_COMP_CTRL0_REG		(0x2)
/**
Device ID Register Address
*/
#define AD7146_PARTID_REG		(0x17)
/**
Stage Configuration Address
*/
#define AD7146_STAGECFG_REG		(0x80)
/**
System config Register Address
*/
#define AD7146_SYSCFG_REG		(0x0)
/**
lower Threshold Enable register
*/
#define STG_LOW_INT_EN_REG		(0x5)
/**
Higher Threshold Enable register
*/
#define STG_HIGH_INT_EN_REG		(0x6)
/**
Conversion complete Interrupt Enable register
*/
#define STG_COM_INT_EN_REG		(0x7)
/**
lower Threshold Status register
*/
#define STG_LOW_INT_STA_REG		(0x8)
/**
Higher Threshold Status register
*/
#define STG_HIGH_INT_STA_REG	(0x9)
/**
Conversion complete Interrupt Status register
*/
#define STG_COM_INT_STA_REG		(0xA)
/**
Register address of Stage 1 CDC result
*/
#define CDC_RESULT_S0_REG		(0xB)
/**
Register address of MOD_FREQ_CTL_REG register
*/
#define MOD_FREQ_CTL_REG		(0x45)
/**
  Highest write accessible register
 */
#define HIGHER_WR_REG			(0x7f)
/**
  lowest write accessible register
 */
#define LOWER_WR_REG			(0x8)
/**
  STAGE0 Connection register 1
*/
#define STAGE0_CONNECTION_6_0_REG	(0x0080)
/**
  STAGE0 Connection register 2
*/
#define STAGE0_CONNECTION_12_7_REG	(0x0081)
/**
  STAGE1 Connection register 1
*/
#define STAGE1_CONNECTION_6_0_REG	(0x0088)
/**
  STAGE1 Connection register 2
*/
#define STAGE1_CONNECTION_12_7_REG	(0x0089)
/**
  STAGE2 Connection register 1
*/
#define STAGE2_CONNECTION_6_0_REG	(0x0090)
/**
  STAGE2 Connection register 2
*/
#define STAGE2_CONNECTION_12_7_REG	(0x0091)
/**
  STAGE3 Connection register 1
*/
#define STAGE3_CONNECTION_6_0_REG	(0x0098)
/**
  STAGE3 Connection register 2
*/
#define STAGE3_CONNECTION_12_7_REG	(0x0099)
/**
  STAGE4 Connection register 1
*/
#define STAGE4_CONNECTION_6_0_REG	(0x00A0)
/**
  STAGE4 Connection register 2
*/
#define STAGE4_CONNECTION_12_7_REG	(0x00A1)
/**
  STAGE5 Connection register 1
*/
#define STAGE5_CONNECTION_6_0_REG	(0x00A8)
/**
  STAGE5 Connection register 2
*/
#define STAGE5_CONNECTION_12_7_REG	(0x00A9)
/**
  STAGE6 Connection register 1
*/
#define STAGE6_CONNECTION_6_0_REG	(0x00B0)
/**
  STAGE6 Connection register 2
*/
#define STAGE6_CONNECTION_12_7_REG	(0x00B1)
/**
  STAGE7 Connection register 1
*/
#define STAGE7_CONNECTION_6_0_REG	(0x00B8)
/**
  STAGE7 Connection register 2
*/
#define STAGE7_CONNECTION_12_7_REG	(0x00B9)
/**
  STAGE8 Connection register 1
*/
#define STAGE8_CONNECTION_6_0_REG	(0x00C0)
/**
  STAGE8 Connection register 2
*/
#define STAGE8_CONNECTION_12_7_REG	(0x00C1)
/**
  STAGE9 Connection register 1
*/
#define STAGE9_CONNECTION_6_0_REG	(0x00C8)
/**
  STAGE9 Connection register 2
*/
#define STAGE9_CONNECTION_12_7_REG	(0x00C9)
/**
  STAGE10 Connection register 1
*/
#define STAGE10_CONNECTION_6_0_REG	(0x00D0)
/**
  STAGE10 Connection register 2
*/
#define STAGE10_CONNECTION_12_7_REG	(0x00D1)
/**
  STAGE11 Connection register 1
*/
#define STAGE11_CONNECTION_6_0_REG	(0x00D8)
/**
  STAGE11 Connection register 2
 */
#define	STAGE11_CONNECTION_12_7_REG	(0x00D9)
/**
  Register address of Stage 0 sf ambient value
*/
#define STG0_SF_AMBIENT_REG		(0xF1)
/**
  Register address of Stage 0 High Threshold value
 */
#define STG0_HIGH_THRESHOLD_REG		(0xFA)
/*
  GENERAL DEFINES used in the drivers
 */
/**
 This is the path for the DAC calibration data file storage
 The File name prefix and the location of the file can be changed
 as per the platform used.
 */
#define OFFSET_FILE_PATH "/data/misc/ad7146_dac_data"
/**
Driver name of this ad7146 driver
*/
#define DRIVER_NAME "ad7146"
/**
 The configuration File name and location in the System.
 This will be used for the Initialization by hardware_initialization
 and also in Run time by "ad7146_filp_config" Sysfs attribute.
 */
#define AD7146_CONFIG_FILE "/data/misc/ad7146cfg.cfg"
/**
This hold the product ID of AD7146
*/
#define AD7146_PRODUCT_ID (0x7146)
/**
  Device ID
 */
#define AD7146_PARTID	(0x1490)
/**
 Bit mask of logic 1
*/
#define AD7146_BIT_MASK(s) (1 << (s))
/**
The Stage count of AD7146 chip.
*/
#define STAGE_NUM             (12)
/**
AD7146 wake up mask used in power management
*/
#define AD7146_WAKEUP_MASK (0xFFFE)
/**
AD7146 shutdown mask used in power management
*/
#define AD7146_SHUTDOWN_MASK (0x01)

#define AD7146_MODE_MASK (~0x3)
#define AD7146_MODE_LOWPOWER 0x2
/**
AD7146 remove Force calibration mask
*/
#define ANTI_FORCE_CAL_MASK (0xBFFF)
/**
Mask For the Force calibration
*/
#define AD7146_FORCED_CAL_MASK  AD7146_BIT_MASK(14)
/**
The maximum amount of data to be stored in the FILP data Buffer.
*/
#define MAX_FILP_DATA_CNT (128)
/**
The buffer size of the filp initialization buffer.
MAX_FILP_DATA_CNT Integers to be stored
*/
#define FILP_BUFF_SIZE ((MAX_FILP_DATA_CNT)*(sizeof(unsigned int)))
/**
The read line buffer size in the configuration
*/
#define VFS_READ_SIZE (64)
/**
The read permission for the ad7146cfg.cfg file open
*/
#define READ_OGW (0666)
/**
General Disable macro for AD7146
*/
#define DISABLE_AD7146			(0)
/**
General Enable macro for AD7146
*/
#define ENABLE_AD7146			(1)
/**
 Zero value
 */
#define ZERO_VAL			(0)
/**
 Conv_complete data written status
*/
#define WRITTEN_CONV			(1)
/**
Register read size
*/
#define REG_READ_SIZE	(100)
/**
Retry attempt on no data in line in the Filp reader
*/
#define RETRY_ATTEMPT_READ (20)
/**
Skip count for the Reading of the Data upon address access
*/
#define SKIP_DATA_CNT	(6)
/**
Minimum number of characters to Read in filp
*/
#define MIN_CHAR_TO_READ (8)
/**
Minimum force calibration sleep in the driver
*/
#define MIN_FORCED_CAL_SLEEP (20)
/**
Maximum force calibration sleep in the driver
*/
#define MAX_FORCED_CAL_SLEEP (50)
/**
Hexa-decimal number base value
*/
#define HEX_BASE (16)
/**
Decimal base value
 */
#define DECIMAL_BASE (10)
/**
 * I2C transfer length for a single read operation
 */
#define I2C_READ_LEN (2)
/**
 * I2C transfer length for a single write operation
 */
#define I2C_W_LEN (4)
/**
 * I2C read/write operation buffer size max
 */
#define MAX_I2C_R_W_LEN (12)
/**
  I2C transfer message length for a single transaction write
 */
#define I2C_W_MSG_LEN			(1)
/**
  I2C Transfer message Length for a single transaction read
 */
#define I2C_R_MSG_LEN			(2)
/**
  Default I2C read length
 */
#define R_LEN				(1)
/**
  GET_AFE_REG to get the current stages AFE register address
 */
#define GET_AFE_REG(cur_stg) ((u16_StageConnRegister[((cur_stg)*2)])+2)
/**
  GET_POS_AFE to get the Positive AFE from the OFFSET register
 */
#define GET_POS_AFE(cur_afe) (((cur_afe) & (0x3f00))>>8)
/**
  GET_NEG_AFE to get the Negative AFE from the OFFSET register
 */
#define GET_NEG_AFE(cur_afe) ((cur_afe) & (0x3f))
/**
  SET_POS_AFE Used to Set the Positive AFE OFFSET.
 */
#define SET_POS_AFE(cur_afe, pos_afe) (((cur_afe) & 0xC0FF) | ((pos_afe)<<8))
/**
  SET_NEG_AFE Used to set the Negative AFE OFFSET.
 */
#define SET_NEG_AFE(cur_afe, neg_afe) (((cur_afe) & (0xFFC0)) | (neg_afe))
/**
 Sleep time calculation for the low power to full power mode switch
 */
#define POWER_SLEEP_TIME(x) ((((x & 0x0C) >> 2) + 1) * 200)
/**
  Default MID value of the DAC calibration
 */
#define DEFAULT_DAC_MID_VALUE		(0x7d00)
/**
  This is the difference value from the MID VALUE
*/
#define DAC_DIFF_VAL			(0xA00)
/**
  OPEN AIR CDC HIGH VALUE
  Used in DAC Calibration Function
 */
#define OPEN_AIR_HIGH_VALUE		(DEFAULT_DAC_MID_VALUE + DAC_DIFF_VAL)
/**
  OPEN AIR CDC LOW VALUE
  Used in DAC Calibration function
 */
#define OPEN_AIR_LOW_VALUE		(DEFAULT_DAC_MID_VALUE - DAC_DIFF_VAL)
/**
  MINIMUM OPEN AIR CDC MID VALUE
  Used in DAC Calibration Function
 */
#define MIN_DAC_MID_VAL			(0x2000)
/**
  MAXIMUM OPEN AIR CDC MID VALUE
  Used in DAC Calibration function
 */
#define MAX_DAC_MID_VAL			(0xA000)
/**
FT_CALIB_DELAY: delay used to read the CDC after setting AFE_OFFSET
 */
#define FT_CALIB_DELAY			(10)
/**
MIN_FT_CALIB_DELAY: delay used CDC update
 */
#define MIN_FT_CALIB_DELAY		(5)
/**
  DECI_MAX maximum decimation factor
 */
#define DECI_MAX			(0x3)
/**
DECIMATION_MASK used to read the decimation factor
 */
#define DECIMATION_MASK (0x300)
/**
  Magic key to identify the configuration file
*/
#define MAGIC_KEY_AD7146 "ADI*"
/**
  CDC zero value for ad7146 chip
  */
#define CDC_ZERO_VALUE (0x0)
/**
  FULL_SCALE_VALUE for ad7146 chip
  */
#define FULL_SCALE_VALUE (0xFFFF)
/**
  FULL_POWER_MASK for ad7146 chip
  */
#define FULL_POWER_MASK (0xFFFC)
/**
  FACTORY calibration routine time out in ms
  */
#define FT_CALIB_T_OUT (8000)
/**
  This is the default mod frequency value for AD7146.
  */
#define DEFAULT_MOD_FREQ_VALUE (0xD00)
/**
  This is the maximum allowed AFE value in the AD7146.
  */
#define MAX_AFE_VALUE (0x3F)
/**
  This is the maximum register address allowed.
  */
#define MAX_REG_COUNT (0x27F)
/**
  This is the POSITIVE SWAP mask for the AD7146.
  */
#define POS_SWAP_MASK (0x8000)
/**
  This is the NEGATIVE SWAP mask for the AD7146.
  */
#define NEG_SWAP_MASK (0x080)
/**
  This is the minimum AFE step in calibration of AD7146.
  */
#define MIN_AFE_STEP (0x06)
/**
Data size in the AFE storage array.
*/
#define CAL_ROW_SIZE (0x2)
/**
  This is the parameter level of Address in the cmd_parser
 */
#define PARSE_ADDR			(0)
/**
FILE NAME BUFFER size
*/
#define FILE_NAME_BUFFER_SIZE (30)
/**
  This is the parameter level of count in the cmd_parser
 */
#define PARSE_CNT			(1)
/**
  This is the parameter level of DATA in the cmd_parser
 */
#define PARSE_DATA			(2)
/**
  This is the maximum parameters to parse in the cmd_parser
 */
#define MAX_PARSE_CNT			(2)
/**
  This is the Separating Character for the parsing
 */
#define SPACE_CHAR			" "
/**
  This is the minus character for the parsing
 */
#define MINUS_CHAR			'-'
/**
  This is the minus value for the cmd parsing
 */
#define MINUS_VAL			(-1)
/**
  This is default activation value used for the SYSFS
 */
#define ACT_SYSFS			(1)
/**
  This is the positive mask to check the full power mode
 */
#define POS_LOW_PWR_MASK		(0x02)
/**
  Maximum i2c read length
*/
#define MAX_SYS_READ_LEN		(12)
/**
  Maximum i2c write length
*/
#define MAX_SYS_WRITE_LEN		(1)
/**
  This is size of a byte
 */
#define BYTE_SIZE			(8)
/**
  This is the shift used for the lower order STG connection for STAGEx_CONNECTION
  register
 */
#define LOW_O_STAGE_SFT			(14)
/**
  This is the bit mask used for the lower order STG connection in getStageInfo
 */
#define STG_CON_MASK			(0x3fff)
/**
  This is the shift used for the lower order STG connection for STAGEx_CONNECTION
  register
 */
#define STG_CONN_CHK_CNT		(14)
/**
  This is total number of STAGEx_CONNECTION registers for a individual stage
 */
#define STG_CONN_CNT			(2)
/**
  Used to skip the unused bits in the enable register
 */
#define SKIP_UNUSED_EN			(0xF000)
/**
  Used to get only the enabled register in the
 */
#define INT_EN_MASK			(0x0FFF)
/**
  Used to mask GPIO_INT_ENABLE in STAGE_COMPLETE_INT_ENABLE Register
 */
#define GPIO_INT_EN_MASK		(0x1000)
/**
  To identify the STAGE CONNECTION to positive input.
*/
#define POS_STG_CON_MASK		(0x2)
/**
  Value of lower half byte
*/
#define LOW_NIBBLE			(0x0F)
/**
  Value of upper half byte
*/
#define HIGH_NIBBLE			(0xF0)
/**
  For single line Comment the Status value is
*/
#define SING_COMMT			(0x1)
/**
  For multi line Comment the Status value is
*/
#define MULT_COMMT			(0x10)
/**
  Status for line Check in filp_config for start of line
*/
#define LINE_CHK_1			(0x1)
/**
  Status for line Check in filp_config for end of line
*/
#define LINE_CHK_2			(0x2)
/**
  Inline data in a line for the filp data file
*/
#define INLINE_DAT			(0x3)
/**
  Sequence stage number mask
*/
#define SEQ_STG_MASK			(0xFF0F)
/**
  Shift bits to place last LAST STAGE
*/
#define LAST_STG_SHIFT			(4)
/**
  Register Address mask
 */
#define REG_ADDR_MASK			(0xFFFF0000)
/**
  Register value mask
 */
#define REG_VAL_MASK			(0x0000FFFF)
/**
  Proximity touch event MASK
 */
#define PROX_TCH_MASK			(0x81)
/**
  Proximity release event MASK
 */
#define PROX_REL_MASK			(0x7F)
/**
  Sensor active mode event mask
 */
#define SENSOR_ACTIVE_MODE		(0x1)
/**
  Hardware detect mask
 */
#define HW_DET_MASK			(0xFFF0)
/**
  Addition factor for variables starting from Zero
 */
#define ADD_FACTOR			(0x1)
/**
 Event shift used for the data packing
 */
#define DATA_SHT			(1)
/**
 Stage 0 value
 */
#define STG_ZERO			(0)
/**
 Stage 1 value
 */
#define	STG_ONE				(1)

#ifdef CONFIG_STG_HYS
#pragma message("Hysteresis Compensation routines included !")

/**
 Default hysteresis percentage for driver
 */
#define HYS_PERCENT                     (25)
/**
 MIN_HYS_VALUE minimal hysteresis value
 */
#define MIN_HYS_VALUE			(5)
/**
 MAX_HYS_VALUE maximum hysteresis value
 */
#define MAX_HYS_VALUE			(90)

#if (HYS_PERCENT > MAX_HYS_VALUE)
#error "Hysteris percentage invalid"
#endif
/**
  Hysteresis compensation macro
 */
#define HYS(S, T, HYS_P)		((T) - ((((T)-(S)) * HYS_P)/100))
/**
  Positive hysteresis compensation macro
 */
#define HYS_POS(S, T, HYS_P)			((T) + ((((T)-(S)) * HYS_P) \
					/ (100 - HYS_P)))
/**
Stage jump count
*/
#define STG_JUMP_CNT			(0x24)
/**
  GET_AMB_REG to get the current stages Ambient register address
 */
#define GET_AMB_REG(x)	(((x) * STG_JUMP_CNT) + STG0_SF_AMBIENT_REG)
/**
  GET_HT_TH_REG to get the current stages HIGH threshold reg address
 */
#define GET_HT_TH_REG(x)	(((x) * STG_JUMP_CNT) + STG0_HIGH_THRESHOLD_REG)

#endif

/**
  This holds the Stage connection register addresses for the 12 stages
  */
const unsigned short u16_StageConnRegister[STAGE_NUM*STG_CONN_CNT] = {
	STAGE0_CONNECTION_6_0_REG,
	STAGE0_CONNECTION_12_7_REG,
	STAGE1_CONNECTION_6_0_REG,
	STAGE1_CONNECTION_12_7_REG,
	STAGE2_CONNECTION_6_0_REG,
	STAGE2_CONNECTION_12_7_REG,
	STAGE3_CONNECTION_6_0_REG,
	STAGE3_CONNECTION_12_7_REG,
	STAGE4_CONNECTION_6_0_REG,
	STAGE4_CONNECTION_12_7_REG,
	STAGE5_CONNECTION_6_0_REG,
	STAGE5_CONNECTION_12_7_REG,
	STAGE6_CONNECTION_6_0_REG,
	STAGE6_CONNECTION_12_7_REG,
	STAGE7_CONNECTION_6_0_REG,
	STAGE7_CONNECTION_12_7_REG,
	STAGE8_CONNECTION_6_0_REG,
	STAGE8_CONNECTION_12_7_REG,
	STAGE9_CONNECTION_6_0_REG,
	STAGE9_CONNECTION_12_7_REG,
	STAGE10_CONNECTION_6_0_REG,
	STAGE10_CONNECTION_12_7_REG,
	STAGE11_CONNECTION_6_0_REG,
	STAGE11_CONNECTION_12_7_REG,
};

/**
  REV C evaluation configuration
 */

#if 0
static unsigned int local_platform_data[] = {
	/* Used for 1_4_1 Driver validation*/
	0x0080FFFB,     0x00811FFF,     0x00820F00,     0x00831414,
	0x00841040,     0x00851240,     0x00860032,     0x00870032,
	0x0088FFFF,     0x00891BFF,     0x008A0F00,     0x008B1414,
	0x008C1040,     0x008D1240,     0x008E0032,     0x008F0032,
	0x00450D01,     0x0000C010,     0x000203CB,     0x0003A008,
	0x000423FF,     0x00050003,     0x00060003,     0x00070000,
	0x0000C01E,     0x00010003,
};

#endif


static unsigned int local_platform_data[] = {
                /* Used for 1_4_1 Driver validation*/
                0x00803BFF,     0x00813FFF,     0x00821100,     0x00830000,
                0x00840700,     0x00850800,     0x00860820,     0x00870710,
                0x00450D01,     0x0000C010,     0x000203CB,     0x0003A008,
                0x000423FF,     0x00050001,     0x00060001,     0x00070000,
                0x0000C00C,     0x00010001,
};

/**
  DAC status values
*/
enum dac_status {
	INIT_DAC,
	PENDING,
	DONE_SUCCESS,
	DONE_FAILED
};

/**
  Driver information which will be used to maintain the software flow
*/
enum ad7146_device_state {
	IDLE,
	ACTIVE
};
/**
This structure holds the driver data of AD7146 for the High threshold.
*/
struct ad7146_driver_data {
	enum ad7146_device_state state;
	unsigned char index;
};
/**
* This structure provides chip information of AD7146.
* \note Contains chip information of the AD7146 chip with attributes
* like Product, Status ,version,drive data etc.
* which are used in the control or to read the status of the device
*/
struct ad7146_chip {
/**
	This is used to hold the current read STAGE_HIGH_INT_STATUS
*/
	unsigned short high_status;
/**
	This is used to hold the current read STAGE_LOW_INT_STATUS
*/
	unsigned short low_status;
/**
	This is used to hold the current read STAGE_COMPLETE_INT_STATUS
*/
	unsigned short complete_status;
/**
	This value hold the configured value for the
	High/Low interrupt enable register.
*/
	unsigned short sensor_int_enable;
/**
	This value hold the configured value complete enable register.
*/
	unsigned short complete_enable;
/**
	This Holds the power register value for suspend/resume functions.
*/
	unsigned short power_reg_value;
/**
	This holds the number of register
	that are connected to the CDC positive input.
*/
	unsigned short no_of_stages;
/**
	This holds the number of register
	that are enabled as STAGE_COMPLETE_INT_ENABLE.
*/
	unsigned short no_of_conv_stages;
/**
	This holds the number of register
	that are enabled as STAGE_HIGH_INT_ENABLEi.
*/
	unsigned short no_of_sens_stages;
/**
	This holds the bit masked version of the connected  register
	that are enabled as STAGE_HIGH_INT_ENABLEi.
*/
	unsigned short stg_calib_enable;
/**
	This holds the bit masked version of the connected  register
*/
	unsigned short stg_connection;
/**
	Used to store the previous COMPLETE_INT_EN register.
*/
	unsigned short previous_conv_en_value;
/**
	used for the identification of the enable register modification through
	the AD7146_reg_write attribute
*/
	unsigned short conv_enable_written;
/**
	This holds the Event data enable for the Sending of event to the
	application.
*/
	unsigned short send_event;
/**
	This holds the driver enable State
*/
	unsigned short driver_enable_state;
/**
	This is used to store the temporary low base value for DAC
	calibration function
*/
	unsigned short open_air_low;
/**
	This is used to store the temporary high base value for DAC
	calibration function
*/
	unsigned short open_air_high;
/**
	This hold the value of the last stage connected
	used in the power control register updation.
*/
	unsigned char last_stg_en;

/**
	This hold the index values of the configured stages
	in STAGE_COMPLETE_INT_ENABLE register.
*/
	unsigned char conv_comp_index[STAGE_NUM];
/**
	This holds the platform data for the AD7146 driver.
*/
	struct ad7146_platform_data *hw;
/**
	This holds the Driver data for the threshold sensing.
*/
	struct ad7146_driver_data *sw;
/**
	IRQ assigned for the chip.
*/
	int irq;
/**
	this is used for the storage of the register read through sysfs.
*/
	char reg_data[REG_READ_SIZE];
/**
	Device stucture of the AD7146.
*/
	struct device *dev;
/**
	Registered input device structure for the device
*/
	struct input_dev *input;
/**
	Driver work strucuture
*/
	//struct work_struct work;
	struct delayed_work dwork;
/**
	Driver DAC calibration work strucuture
*/
	struct work_struct calib_work;
/**
	Function pointer for the Read funtion
*/
	ad7146_read_t read;
/**
	Function pointer for the Write funtion
*/
	ad7146_write_t write;
/**
	Used to store the factory calibrated offset.
*/
	unsigned short afe_offset;
/**
	current DAC completion value
*/
	unsigned short curr_dac_status;
/**
  Filp buffer pointer used in the filp configuration
*/
	int *filp_buffer;
/**
  Filp buffer count of the data.
*/
	unsigned short filp_buff_cnt;
#ifdef CONFIG_STG_HYS
/**
  Hysteresis percentage for AD7146
*/
	unsigned short hys_percent;
#endif
#ifdef CONFIG_DEVICE_LIST
/**
  LIST Head for AD7146
*/
	struct list_head  list_head;
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct  early_suspend    early_suspend;
#endif
/**
	Mutex for use with the device;
*/
	struct mutex mutex;
/**
	This holds the product details.
*/
	unsigned product;
/**
	This Hold the hardware version.
*/
	unsigned version;
/**
 This holds the private data of the driver.
 */
	void *private_data;
};

static unsigned short ad7146_en = ENABLE_AD7146;
module_param(ad7146_en, ushort, S_IRUGO | S_IWUSR);

/* SYSFS Attribute declaration & Initialization */
static ssize_t show_cap_status(struct device *dev,
	struct device_attribute *attr, char *buf);
static DEVICE_ATTR(ad7146_cap_status,  S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP, show_cap_status, NULL);
/*--------------------------------------------------------------*/
static ssize_t show_ic_status(struct device *dev,
	struct device_attribute *attr, char *buf);
static DEVICE_ATTR(ad7146_prox_status, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP, show_ic_status, NULL);
/*--------------------------------------------------------------*/
static ssize_t store_reg_read(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
static ssize_t show_reg_read(struct device *dev,
	struct device_attribute *attr, char *buf);
static DEVICE_ATTR(ad7146_reg_read, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP, show_reg_read, store_reg_read);
/*--------------------------------------------------------------*/
static ssize_t store_reg_write(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
static  DEVICE_ATTR(ad7146_reg_write, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP, NULL, store_reg_write);
/*--------------------------------------------------------------*/
static ssize_t redo_filp_calib(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
static DEVICE_ATTR(ad7146_filp_config, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP, NULL, redo_filp_calib);
/*--------------------------------------------------------------*/
#ifdef CONFIG_STG_HYS
static ssize_t store_hys_percent(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
static ssize_t show_hys_percent(struct device *dev,
	struct device_attribute *attr, char *buf);
static DEVICE_ATTR(ad7146_hys_percent, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP, show_hys_percent,
		   store_hys_percent);
#endif
/*--------------------------------------------------------------*/
static ssize_t store_stage_info(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
static ssize_t show_stage_info(struct device *dev,
	struct device_attribute *attr, char *buf);
static DEVICE_ATTR(ad7146_stage_info, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP, show_stage_info,
		   store_stage_info);
/*--------------------------------------------------------------*/
static ssize_t do_calibrate(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
static DEVICE_ATTR(ad7146_calibrate, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP, NULL, do_calibrate);
/*--------------------------------------------------------------*/
static ssize_t device_force_calibrate(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
static DEVICE_ATTR(ad7146_force_calib, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP, NULL, device_force_calibrate);
/*--------------------------------------------------------------*/
static ssize_t show_dev_enable(struct device *dev,
	struct device_attribute *attr, char *buf);
static ssize_t device_enable(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
static DEVICE_ATTR(ad7146_enable, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP, show_dev_enable, device_enable);
/*--------------------------------------------------------------*/
static ssize_t show_send_event(struct device *dev,
	struct device_attribute *attr, char *buf);
static ssize_t send_eventout(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
static DEVICE_ATTR(ad7146_send_eventout, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP, show_send_event,
		   send_eventout);
/*--------------------------------------------------------------*/
static ssize_t do_dac_calibrate(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
static ssize_t show_dac_status(struct device *dev,
	struct device_attribute *attr, char *buf);
static DEVICE_ATTR(ad7146_dac_calibrate, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP, show_dac_status,
		   do_dac_calibrate);
/*--------------------------------------------------------------*/
static ssize_t store_dac_mid_value(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count);
static ssize_t show_dac_mid_value(struct device *dev,
	struct device_attribute *attr, char *buf);
static DEVICE_ATTR(ad7146_dac_mid_val, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP, show_dac_mid_value,
		   store_dac_mid_value);
/*--------------------------------------------------------------*/
static struct attribute *ad7146_sysfs_entries[] = {
	&dev_attr_ad7146_cap_status.attr,
	&dev_attr_ad7146_prox_status.attr,
	&dev_attr_ad7146_reg_read.attr,
	&dev_attr_ad7146_reg_write.attr,
	&dev_attr_ad7146_stage_info.attr,
	&dev_attr_ad7146_calibrate.attr,
	&dev_attr_ad7146_filp_config.attr,
#ifdef CONFIG_STG_HYS
	&dev_attr_ad7146_hys_percent.attr,
#endif
	&dev_attr_ad7146_force_calib.attr,
	&dev_attr_ad7146_enable.attr,
	&dev_attr_ad7146_send_eventout.attr,
	&dev_attr_ad7146_dac_calibrate.attr,
	&dev_attr_ad7146_dac_mid_val.attr,
	NULL
};

static struct attribute_group ad7146_attr_group = {
	.name = NULL,
	.attrs = ad7146_sysfs_entries,
};
/*--------------------------------------------------------------*/

/**
Writes to the Device register through i2C.
Used to Write the data to the I2C client's Register through the i2c protocol

@param data The data to be written
@param reg The register address
@param dev The Device Structure
@return 0 on success

@see ad7146_i2c_read
*/
static int ad7146_i2c_write(struct device *dev, unsigned short reg,
		unsigned short data)
{
	struct i2c_client *client = to_i2c_client(dev);
	char device_addr = client->addr;
	unsigned short tx[I2C_READ_LEN] = {0};
	int ret = 0;
	struct i2c_msg ad7146_wr_msg = {
			.addr = device_addr,
			.buf = (u8 *)tx,
			.len = I2C_W_LEN,
			.flags = 0,
	};
	tx[ZERO_VAL] = cpu_to_be16(reg);
	tx[I2C_W_MSG_LEN] = cpu_to_be16(data);
	ret = i2c_transfer(client->adapter, &ad7146_wr_msg, I2C_W_MSG_LEN);
	if (ret < 0)
			dev_err(&client->dev, "I2C write error\n");
	return ret;
}

/**
Reads data from the Device register through i2C.
This is used to read the data from the AD7146 I2C client

@param dev The Device Structure (Standard linux call)
@param reg The register address to be read.
@param data The data Read from the Given address.
@return The number of bytes transfered as an integer

@see ad7146_i2c_write
*/
static int ad7146_i2c_read(struct device *dev, unsigned short reg,
			unsigned short *data, unsigned short data_cnt)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned short tx = cpu_to_be16(reg);
	unsigned short loop_cnt = 0;
	unsigned short rx[MAX_I2C_R_W_LEN] = {0};
	char device_addr = client->addr;
	int ret = 0;
	struct i2c_msg ad7146_rd_msg[I2C_READ_LEN] = {
			{
				.addr = device_addr,
				.buf = (u8 *)&tx,
				.len = I2C_READ_LEN,
				.flags = 0,
			},
			{
				.addr = device_addr,
				.buf = (u8 *)rx,
				.len = data_cnt * I2C_READ_LEN,
				.flags = I2C_M_RD,
			}
	};
	ret = i2c_transfer(client->adapter, ad7146_rd_msg, I2C_R_MSG_LEN);
	if (unlikely(ret < 0)) {
		dev_err(dev, "I2C READ error %d\n", ret);
		*data = (unsigned short) 0;
	} else {
		for (loop_cnt = 0; loop_cnt < data_cnt; loop_cnt++)
			*(data + loop_cnt) = be16_to_cpu(rx[loop_cnt]);
	}
	return ret;
}

/*Reads cap value from the Device register through i2C.
This is used to read the ic status from the AD7146 I2C client

@param dev The Device Structure (Standard linux call)
@param attr standard Linux Device attributes to the AD7146
@param buf The buffer to store the Read data
@return The number of bytes transfered as an integer

@see ad7146_i2c_write

*/
static ssize_t show_cap_status(struct device *dev,
	struct device_attribute *attr, char *buf){

	unsigned short 	u16Temp;
	struct ad7146_chip *ad7146 = dev_get_drvdata(dev);

	ad7146->read(ad7146->dev, 0x0b, &u16Temp, R_LEN);
		
	return sprintf(buf, "0x%x\n", u16Temp);
}

static ssize_t show_ic_status(struct device *dev,
	struct device_attribute *attr, char *buf){

	struct ad7146_chip *ad7146 = dev_get_drvdata(dev);

	unsigned short status = ad7146->high_status;
	if (ad7146->low_status)
		status =2;

	return sprintf(buf, "%d\n", status);


}

/**
Command parsing function from echo/cat commands from command prompt.
This function is called when ever the User tries an echo / cat command
to the /../sysfs/<Device> especially during read/write registers.

@return void Returns Nothing
@see store_reg_read
 */
static int cmd_parsing(const char *buf, unsigned short *addr,
		unsigned short *cnt, unsigned short *data)
{
	char **bp = (char **)&buf;
	char *token, minus, parsing_cnt = 0;
	unsigned short val;
	int ret;
	int pos;
	while ((token = strsep(bp, SPACE_CHAR))) {
		pos = 0;
		minus = false;
		if ((char)token[pos] == MINUS_CHAR) {
			minus = true;
			pos++;
		}

		ret = kstrtou16(&token[pos], 0, (unsigned short *)&val);
		if (ret)
			return ret;
		if ((parsing_cnt == 0) & (val > MAX_REG_COUNT))
			return -ERANGE;
		if (minus)
			val *= MINUS_VAL;

		switch (parsing_cnt) {
		case PARSE_ADDR:
			*addr = val;
			break;
		case PARSE_CNT:
			*cnt  = val;
			break;
		default:
		case PARSE_DATA:
			*data = val;
			data++;
			break;
		}
		parsing_cnt++;
		if (parsing_cnt > MAX_PARSE_CNT)
			return ret;
	}
	return ret;
}

/**
This function is used for the calculation of the checksum for the driver data

@return unsigned_short The sum of the data in the buffer
@see get_cal_data
 */
static unsigned short calc_checksum(unsigned short *storage_buff,
				    unsigned short size_of_buffer)
{
	unsigned short	u16_Sum = 0;
	int	u32_index;
	for (u32_index = 0; u32_index < (size_of_buffer*2); u32_index++)
		u16_Sum = u16_Sum + storage_buff[u32_index];
	return u16_Sum;
}

/**
This function is used for retreving DAC calibration OFFSETS from a file.
The FILE PATH can be given by OFFSET_FILE_PATH followed by the AD7146 chip
I2C address for which the calibration data is generated.

\note The file format will be similiar to the following.
\note | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |   bytes
\note | A | D | I | * |LENGTH | X | X |
\note | X | X | X | X | X | X | X | X |
\note | . | . | . | . | . | . | . | . |
\note | X | X | X | X | X | X |CHK_SUM|    EOF
here X represents the data, which are paired as
Register address(2-Bytes) followed by (2-Bytes) the value in the register.
@return int Error code on error, and number of bytes read in
@see save_cal_data
 */
static int get_cal_data(struct ad7146_chip *ad7146)
{
	struct file *offset_filp = NULL;
	struct i2c_client *client = to_i2c_client(ad7146->dev);
	char file_path[FILE_NAME_BUFFER_SIZE];
	char read_buff[DECIMAL_BASE];
	short u16_data_count = 0;
	short u16_data_sum;
	unsigned short loop_cnt;
	unsigned short *calib_buff = NULL;
	unsigned short *temp_buff = NULL;
	int err = 0;
	loff_t pos = 0;
	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	sprintf(file_path, OFFSET_FILE_PATH"_%x", client->addr);

	offset_filp = filp_open(file_path, O_RDONLY, 0666);
	if (IS_ERR(offset_filp)) {
		pr_err("%s: FILE OPEN failed\n", __func__);
		set_fs(old_fs);
		return -EIO;
	}

	u16_data_sum = (strlen(MAGIC_KEY_AD7146) +  sizeof(short));
	err = vfs_read(offset_filp, read_buff, u16_data_sum, &pos);
	if ((err == -EIO) || (err != u16_data_sum))
		goto err_file_nodata;

	if (!strncmp(read_buff, MAGIC_KEY_AD7146, strlen(MAGIC_KEY_AD7146))) {
		AD7146_Driver_Dbg("FILE AUTHENTIFICATION SUCCESS");
		temp_buff = (unsigned short *)(read_buff +
					       strlen(MAGIC_KEY_AD7146));
		u16_data_count = (*temp_buff);
		AD7146_Driver_Dbg("%s data cnt %d", __func__, u16_data_count);
		if (u16_data_count > 0) {
			u16_data_sum = (sizeof(short) * CAL_ROW_SIZE *
					u16_data_count) + sizeof(short);
			calib_buff = devm_kzalloc(ad7146->dev, u16_data_sum,
						  GFP_KERNEL);
			if (!(calib_buff)) {
				err = -ENOMEM;
				pr_err("%s: NO MEM\n", __func__);
				goto err_file_nomem;
			}
		} else {
			goto err_file_nodata;
		}
		err = vfs_read(offset_filp, (char *)calib_buff,
			       u16_data_sum, &pos);
		if (err != (u16_data_sum)) {
			pr_err("%s: READ ERROR POS %d\n", __func__, (int)pos);
			goto err_file_nodata;
		}
		temp_buff = (calib_buff + (u16_data_sum/sizeof(short)) - 1);

		AD7146_Driver_Dbg("checksum final %x\n", *temp_buff);
		/*Calculate Checksum*/
		u16_data_sum  = calc_checksum(calib_buff, u16_data_count);

		AD7146_Driver_Dbg("CHECKSUM calculated%x\n", u16_data_sum);

		if (*temp_buff == u16_data_sum) {
			pr_info("checksumPASS %x", (unsigned short)*read_buff);
			for (loop_cnt = 0; loop_cnt < (u16_data_count*2);
			     loop_cnt++) {
				AD7146_Driver_Dbg("REGISTER %x ADDRESS %x\n",
						  calib_buff[loop_cnt],
						  calib_buff[loop_cnt+1]);
				ad7146->write(ad7146->dev, calib_buff[loop_cnt]
					      , calib_buff[loop_cnt+1]);
				loop_cnt++;
			}
		} else {
			AD7146_Driver_Dbg("checksumFAILED %x",
					  (unsigned short)*read_buff);
			goto err_file_nodata;
		}
	} else {
		pr_err("%s: NO data/invalid data\n", __func__);
		goto err_file_nodata;
	}

	filp_close(offset_filp, current->files);
	set_fs(old_fs);
	devm_kfree(ad7146->dev, calib_buff);
	return err;
err_file_nodata:
	err = -EIO;
	if (u16_data_count > 0)
		devm_kfree(ad7146->dev, calib_buff);
err_file_nomem:
	pr_err("%s: NO data/invalid data\n", __func__);
	filp_close(offset_filp, current->files);
	set_fs(old_fs);
	return err;
}

/**
This function is used for saving the DAC calibration OFFSETS from a file.
The FILE PATH can be given by OFFSET_FILE_PATH followed by the AD7146 chip
I2C address for which the calibration data is generated.

\note The file format will be similiar to the following.
\note | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |   bytes
\note | A | D | I | * |LENGTH | X | X |
\note | X | X | X | X | X | X | X | X |
\note | . | . | . | . | . | . | . | . |
\note | X | X | X | X | X | X |CHK_SUM|    EOF

here X represents the data, which are paired as
Register address(2-Bytes) followed by (2-Bytes) the value in the register.
@return int Error code on error, and number of bytes written in
@see get_cal_data
*/
static int save_cal_data(struct ad7146_chip *ad7146,
			 unsigned short *storage_buff,
			 unsigned short size_of_buffer)
{
	struct file *offset_filp = NULL;
	struct i2c_client *client = to_i2c_client(ad7146->dev);
	int ret = 0;
	char file_path[FILE_NAME_BUFFER_SIZE];
	unsigned char *pu8_temp_buff = NULL;
	unsigned short crc_code = 0;
	loff_t pos = 0;
	mm_segment_t old_fs;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	sprintf(file_path, OFFSET_FILE_PATH"_%x", client->addr);

	offset_filp = filp_open(file_path, O_CREAT | O_RDWR | O_SYNC, 0666);
	if (IS_ERR(offset_filp)) {
		pr_err("%s: no offset file\n", __func__);
		ret = PTR_ERR(offset_filp);
		if (ret != -ENOENT)
			pr_err("%s: Can't open calibration file\n", __func__);
		set_fs(old_fs);
		return ret;
	}

	memcpy(storage_buff, MAGIC_KEY_AD7146, strlen(MAGIC_KEY_AD7146));

	pu8_temp_buff = ((char *)storage_buff + strlen(MAGIC_KEY_AD7146));

	memcpy(pu8_temp_buff, &size_of_buffer, sizeof(short));

	pu8_temp_buff = pu8_temp_buff + sizeof(short);
	crc_code = calc_checksum((unsigned short *)pu8_temp_buff,
				 size_of_buffer);

	pu8_temp_buff = ((char *)pu8_temp_buff +
			 (sizeof(short) * CAL_ROW_SIZE * size_of_buffer));

	memcpy(pu8_temp_buff, &crc_code, sizeof(short));

	ret = vfs_write(offset_filp, (char *)storage_buff,
			((sizeof(short) * CAL_ROW_SIZE * size_of_buffer) +
			 (sizeof(short)*2) + strlen(MAGIC_KEY_AD7146)), &pos);

	if (ret != ((sizeof(short) * CAL_ROW_SIZE * size_of_buffer) +
		    (sizeof(short)*2) + strlen(MAGIC_KEY_AD7146))) {
		pr_err("%s:  offset file write error %d\n", __func__, ret);
		ret = -EIO;
	}

	filp_close(offset_filp, current->files);
	set_fs(old_fs);
	return ret;
}

/**
This is used for the DAC CAlibration attribute in the sysfs
This will invoke the DAC calibration routine.
\note This is evoked upon an echo request in the /sys/.../<Device> region.

@return The Size of the Read Data, else work pending will be printed.
@param dev The Device Id and Information structure(Linux Standard argument)
@param attr standard Linux Device attributes to the AD7146
@param buf The buffer to store the Read data
@param count The buffer size to be read
*/
static ssize_t do_dac_calibrate(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	unsigned short val = 0;
	struct ad7146_chip *ad7146 = dev_get_drvdata(dev);

	ret = kstrtou16(buf, DECIMAL_BASE, &val);
	if (unlikely(ret || (val != ACT_SYSFS))) {
		pr_err("[AD7146]: %s INVALID CMD", __func__);
		return count;
	}
	if (!work_pending(&ad7146->calib_work))
		schedule_work(&ad7146->calib_work);
	else
		pr_err("[AD7146]: WORK PENDING!\n");
	return count;
}

/**
This is used for the DAC CAlibration attribute in the sysfs
This will indicate the status of the DAC calibration routine.
\note This is evoked upon an cat request in the /sys/.../<Device> region.

@return 0 if the calibration is complete 1 if its pending.
@param dev The Device Id and Information structure(Linux Standard argument)
@param attr standard Linux Device attributes to the AD7146
@param buf The buffer to store the Read data
*/
static ssize_t show_dac_status(struct device *dev,
	struct device_attribute *attr, char *buf) {
	int ret = 0;
	struct ad7146_chip *ad7146 = dev_get_drvdata(dev);

	ret = sprintf(buf, "%d", ad7146->curr_dac_status);

	return ret;
}

/**
 Function used for the calculation of decimation factor
 @param u16_pwr_reg Power register value
 @reurn The decimation of the device.
 */
static inline unsigned short decimation_calc(unsigned short u16_pwr_reg)
{
	unsigned short u16_deci_factor = 0;
	u16_deci_factor = ((u16_pwr_reg & DECIMATION_MASK) >> BYTE_SIZE) +
			   ADD_FACTOR;
	if (u16_deci_factor > DECI_MAX)
		u16_deci_factor  = DECI_MAX;
	return u16_deci_factor;
}

/**
This is the DAC CAlibration routine that is used for the compensation of the
stray capacitance in the AD7146 sensor.
This is a optimized version of calibration that will run the check and change
the AFE values for all the connected stages before waiting for the conversion
to begin.
The conversion time for full power mode will be calculated based on the
POWER_CTRL register and the conversion time will be calculated based on the
DECIMATION. MOD_FREQ_CTL_REG will be changed to default for fast conversions.

the settings will be restored on exit. If all the stages calibration are
SUCCESS then the calibration data is stored for future reuse.

@return void No return values.
@param work The calibration work structure.
@see save_cal_data
*/
static void dac_calibration_work(struct work_struct *work)
{
	struct ad7146_chip *ad7146 = container_of(work,
						  struct ad7146_chip,
						  calib_work);
	unsigned short u16_curr_cdc = ZERO_VAL;
	unsigned short u16_cur_afe = ZERO_VAL;
	unsigned short u16_mod_ctrl = ZERO_VAL;
	unsigned short u16_pos_afe = ZERO_VAL;
	unsigned short u16_neg_afe = ZERO_VAL;
	unsigned short u16_power_reg = ZERO_VAL;
	unsigned short u16_curr_stg = ZERO_VAL;
	unsigned short u16_afe_reg = ZERO_VAL;
	unsigned short u16_low_int_en = ZERO_VAL;
	unsigned short try_neg_swap = ZERO_VAL;
	unsigned short try_pos_swap = ZERO_VAL;
	unsigned short u16_sleeptime = ZERO_VAL;
	unsigned short u16_tot_caldata = ZERO_VAL;
	unsigned short u16_deci_factor = ZERO_VAL;
	unsigned short *pu16_cal_buffer = NULL;
	unsigned short *pu16_temp_buffer = NULL;
	unsigned long u64_start_time = ZERO_VAL;
	unsigned char stage_cal_status[STAGE_NUM] = {[0 ... (STAGE_NUM-1)] = 0};
	unsigned char afe_written[STAGE_NUM] = {[0 ... (STAGE_NUM-1)] = 0};
	unsigned short calib_pending = ad7146->stg_connection;

	mutex_lock(&ad7146->mutex);
	ad7146->read(ad7146->dev, STG_LOW_INT_EN_REG, &u16_low_int_en, R_LEN);
	ad7146->write(ad7146->dev, STG_COM_INT_EN_REG, DISABLE_AD7146);
	ad7146->write(ad7146->dev, STG_HIGH_INT_EN_REG, DISABLE_AD7146);
	ad7146->write(ad7146->dev, STG_LOW_INT_EN_REG, DISABLE_AD7146);

	disable_irq(ad7146->irq);

	ad7146->curr_dac_status = PENDING;

	dev_info(ad7146->dev, "DO NOT HOLD THE DEVICE!!!\n");

	ad7146->read(ad7146->dev, PWR_CTRL_REG, &u16_power_reg, R_LEN);
	/* Change to Full power mode if needed*/
	if ((u16_power_reg & POS_LOW_PWR_MASK) ||
	    (u16_power_reg & AD7146_SHUTDOWN_MASK)) {
		ad7146->write(ad7146->dev, PWR_CTRL_REG,
			      (u16_power_reg & FULL_POWER_MASK));
		u16_sleeptime =  POWER_SLEEP_TIME(u16_power_reg);
		dev_err(ad7146->dev, "changing to full power with delay %d",
			u16_sleeptime);
		msleep(u16_sleeptime);
	}
	ad7146->read(ad7146->dev, MOD_FREQ_CTL_REG, &u16_mod_ctrl, R_LEN);
	/*Change Modulation frequency if needed*/
	if (u16_mod_ctrl & MAX_AFE_VALUE)
		ad7146->write(ad7146->dev, MOD_FREQ_CTL_REG,
			      DEFAULT_MOD_FREQ_VALUE);
	ad7146->read(ad7146->dev, MOD_FREQ_CTL_REG, &u16_sleeptime, R_LEN);

	if (!ad7146->stg_connection) {
		pr_info("NO STAGE connection!!!\n");
		goto er_cal_f;
	}
	/*Read decimation and calculate the sleep time for conversion*/
	u16_deci_factor = decimation_calc(u16_power_reg);

	dev_dbg(ad7146->dev, "DECIMATION factor %d!!!\n", u16_deci_factor);
	u16_sleeptime = ((ad7146->last_stg_en + ADD_FACTOR) * FT_CALIB_DELAY *
			 ((u16_sleeptime & MAX_AFE_VALUE) + ADD_FACTOR)) /
			 u16_deci_factor;
	dev_dbg(ad7146->dev, "Calc CONV Time %dms!\n", u16_sleeptime);
	pu16_cal_buffer = devm_kzalloc(ad7146->dev,
				       (strlen(MAGIC_KEY_AD7146) + sizeof(int)
					+(sizeof(short) *
				       CAL_ROW_SIZE * ad7146->no_of_stages)),
				       GFP_KERNEL);
	if (!pu16_cal_buffer) {
		dev_err(ad7146->dev, "%s: NO MEM\n", __func__);
		goto er_cal_f;
	}
	/*Store the data in the data field by using this temp pointer*/
	pu16_temp_buffer =  (unsigned short *)((char *)pu16_cal_buffer +
					       (strlen(MAGIC_KEY_AD7146) +
					       sizeof(short)));
	u64_start_time = jiffies;

	while ((jiffies_to_msecs(jiffies - u64_start_time) < FT_CALIB_T_OUT) &&
			calib_pending) {
		/*Loop till the enabled stages*/
		for (u16_curr_stg = 0; u16_curr_stg <= ad7146->last_stg_en;
				u16_curr_stg++) {
			dev_info(ad7146->dev, "CURRENT STAGE = %d!!!\n",
				 u16_curr_stg);
			if ((!(ad7146->stg_connection &
			       AD7146_BIT_MASK(u16_curr_stg))) ||
			    stage_cal_status[u16_curr_stg]) {
				/*
				  Stage not connected or
				  calibration already done
				 */
				continue;
			}
			/*Reading AFE OFFSET Register*/
			u16_afe_reg = GET_AFE_REG(u16_curr_stg);
			u16_cur_afe = CDC_ZERO_VALUE;
			ad7146->read(ad7146->dev,
					(CDC_RESULT_S0_REG+u16_curr_stg),
					&u16_curr_cdc, R_LEN);

			if (((u16_curr_cdc == CDC_ZERO_VALUE) ||
			     (u16_curr_cdc == FULL_SCALE_VALUE)) &&
			    !afe_written[u16_curr_stg]) {
				ad7146->write(ad7146->dev, u16_afe_reg,
					      DISABLE_AD7146);
				afe_written[u16_curr_stg] = ENABLE_AD7146;
			} else {
				ad7146->read(ad7146->dev, u16_afe_reg,
					     &u16_cur_afe, R_LEN);
				try_pos_swap = u16_cur_afe & POS_SWAP_MASK;
				try_neg_swap = u16_cur_afe & NEG_SWAP_MASK;
			}

			AD7146_Driver_Dbg("CURSTG %x R %x D %x CDC %x\n",
					  (u16_curr_stg), u16_afe_reg,
					  u16_cur_afe, u16_curr_cdc);

			u16_neg_afe = GET_NEG_AFE(u16_cur_afe);
			u16_pos_afe = GET_POS_AFE(u16_cur_afe);
			/*Saturation Exception flow*/
			if ((u16_curr_cdc == CDC_ZERO_VALUE) ||
			    (u16_curr_cdc == FULL_SCALE_VALUE)) {
				AD7146_Driver_Dbg("PS%d NS%d\n", try_pos_swap,
						  try_neg_swap);
				if ((!try_neg_swap) &&
				    (!try_pos_swap)) {
					/*As both of the afe */
					u16_pos_afe += MIN_AFE_STEP;
					if (u16_pos_afe > MAX_AFE_VALUE) {
						try_neg_swap = ENABLE_AD7146;
						u16_cur_afe |= NEG_SWAP_MASK;
					} else {
					u16_cur_afe = SET_POS_AFE(u16_cur_afe,
								  u16_pos_afe);
					}
				} else if ((try_neg_swap) &&
					   (!try_pos_swap)) {
					u16_neg_afe += MIN_AFE_STEP;
					if (u16_neg_afe > MAX_AFE_VALUE) {
						try_pos_swap = ENABLE_AD7146;
						try_neg_swap = DISABLE_AD7146;
						u16_cur_afe = (u16_cur_afe |
							       POS_SWAP_MASK) &
							      (~NEG_SWAP_MASK);
					} else {
					u16_cur_afe = SET_NEG_AFE(u16_cur_afe,
								  u16_neg_afe);
					}
				} else if ((try_pos_swap) &&
					   (!try_neg_swap)) {
					if (u16_pos_afe > MIN_AFE_STEP) {
						u16_pos_afe -= MIN_AFE_STEP;
					} else if (u16_neg_afe >
							MIN_AFE_STEP) {
						u16_neg_afe -= MIN_AFE_STEP;
					} else {
						goto er_cal_f;
					}
					u16_cur_afe = SET_NEG_AFE(u16_cur_afe,
								  u16_neg_afe);
					u16_cur_afe = SET_POS_AFE(u16_cur_afe,
								  u16_pos_afe);
				} else {
					/*Both of the swap is set*/
					goto er_cal_unkwn_f;
				}
			} else if (u16_curr_cdc > ad7146->open_air_high) {
				if ((u16_pos_afe < MAX_AFE_VALUE) &&
				    (!try_pos_swap)) {
					u16_pos_afe++;
					u16_cur_afe = SET_POS_AFE(u16_cur_afe,
								  u16_pos_afe);
				} else if ((u16_neg_afe > 0) &&
					   (!try_neg_swap)) {
					u16_neg_afe--;
					u16_cur_afe = SET_NEG_AFE(u16_cur_afe,
								  u16_neg_afe);
				} else if ((u16_pos_afe > 0) && try_pos_swap) {
					u16_pos_afe--;
					u16_cur_afe = SET_POS_AFE(u16_cur_afe,
								  u16_pos_afe);
				} else if ((u16_neg_afe < MAX_AFE_VALUE) &&
					   (try_neg_swap)) {
					u16_neg_afe++;
					u16_cur_afe = SET_NEG_AFE(u16_cur_afe,
								  u16_neg_afe);
				} else {
					dev_err(ad7146->dev,
						"CDC can't be reduced\n");
					goto er_cal_f;
				}
			} else if (u16_curr_cdc < ad7146->open_air_low) {
				if ((u16_pos_afe > 0) &&
				    (!try_pos_swap)) {
					u16_pos_afe--;
					u16_cur_afe = SET_POS_AFE(u16_cur_afe,
								  u16_pos_afe);
				} else if ((u16_neg_afe < MAX_AFE_VALUE) &&
					   (!try_neg_swap)) {
						u16_neg_afe++;
					u16_cur_afe = SET_NEG_AFE(u16_cur_afe,
								  u16_neg_afe);
				} else if ((u16_pos_afe < MAX_AFE_VALUE) &&
					   (try_pos_swap)) {
					u16_pos_afe++;
					u16_cur_afe = SET_POS_AFE(u16_cur_afe,
								  u16_pos_afe);
				} else if ((u16_neg_afe > 0) && try_neg_swap) {
					u16_neg_afe--;
					u16_cur_afe = SET_NEG_AFE(u16_cur_afe,
								  u16_neg_afe);
				} else {
					dev_err(ad7146->dev,
						"CDC can't be increased\n");
					goto er_cal_f;
				}
			} else {
				/* CDC is in RANGE*/
				*pu16_temp_buffer = u16_afe_reg;
				pu16_temp_buffer++;
				stage_cal_status[u16_curr_stg] = ENABLE_AD7146;
				*pu16_temp_buffer = u16_cur_afe;
				pu16_temp_buffer++;
				calib_pending &= (unsigned short)
						~AD7146_BIT_MASK(u16_curr_stg);
				u16_tot_caldata++;
				continue;
			}
			/*Update the AFE for the CURRENT STAGE*/
			ad7146->write(ad7146->dev, u16_afe_reg, u16_cur_afe);
		}
		/*Sleep after one complete cycle of AFE changes*/
		msleep(u16_sleeptime);
	}
	dev_info(ad7146->dev, "Factory Calibration Complete !\nStatus:");
	if ((!calib_pending) && (u16_tot_caldata == ad7146->no_of_stages)) {
		save_cal_data(ad7146, pu16_cal_buffer, u16_tot_caldata);
		dev_err(ad7146->dev, "SUCCESS\t TOTAL_STG = %d!!!\n",
			ad7146->no_of_stages);
		ad7146->curr_dac_status = DONE_SUCCESS;
	} else {
		dev_err(ad7146->dev, "FAILED\t TOTAL_STG=%d STAGES_DONE=%d\n",
			ad7146->no_of_stages, u16_tot_caldata);
		ad7146->curr_dac_status = DONE_FAILED;
	}
	ad7146->write(ad7146->dev, MOD_FREQ_CTL_REG, u16_mod_ctrl);
	ad7146->write(ad7146->dev, PWR_CTRL_REG, u16_power_reg);
	dev_info(ad7146->dev, "POWER MODE restored!!!\n");
	/* Do force recalibration of all the stages */
	ad7146->read(ad7146->dev, AMB_COMP_CTRL0_REG, &u16_curr_cdc, R_LEN);
	u16_curr_cdc = (u16_curr_cdc | AD7146_FORCED_CAL_MASK); /*chk*/
	ad7146->write(ad7146->dev, AMB_COMP_CTRL0_REG, u16_curr_cdc);
	msleep(MIN_FORCED_CAL_SLEEP);
	/*Check and write the enable registers */
	enable_irq(ad7146->irq);
	ad7146->write(ad7146->dev, STG_COM_INT_EN_REG,
		      ad7146->complete_enable);
	ad7146->write(ad7146->dev, STG_HIGH_INT_EN_REG,
		      ad7146->sensor_int_enable);
	ad7146->write(ad7146->dev, STG_LOW_INT_EN_REG,
		      u16_low_int_en);
	mutex_unlock(&ad7146->mutex);
	devm_kfree(ad7146->dev, pu16_cal_buffer);
	return;
er_cal_unkwn_f:
	dev_err(ad7146->dev, "UNKNOWN error\n");
er_cal_f:
	enable_irq(ad7146->irq);
	ad7146->write(ad7146->dev, STG_COM_INT_EN_REG,
		      ad7146->complete_enable);
	ad7146->write(ad7146->dev, STG_HIGH_INT_EN_REG,
		      ad7146->sensor_int_enable);
	ad7146->write(ad7146->dev, STG_LOW_INT_EN_REG,
		      u16_low_int_en);
	if (!pu16_cal_buffer)
		devm_kfree(ad7146->dev, pu16_cal_buffer);
	ad7146->write(ad7146->dev, PWR_CTRL_REG, u16_power_reg);
	ad7146->write(ad7146->dev, MOD_FREQ_CTL_REG, u16_mod_ctrl);
	mutex_unlock(&ad7146->mutex);
	dev_err(ad7146->dev, "POWER MODE restored!!!\n");
	dev_err(ad7146->dev, "Factory calibration FAILED\n");
	dev_err(ad7146->dev, "MAXIMUX RETRY REACHED\n");
	return;
}

/**
This is used to get the Stage information of the device using i2c and update
the driver settings and enabled interrupts index accordingly with the loaded
configuration. This function also sets the interrupt modes based on the
connectivity and the provided enable registers.

\note If multiple stages are enabled for the convertion complete mode based
interrupts, The last convertion enabled stage will alone be written,
Thus after a convertion cycle all the enabled registers CDC can be updated,
This routine stores the enabled registers index in the corresponding driver
data [conv_comp_index and ad7146_driver_data] for the driver to use in the ISR.
\note Thus it is important to update the STAGE_COMPLETE_INT_ENABLE register,
in the event of a configuration change in runtime before recalibration using
calibrate.

@param ad7146 The Device structure
@return Void Returns Nothing
*/
static void getStageInfo(struct ad7146_chip *ad7146, unsigned short write_reg)
{
	unsigned short u16_IntrEn = ZERO_VAL;
	unsigned short u16_NumOfStages = ZERO_VAL;
	unsigned short u16_LastStageNum = ZERO_VAL;
	unsigned short u16_Sens_Stages = ZERO_VAL;
	unsigned short u16_Conv_Stages = ZERO_VAL;
	unsigned short u16_Sens_Act = ZERO_VAL;
	unsigned short u16_Cnv_Cmp = ZERO_VAL;
	unsigned short u16Temp1, u16Temp2, u16Cnt;
	unsigned char u8_StageIndex = ZERO_VAL;
	unsigned char curnt_stg = ZERO_VAL;
	unsigned int u32_StgConn;
	u16_IntrEn = ZERO_VAL;
	ad7146->sensor_int_enable = ZERO_VAL;
	ad7146->complete_enable = ZERO_VAL;
	ad7146->last_stg_en = ZERO_VAL;
	u16_NumOfStages = ZERO_VAL;

	ad7146->read(ad7146->dev, STG_HIGH_INT_EN_REG, &u16_Sens_Act, R_LEN);
	if (!ad7146->conv_enable_written) {
		/* If the Conversion enable register is not modified the
		   the previous enabled value could be used*/
		u16_Cnv_Cmp = ad7146->previous_conv_en_value;
	} else {
		/* Read the new enable value for the STG_COM_INT_EN_REG
		   register*/
		ad7146->read(ad7146->dev, STG_COM_INT_EN_REG,
			     &u16_Cnv_Cmp, R_LEN);
		ad7146->previous_conv_en_value = u16_Cnv_Cmp;
	}

	while (u8_StageIndex < (STAGE_NUM * STG_CONN_CNT)) {
		ad7146->read(ad7146->dev, u16_StageConnRegister[u8_StageIndex],
			     &u16Temp1, R_LEN);
		u8_StageIndex++;

		ad7146->read(ad7146->dev, u16_StageConnRegister[u8_StageIndex],
			     &u16Temp2, R_LEN);
		u8_StageIndex++;
		/*u16Temp1 -> Lower order, while u16Temp2 -> Higher order*/
		u32_StgConn = ((u16Temp2 << LOW_O_STAGE_SFT) |
			       (u16Temp1 & STG_CON_MASK));
		curnt_stg = (u8_StageIndex/STG_CONN_CNT) - ADD_FACTOR;
		AD7146_Driver_Dbg("STAGE %d CON_REG = %x\n",
				  curnt_stg, u32_StgConn);
		/*Check the 13 CIN's and the internal Sensor connectivity*/
		for (u16Cnt = 0; u16Cnt < STG_CONN_CHK_CNT; u16Cnt++) {
			if ((u32_StgConn & DECI_MAX) == POS_STG_CON_MASK) {
				if (u16_Sens_Act & AD7146_BIT_MASK(curnt_stg)) {
					ad7146->sw[u16_Sens_Stages].index =
						curnt_stg;
					u16_Sens_Stages++;
				}
				if (u16_Cnv_Cmp & AD7146_BIT_MASK(curnt_stg)) {
					ad7146->conv_comp_index[u16_Conv_Stages]
						= curnt_stg;
					u16_Conv_Stages++;
				}
				u16_IntrEn = (u16_IntrEn |
					      (AD7146_BIT_MASK(curnt_stg)));
				u16_LastStageNum = curnt_stg;
				u16_NumOfStages++;
				break;
			}
			u32_StgConn = (u32_StgConn >> STG_CONN_CNT);
		}
	}
	u16_IntrEn = u16_IntrEn & INT_EN_MASK;
	ad7146->sensor_int_enable = (u16_IntrEn & u16_Sens_Act);
	ad7146->stg_calib_enable =  ad7146->sensor_int_enable;
	ad7146->last_stg_en = u16_LastStageNum;
	ad7146->stg_connection = u16_IntrEn;
	ad7146->no_of_conv_stages = u16_Conv_Stages;
	ad7146->no_of_sens_stages = u16_Sens_Stages;
	ad7146->no_of_stages = u16_NumOfStages;
	ad7146->conv_enable_written = DISABLE_AD7146;

	/*
	 This sets the last connected stage to only
	 Thus the final interrupt will yield all the converted data
	*/
	if (u16_Conv_Stages)
		/*If Non-zero - Contains some CONV stages*/
		ad7146->complete_enable = AD7146_BIT_MASK(
			ad7146->conv_comp_index[(u16_Conv_Stages-ADD_FACTOR)]);
	else
		/*If Zero set to Zero*/
		ad7146->complete_enable = u16_Conv_Stages;
	/*Check and write the enable registers */
	ad7146->complete_enable = (ad7146->complete_enable |
				   (u16_Cnv_Cmp & GPIO_INT_EN_MASK));
	if (write_reg) {
		if (ad7146->complete_enable != u16_Cnv_Cmp)
			ad7146->write(ad7146->dev, STG_COM_INT_EN_REG,
				      ad7146->complete_enable);
		/*Check and write the enable registers */
		if (ad7146->sensor_int_enable != u16_Sens_Act)
			ad7146->write(ad7146->dev, STG_HIGH_INT_EN_REG,
				      ad7146->sensor_int_enable);
		/*Don't Disturb the gpio settings*/
		ad7146->read(ad7146->dev, STG_LOW_INT_EN_REG,
			     &u16_Sens_Act, R_LEN);
		u16_Sens_Act = (u16_Sens_Act & SKIP_UNUSED_EN) |
				ad7146->sensor_int_enable;
		ad7146->write(ad7146->dev, STG_LOW_INT_EN_REG, u16_Sens_Act);
	}

	AD7146_Driver_Dbg("Stage Calibration Enable = %d No_of_Sens_stg = %d ",
			  u16_IntrEn, u16_Sens_Stages);
	AD7146_Driver_Dbg("No_of_Conv_stg = %d last_stg_en = %d\n",
			  u16_Conv_Stages, u16_LastStageNum);

	AD7146_Driver_Dbg("sensor_int_enable  = %d complete_enable = %d\n",
			  ad7146->sensor_int_enable, ad7146->complete_enable);
}

/**
@brief This API is used to start calibration from a user space
file named ad7146cfg.cfg in the /data/misc folder
*	@param filename name of the file
*	@param ad7146 the AD7146 device pointer
*	@return int Neagative if ERROR
*/
static int ad7146_filp_start_calib(char *filename,
		struct ad7146_chip *ad7146)
{
	mm_segment_t old_fs;
	struct file *fp_ad7146 = NULL;
	unsigned short data_cnt = 0;
	unsigned short tot_data_cnt = 0;
	unsigned int Start_Addr = 0;
	unsigned short line_no = 0;
	unsigned short loop_cnt = 0;
	unsigned short line_chk = 0;
	unsigned short cmt_lock = 0;
	unsigned short recv_data = 0;
	int scanf_ret = 0;
	int ret = 0;
	unsigned char Chk_comment = 0;
	char read_buff[VFS_READ_SIZE];
	char retry_cnt = RETRY_ATTEMPT_READ;
	char *temp_buf = NULL;
	char *pos_buf = NULL;
	int *filp_buf = ad7146->filp_buffer;
	loff_t pos = 0;
	int char1, char2;
	unsigned char line_char_count = 0;
	memset(read_buff, '\0', sizeof(read_buff));

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	fp_ad7146 = filp_open(filename, O_RDONLY, READ_OGW);
	if (IS_ERR(fp_ad7146)) {
		printk(KERN_INFO"[AD7146]: Unable to Open the file %ld\n",
		       PTR_ERR(fp_ad7146));
		set_fs(old_fs);
		ret = MINUS_VAL;
		goto err_filp_open;
	}
	do {
		ret = vfs_read(fp_ad7146, read_buff, sizeof(read_buff), &pos);
		if ((ret == -EIO) | (ret <= MIN_CHAR_TO_READ)) {
			ret = ENABLE_AD7146;
			AD7146_Driver_Dbg("Reached EOF\n");
			if (tot_data_cnt == 0) {
				ret = -EIO;
				goto err_filp_nodata;
			} else {
				break;
			}

		} else {
			temp_buf = read_buff;
			temp_buf = strsep((char **)&temp_buf, "\n");
			if (temp_buf == NULL)
				goto err_filp_nodata;
			pos_buf = temp_buf;
			line_char_count = ZERO_VAL;
			line_chk = ZERO_VAL;
			while ((*pos_buf) != '\0') {
				/*traverse and  check for comments*/
				line_char_count += ADD_FACTOR;
				if ((*pos_buf == '/') &&
				    (*(pos_buf + ADD_FACTOR) == '/'))
					Chk_comment = SING_COMMT;
				/*Check Multiple Comment*/
				else if ((*pos_buf == '/') &&
					 (*(pos_buf + ADD_FACTOR) == '*'))
					cmt_lock = ENABLE_AD7146;
				else if ((*pos_buf == '*') &&
					 (*(pos_buf + ADD_FACTOR) == '/'))
					line_chk |= MULT_COMMT;
				/*Check inline Data*/
				else if (*pos_buf == ':')
					line_chk |= LINE_CHK_1;
				/*Check inline Data*/
				else if (*pos_buf == ';')
					line_chk |= LINE_CHK_2;
				if ((!cmt_lock) && (Chk_comment) &&
				    (*pos_buf != '/') && (*pos_buf != ' ') &&
				    (*pos_buf != '\r') && (*pos_buf != '\t'))
					line_chk |= MULT_COMMT;
				pos_buf++;
			}
			pos = pos + line_char_count - (unsigned int) ret + 1;
			line_no++; /*New Line Found*/
			if (Chk_comment || cmt_lock) {
				if ((line_chk & LOW_NIBBLE) == INLINE_DAT) {
					if (!cmt_lock) {
						/*Comment with Inline data*/
						Chk_comment = ZERO_VAL;
						goto read_line;
					}
				} else if (line_chk & HIGH_NIBBLE) {
					/*Reset Comment Status- single line*/
					cmt_lock = ZERO_VAL;
				} else { /*Check double / comment*/
					if (Chk_comment)
						cmt_lock = cmt_lock ^
						   (unsigned short)LINE_CHK_1;
				}
					Chk_comment = ZERO_VAL;
					continue;/*Skip the line parsing*/
			}
read_line:
			if (line_char_count >= MIN_CHAR_TO_READ) {
				/*Get Starting Address*/
				scanf_ret = sscanf(temp_buf, "%x: ",
						   &Start_Addr);
				if (scanf_ret < 1)
					goto err_filp_nodata;
			} else {
				AD7146_Driver_Dbg("No valid data in line %d\n",
						  line_no);
				if (!retry_cnt--)
					goto err_filp_nodata;
				else
					continue;
			}
			temp_buf += SKIP_DATA_CNT;
			data_cnt = ZERO_VAL; /*Reset the Data, to the start*/
			while (loop_cnt <= VFS_READ_SIZE) {
				/*For Parsing a line*/
				scanf_ret = sscanf(temp_buf, "%x%x",
						   (unsigned int *)&char1,
						   (unsigned int *)&char2);
				if (scanf_ret < LINE_CHK_2)
					goto err_filp_nodata;
				recv_data = (unsigned short)(char1 << BYTE_SIZE
							     | char2);
				if (tot_data_cnt >= MAX_FILP_DATA_CNT) {
					AD7146_Driver_Dbg("Filp_buff full\n");
					break;
				}
				*filp_buf = (unsigned int)
						(((unsigned short)
						  (Start_Addr + data_cnt) <<
						  (BYTE_SIZE * I2C_READ_LEN))
						 | recv_data);
				filp_buf++;
				data_cnt++;
				tot_data_cnt++;
				/*Skip the Read Stream*/
				temp_buf += SKIP_DATA_CNT;

				if ((*temp_buf == ';')|(*(temp_buf+1) == ';'))
					break;/*Line parsed*/
			}
		}
	} while ((ret > MIN_CHAR_TO_READ) | (ret != -EIO));

	if (tot_data_cnt == 0)
		goto err_filp_nodata;
	else
		ad7146->filp_buff_cnt = tot_data_cnt;
	filp_close(fp_ad7146, NULL);
	set_fs(old_fs);
	return ret;
err_filp_nodata:
	ret = MINUS_VAL;
	ad7146->filp_buff_cnt = 0;
	printk(KERN_INFO"[AD7146]: File parser failed : %s\n", filename);
	AD7146_Driver_Dbg("No valid data found\n");
	filp_close(fp_ad7146, NULL);
err_filp_open:
	set_fs(old_fs);
	return ret;
}

/**
  This is to configure the device with the register set defined in platform file.
  Finally calibration is done and status registers will be cleared.
 * @param  ad7146 The Device structure
 * @return void  Nothing Returned
 */
static int ad7146_hw_init(struct ad7146_chip *ad7146,
			  unsigned int u32_Of_Reg_Cnt)
{
	int lcnt = 0;
	int ret = MINUS_VAL;
	unsigned short data;
	unsigned short val_mod_freq = DEFAULT_MOD_FREQ_VALUE;
	unsigned int *buffer = NULL;
#ifndef CONFIG_NOSTG_CAL_LAST
	unsigned short val_stg_cal_en = 0;
#endif
	ad7146->no_of_stages = 0;
	ret = ad7146_filp_start_calib(AD7146_CONFIG_FILE, ad7146);
	mutex_lock(&ad7146->mutex);
	for (lcnt = ZERO_VAL; lcnt < STAGE_NUM; lcnt++) {
		unsigned short addr;
		addr = u16_StageConnRegister[lcnt*STG_CONN_CNT];
		/*Default No Connection Setting*/
		ad7146->write(ad7146->dev, addr, FULL_SCALE_VALUE);
		ad7146->write(ad7146->dev, (addr + ADD_FACTOR), STG_CON_MASK);
	}
	/* Platform data can be NULL if not given*/
	/*There is no config file*/
	if (likely(ad7146->hw)) {
		buffer = ad7146->hw->regs;
		data = sizeof(ad7146->hw->regs)/sizeof(int);
	} else {
		buffer = local_platform_data;
		data = sizeof(local_platform_data)/sizeof(int);
	}
	if (ad7146->filp_buff_cnt) {
		buffer = ad7146->filp_buffer;
		data = ad7146->filp_buff_cnt;
		AD7146_Driver_Dbg("FILP CNT %d\n", data);
		u32_Of_Reg_Cnt = 0;
	}
	/** configuration CDC and interrupts */
	for (lcnt = ZERO_VAL; lcnt < data; lcnt++) {
		unsigned short addr;
		unsigned short value;
		if (u32_Of_Reg_Cnt) {
			/*Not from Filp config but from FDT*/
			addr = (unsigned short)((be32_to_cpu(buffer[lcnt]) &
						REG_ADDR_MASK) >> HEX_BASE);
			value = (unsigned short)(be32_to_cpu(buffer[lcnt]) &
					REG_VAL_MASK);
		} else {
			addr = (unsigned short)((((buffer[lcnt])) &
						REG_ADDR_MASK) >> HEX_BASE);
			value = (unsigned short)((buffer[lcnt]) & REG_VAL_MASK);
		}
		/*Force calibration done afterwards*/
		if (addr == AMB_COMP_CTRL0_REG) {
			value = value & ANTI_FORCE_CAL_MASK;
		} else if (addr == MOD_FREQ_CTL_REG){
			val_mod_freq = value;
			value = DEFAULT_MOD_FREQ_VALUE;
		}
#ifndef CONFIG_NOSTG_CAL_LAST
		if (addr == STG_CAL_EN_REG) {
			val_stg_cal_en = value;
			continue;
		}
#endif
		AD7146_Driver_Dbg("Writing 0x%x with Value %x\n",
				  addr, value);
		ad7146->write(ad7146->dev, addr, value);
	}
#ifndef CONFIG_NOSTG_CAL_LAST
	if (val_stg_cal_en) {
		AD7146_Driver_Dbg("Writing 0x01 with Value %x\n",
				  val_stg_cal_en);
		ad7146->write(ad7146->dev, STG_CAL_EN_REG,
			      val_stg_cal_en);
	}
#endif
	data = 0;

	/* Get Number of Stages used and Last stage enabled
	 * Set the interrupts enables too*/
	getStageInfo(ad7146, DISABLE_AD7146);

	/* Identify SEQUENCE_STAGE_NUM and re-write once  */
	ad7146->read(ad7146->dev, PWR_CTRL_REG,
		     &ad7146->power_reg_value, R_LEN);

	ad7146->power_reg_value = ((ad7146->power_reg_value & SEQ_STG_MASK) |
			(ad7146->last_stg_en << LAST_STG_SHIFT));

	/* Calibrate only the stages that have been used */
	ad7146->read(ad7146->dev, STG_CAL_EN_REG, &data, R_LEN);
	data = (data & SKIP_UNUSED_EN) |
	       (ad7146->stg_calib_enable & INT_EN_MASK);
	ad7146->write(ad7146->dev, STG_CAL_EN_REG, data);
	ret = get_cal_data(ad7146);
	if (ret < 0)
		pr_err("WARNING: DAC offset file not found!\n"
				"Using the initial AFE offset\n");
	else 
		ad7146->curr_dac_status = DONE_SUCCESS;

	msleep(MAX_FORCED_CAL_SLEEP);
	ad7146->write(ad7146->dev, MOD_FREQ_CTL_REG, val_mod_freq);
	AD7146_Driver_Dbg("Mod_freq = %x ", val_mod_freq);

	/* Do force recalibration of all the stages */
	ad7146->read(ad7146->dev, AMB_COMP_CTRL0_REG, &data, R_LEN);
	data = (data | AD7146_FORCED_CAL_MASK) ; /*chk*/
	ad7146->write(ad7146->dev, AMB_COMP_CTRL0_REG, data);
	msleep(MAX_FORCED_CAL_SLEEP);

	AD7146_Driver_Dbg("sleeptime = %d ms", MAX_FORCED_CAL_SLEEP);
	/* clear all interrupts status for the new interrupts to flow*/
	ad7146->read(ad7146->dev, STG_LOW_INT_STA_REG, &data, R_LEN);
	ad7146->read(ad7146->dev, STG_HIGH_INT_STA_REG, &data, R_LEN);
	ad7146->read(ad7146->dev, STG_COM_INT_STA_REG, &data, R_LEN);
	if (!ad7146->driver_enable_state) /* It the driver is Disabled */
		ad7146->write(ad7146->dev, PWR_CTRL_REG,
			      (ad7146->power_reg_value |
			      AD7146_SHUTDOWN_MASK));
	else
		ad7146->write(ad7146->dev, PWR_CTRL_REG,
			      ad7146->power_reg_value);

	dev_info(ad7146->dev, "%s done", __func__);
	mutex_unlock(&ad7146->mutex);
	if (ad7146->no_of_stages) {
		return 0;
	} else {
		dev_err(ad7146->dev, "%s NO STAGE Connections", __func__);
		return -ENODEV;
	}
}

/**
This is used to redo a hardware init upon the request from the user.
This function calibrates the entire chip with the new configuration provided
or redos the initial platform configuration.
\note This called when the user requires to redo the configuration
\note This is evoked upon an echo request in the /sys/.../<Device> file.

@return The Size of the parsed command
@param dev The Device Id and Information structure
@param attr The Device attributes to the AD7146
@param buf The buffer to store the Read data
@param count The count of bytes to be transfered to the Buffer

@return Returns the Size of the Read value
*/
static ssize_t redo_filp_calib(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	unsigned long val = 0;
	struct ad7146_chip *ad7146 = dev_get_drvdata(dev);

	ret = kstrtoul(buf, DECIMAL_BASE, &val);
	if ((ret) || (val != 1)) {
		printk(KERN_INFO"[AD7146]: %s INVALID CMD", __func__);
		return count;
	}
	ad7146->conv_enable_written = 1;
	ret = ad7146_hw_init(ad7146, 0);
	return count;
}

/**
  This is used to get register address whose data is to be read and size of data to be read

  This function Reads the value at the Device's Register for the i2c client
  \note This called when the user requires to read the configuration
\note This is evoked upon an echo request in the /sys/.../<Device> region.
\note it hold the register address to be read.

@return The Size of the Read Register 0 if not read
@param dev The Device Id and Information structure
@param attr The Device attributes to the AD7146
@param buf The buffer to store the Read data
@param count The count of bytes to be transfered to the Buffer

@return Returns the Size of the Read value
@see cmd_parsing
*/
static ssize_t store_reg_read(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned short addr, cnt, val;
	unsigned short rd_data[MAX_I2C_R_W_LEN], lp_cnt;
	struct ad7146_chip *ad7146 = dev_get_drvdata(dev);

	mutex_lock(&ad7146->mutex);

	ret = cmd_parsing(buf, &addr, &cnt, &val);
	if (ret == -ERANGE) {
		pr_err("Values not in RANGE\n");
		goto error;
	} else if (ret == -EINVAL) {
		pr_err("Invalid COMMAND/ARGUMENT\n");
		goto error;
	} else {
		val = 0;
	}

	if (cnt > sizeof(rd_data))
		goto error;

	memset(ad7146->reg_data, 0, sizeof(ad7146->reg_data));

	if (cnt > MAX_SYS_READ_LEN) {
		AD7146_Driver_Dbg(
		"I2C Multi Read support MAX reached %s\n", __func__);
		goto error;
	} else {
		ad7146->read(ad7146->dev, addr, rd_data, cnt);
	}

	for (lp_cnt = 0; lp_cnt < cnt; lp_cnt++) {
		val += sprintf(&ad7146->reg_data[val], "0x%04X ",
			       rd_data[lp_cnt]);
		dev_info(ad7146->dev, "Reg Read cmd:reg 0x%04x Data 0x%04x\n",
			 addr + lp_cnt, rd_data[lp_cnt]);
	}

error:
	mutex_unlock(&ad7146->mutex);
	return count;
}

/**
This is used to read the data of the register address sent to reg_read

This functions Reads the value at the Device's Register for the given
client and Prints it in the output window
\note This is evoked upon an cat request in the /sys/.../<Device> region.

@return The Size of the Read Data, 0 if not read
@param dev The Device Id and Information structure(Linux Standard argument)
@param attr standard Linux Device attributes to the AD7146
@param buf The buffer to store the Read data

@see dev_get_drvdata
@see store_reg_read
*/
static ssize_t show_reg_read(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ad7146_chip  *ad7146 = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n", ad7146->reg_data);
}

/**
This is used to write data to a register through i2c.

This functions Writes the value of the buffer to the given client
provided the count value to write
\note This is used to store the register address to write the data.
\note This is evoked upon an echo request in the /sys/.../<Device> region.
\note This also prints the Command received before writing the Registers.

@param dev The Device Id and Information structure(Linux Standard argument)
@param attr standard Linux Device attributes to the AD7146
@param count The number of bytes to write from the buffer
@param buf The buffer to store the Read data

@return The Size of the writen Data, 0 if not writen
*/
static ssize_t store_reg_write(struct device *dev,
	 struct device_attribute *attr, const char *buf, size_t count)
{
	struct ad7146_chip *ad7146 = dev_get_drvdata(dev);
	unsigned short addr, cnt;
	unsigned short wr_data[I2C_READ_LEN], loop_cnt;
	int ret;

	mutex_lock(&ad7146->mutex);

	ret = cmd_parsing(buf, &addr, &cnt, &wr_data[0]);
	if (ret == -ERANGE) {
		pr_err("Values not in RANGE\n");
		goto error;
	} else if (ret == -EINVAL) {
		pr_err("Invalid Commands ARGUMENTS\n");
		goto error;
	}
	if (cnt > sizeof(wr_data))
		goto error;

	AD7146_Driver_Dbg("Register Write command : reg = 0x%x, size = %d\n",
			  addr, cnt);
	for (loop_cnt = 0; loop_cnt < cnt; loop_cnt++)
		AD7146_Driver_Dbg("DATA = 0x%04X\n", wr_data[loop_cnt]);
	if (cnt > MAX_SYS_WRITE_LEN) {
		AD7146_Driver_Dbg(
	"I2C Multi Write not supported in function %s\n", __func__);
	} else {
		if (addr == STG_COM_INT_EN_REG)
			ad7146->conv_enable_written = WRITTEN_CONV;
		/*Added ->  To make the driver stay in the Shutdown*/
		if (ad7146->driver_enable_state == DISABLE_AD7146) {
			if (addr == PWR_CTRL_REG) {
				ad7146->power_reg_value = wr_data[0] &
							  AD7146_WAKEUP_MASK;
				wr_data[0] = wr_data[0] |
					     AD7146_SHUTDOWN_MASK;
			}
		}

		if ((addr < LOWER_WR_REG) || (addr > HIGHER_WR_REG) ||
		    (addr == MOD_FREQ_CTL_REG))
			ad7146->write(ad7146->dev, addr, wr_data[0]);
		else
			pr_err("[AD7146]:READ only ,WRITE protected\n");
	}

error:
	mutex_unlock(&ad7146->mutex);

	return count;
}

/**
This functions stores the DAC MID scale value for the DAC calibration.
This is to be set if the Default DAC MID scale of 0x2710 is not required.
\note This is evoked upon an write request in the /sys/.../<Device> region.

@param dev The Device Id and Information structure(Linux Standard argument)
@param attr standard Linux Device attributes to the AD7146
@param buf The buffer to store the Read data
@param count The count of bytes to be transfered to the Buffer
@return The Size of the Read Data, 0 if not Read
@see getStageInfo
*/
static ssize_t store_dac_mid_value(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct ad7146_chip *ad7146 = dev_get_drvdata(dev);
	int err;
	unsigned short val;
	err = kstrtou16(buf, 0, &val);
	if (err)
		return err;
	mutex_lock(&ad7146->mutex);/*Chip mutex*/

	if ((val >= MIN_DAC_MID_VAL) && (val <= MAX_DAC_MID_VAL)) {
		ad7146->open_air_low = val - DAC_DIFF_VAL;
		ad7146->open_air_high = val + DAC_DIFF_VAL;
		ad7146->curr_dac_status = INIT_DAC;
	}

	mutex_unlock(&ad7146->mutex);/*chip Mutex*/
	return count;
}

/**
This function is to display the DAC threshold values
@param dev The Device Id and Information structure(Linux Standard argument)
@param attr standard Linux Device attributes to the AD7146
@param buf The buffer to store the Read data
\note This is evoked upon an read in the /sys/.../<Device> file.

@return The Size of the Read Data, 0 if not Read
@see getStageInfo
 */
static ssize_t show_dac_mid_value(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ad7146_chip  *ad7146 = dev_get_drvdata(dev);
	int s32_mid_val = (ad7146->open_air_low + ad7146->open_air_high) /
			   STG_CONN_CNT;
	return sprintf(buf, "%d %d %d\n", ad7146->open_air_low, s32_mid_val,
		       ad7146->open_air_high);
}

/**
This is used to identify and store the Stages information of the device.
This is required to configure the interrupt modes
based on the stages enabled during runtime.

This functions Reads the value of the Stage status,
and stores it in the buffer given.
\note This is evoked upon an read request in the /sys/.../<Device> region.

@param dev The Device Id and Information structure(Linux Standard argument)
@param attr standard Linux Device attributes to the AD7146
@param buf The buffer to store the Read data
@param count The count of bytes to be transfered to the Buffer
@return The Size of the Read Data, 0 if not Read
@see getStageInfo
*/
static ssize_t store_stage_info(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct ad7146_chip *ad7146 = dev_get_drvdata(dev);
	int err;
	unsigned long val;
	err = kstrtoul(buf, DECIMAL_BASE, &val);
	if (err)
		return err;
	mutex_lock(&ad7146->mutex);

	if (val)
		getStageInfo(ad7146, ENABLE_AD7146);

	mutex_unlock(&ad7146->mutex);/*Global Mutex*/
	return count;
}

/**
This function is to show the stages info to the user.
The output would be like [COUNT] [stageX] [stageY] .... [stageZ]

@param dev The Device Id and Information structure(Linux Standard argument)
@param attr standard Linux Device attributes to the AD7146
@param buf The buffer to store the Read data
\note This is evoked upon an echo[write] in the /sys/.../<Device> file.

@return The Size of the Read Data, 0 if not Read
@see getStageInfo
 */
static ssize_t show_stage_info(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ad7146_chip  *ad7146 = dev_get_drvdata(dev);
	unsigned short temp;
	unsigned int shift_count = 0;
	unsigned char stage[STAGE_NUM];
	unsigned int count;
	int loop_cnt, loop_cnt2 = 0;

	temp = ad7146->stg_connection;
	count = 0;
	memset(stage, 0, sizeof(stage));
	while (temp) {
		if ((temp & ENABLE_AD7146) == ENABLE_AD7146)
			stage[count++] = shift_count;
		shift_count++;
		temp >>= ADD_FACTOR;
	}
	loop_cnt2 = sprintf(buf, "%d ", count);
	loop_cnt = 0;
	while (loop_cnt < count)
		loop_cnt2 += sprintf(buf+loop_cnt2, "%d ", stage[loop_cnt++]);
	loop_cnt2 += sprintf(buf+loop_cnt2, "\n");

	return loop_cnt2;
}

/**
This is used to Calibrate the driver and also the device through i2c,
also this functions force calibrates the AD7146 chip after driver data updates.
This attribute should be called to update the driver for changes in the
configuration on runtime. It is stronly recommended to update the entire
configuration [Including StageConnects, Enable registers, Thresholds]
rather than only some registers unless the user is aware of the outcome.

@param dev The Device Id and Information structure(Linux Standard argument)
@param attr standard Linux Device attributes to the AD7146
@param buf The buffer to store the data to be written
@param count The count of bytes to be transfered to the Device.
\note This call should be used only if there is a complete configuration change
\note configurations could be written using AD7146_reg_write attribute
 also note that the driver overwrites the STAGE_COMPLETE_INT_ENABLE with the
last enable value, but stores the enabled registers index for reading the
convertion data after a convertion cycle.
\note Thus STAGE_COMPLETE_INT_ENABLE should be written before
use of this attribute, failing to do so will cause the driver to
report only the last enabled CDC stage of the previos configuration.

\note This is evoked upon an echo[write] request in the /sys/.../<Device> file.
\note This also prints the results in the console for the user.
\note The calibration is invoked by this function forcefully upon user request.
\note Effects are immediate in this calibration.

@return count of data written
*/
static ssize_t do_calibrate(struct device *dev,
struct device_attribute *attr, const char *buf, size_t count)
{
	struct ad7146_chip *ad7146 = dev_get_drvdata(dev);
	int err;
	unsigned long val;
	unsigned short u16Temp;
	err = kstrtoul(buf, DECIMAL_BASE, &val);
	if (err)
		return err;
	mutex_lock(&ad7146->mutex);

	getStageInfo(ad7146, ENABLE_AD7146);
	/* Identify SEQUENCE_STAGE_NUM and re-write once  */
	ad7146->read(ad7146->dev, PWR_CTRL_REG, &u16Temp, R_LEN);
	u16Temp = ((u16Temp & SEQ_STG_MASK) |
			(ad7146->last_stg_en << LAST_STG_SHIFT)); /*chk*/
	ad7146->write(ad7146->dev, PWR_CTRL_REG, u16Temp);
	if (ad7146->driver_enable_state == DISABLE_AD7146)
		u16Temp	= u16Temp & AD7146_WAKEUP_MASK;
	ad7146->power_reg_value = u16Temp; /*for power management*/
	u16Temp = 0; /*Reset u16Temp for reuse*/
	ad7146->read(ad7146->dev, STG_CAL_EN_REG, &u16Temp, R_LEN);
	u16Temp = u16Temp | (ad7146->sensor_int_enable & INT_EN_MASK);
	ad7146->write(ad7146->dev, STG_CAL_EN_REG, u16Temp);
	u16Temp = 0; /*Reset u16Temp for reuse*/
	ad7146->read(ad7146->dev, AMB_COMP_CTRL0_REG, &u16Temp, R_LEN);
	/*Set the 14th bit For Force calibrate*/
	u16Temp = u16Temp | AD7146_FORCED_CAL_MASK;
	ad7146->write(ad7146->dev, AMB_COMP_CTRL0_REG, u16Temp);
	msleep(MAX_FORCED_CAL_SLEEP);
	ad7146->read(ad7146->dev, STG_LOW_INT_STA_REG, &u16Temp, R_LEN);
	ad7146->read(ad7146->dev, STG_HIGH_INT_STA_REG, &u16Temp, R_LEN);
	ad7146->read(ad7146->dev, STG_COM_INT_STA_REG, &u16Temp, R_LEN);
	AD7146_Driver_Dbg("Force calibration done\n");
	mutex_unlock(&ad7146->mutex);

	return count;
}

#ifdef CONFIG_STG_HYS
/**
This function is used to change the hysteresis percentage of the device.

@param dev The Device Id and Information structure(Linux Standard argument)
@param attr standard Linux Device attributes to the AD7146
@param buf The buffer to store the data to be written
@param count The count of bytes to be transfered to the Device

\note This is evoked upon an echo[write] request in the /sys/.../<Device> file.
\note This also prints the results in the console for the user.
@see show_hys_percent
@return count of data handled
*/
static ssize_t store_hys_percent(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct ad7146_chip *ad7146 = dev_get_drvdata(dev);
	int err;
	unsigned long val;
	err = kstrtoul(buf, DECIMAL_BASE, &val);
	if (err)
		return err;
	mutex_lock(&ad7146->mutex);
	if ((val >= MIN_HYS_VALUE) && (val <= MAX_HYS_VALUE))
		ad7146->hys_percent = val;
	else
		dev_warn(ad7146->dev, "Hystersis Value off limits");

	mutex_unlock(&ad7146->mutex);
	return count;
}

/**
This displays the current hysteresis percentage used by the device.

\note This is evoked upon an cat request in the /sys/.../<Device> region.

@return The Size of the data buffer to be read, 0 if not written.
@param dev The Device Id and Information structure(Linux Standard argument)
@param attr standard Linux Device attributes to the AD7146
@param buf The buffer to store the Read data
@see store_hys_percent
*/
static ssize_t show_hys_percent(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ad7146_chip  *ad7146 = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", ad7146->hys_percent);
}
#endif

/**
This function is used to Enable / Disable the events sent out to the userspace.

@param dev The Device Id and Information structure(Linux Standard argument)
@param attr standard Linux Device attributes to the AD7146
@param buf The buffer to store the data to be written
@param count The count of bytes to be transfered to the Device

\note This is evoked upon an echo[write] request in the /sys/.../<Device> file.
\note This also prints the results in the console for the user.

@return count of data handled
*/
static ssize_t send_eventout(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct ad7146_chip *ad7146 = dev_get_drvdata(dev);
	int err;
	unsigned long val;
	err = kstrtoul(buf, DECIMAL_BASE, &val);
	if (err)
		return err;
	mutex_lock(&ad7146->mutex);
	if ((val == ENABLE_AD7146) || (val == DISABLE_AD7146))
		ad7146->send_event = val;

	mutex_unlock(&ad7146->mutex);
	return count;
}

/**
This is used to display the current event enable state.

\note This is evoked upon an cat request in the /sys/.../<Device> region.

@return The Size of the Read Data, 0 if not read
@param dev The Device Id and Information structure(Linux Standard argument)
@param attr standard Linux Device attributes to the AD7146
@param buf The buffer to store the Read data

@see dev_get_drvdata
@see store_reg_read
*/
static ssize_t show_send_event(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ad7146_chip  *ad7146 = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", ad7146->send_event);
}

/**
This functions force calibrates the Device AD7146 at run time.
This does not check for any configurational changes,
the driver states are not changed in this calibration,
only the base CDC value is updated in the device with the current CDC value.

@param dev The Device Id and Information structure(Linux Standard argument)
@param attr standard Linux Device attributes to the AD7146
@param buf The buffer to store the data to be written
@param count The count of bytes to be transfered to the Device

\note This is evoked upon an echo[write] request in the /sys/.../<Device> file.
\note This also prints the results in the console for the user.
\note The calibration is invoked by this function forcefully upon user request.
\note Effects are immediate in this calibration.

@return count of data written
*/
static ssize_t device_force_calibrate(struct device *dev,
struct device_attribute *attr, const char *buf, size_t count)
{
	struct ad7146_chip *ad7146 = dev_get_drvdata(dev);
	int err ;
	unsigned long val;
	unsigned short u16Temp;
	err = kstrtoul(buf, DECIMAL_BASE, &val);
	if (err)
		return err;
	if (val != ACT_SYSFS)
		return count;
	mutex_lock(&ad7146->mutex);

	ad7146->read(ad7146->dev, AMB_COMP_CTRL0_REG, &u16Temp, R_LEN);
	/*Set the 14th bit For Force calibrate*/
	u16Temp = u16Temp | AD7146_FORCED_CAL_MASK;
	ad7146->write(ad7146->dev, AMB_COMP_CTRL0_REG, u16Temp);
	msleep(MAX_FORCED_CAL_SLEEP);
	ad7146->read(ad7146->dev, STG_LOW_INT_STA_REG, &u16Temp, R_LEN);
	ad7146->read(ad7146->dev, STG_HIGH_INT_STA_REG, &u16Temp, R_LEN);
	ad7146->read(ad7146->dev, STG_COM_INT_STA_REG, &u16Temp, R_LEN);
	AD7146_Driver_Dbg("Force write register done\n");

	mutex_unlock(&ad7146->mutex);
	return count;
}
static int ad7146_enable_for_resume(struct ad7146_chip *ad7146)
{
	unsigned short data;
	
	dev_info(ad7146->dev, "%s enter\n", __func__);
	mutex_lock(&ad7146->mutex);
	if (ad7146->driver_enable_state == ENABLE_AD7146) {
        dev_info(ad7146->dev,
                "%s setting to non-shutdown mode: PWR_CTRL_REG=0x%x\n",
                __func__, ad7146->power_reg_value);
		ad7146->write(ad7146->dev, PWR_CTRL_REG,
				ad7146->power_reg_value);
		/* Make sure the interrupt output line is not low after resume,
		 * otherwise we will get no interrupt again again
		 */
		ad7146->read(ad7146->dev, STG_LOW_INT_STA_REG, &data, R_LEN);
		ad7146->read(ad7146->dev, STG_HIGH_INT_STA_REG, &data, R_LEN);
		ad7146->read(ad7146->dev, STG_COM_INT_STA_REG, &data, R_LEN);
	}

	mutex_unlock(&ad7146->mutex);
	return 0;
}

static int ad7146_disable_for_suspend(struct ad7146_chip *ad7146)
{
	unsigned short data;

	dev_info(ad7146->dev, "%s enter\n", __func__);
	mutex_lock(&ad7146->mutex);
	if (ad7146->driver_enable_state == ENABLE_AD7146) {
		ad7146->read(ad7146->dev, PWR_CTRL_REG, &data, R_LEN);
		ad7146->power_reg_value = data & AD7146_WAKEUP_MASK;
        data = (data & AD7146_MODE_MASK) | AD7146_MODE_LOWPOWER;
        dev_info(ad7146->dev,
                "%s setting to low power mode: PWR_CTRL_REG=0x%x\n",
                __func__, data);
		ad7146->write(ad7146->dev, PWR_CTRL_REG, data);
	}
	mutex_unlock(&ad7146->mutex);

	return 0;
}

/**
 Common enable routine for the AD7146 driver.
 used for the recovery from shutdown to normal mode
 @param ad7146 the Chip to be enabled
 @return Returns 0 on success.
 */
static int ad7146_enable(struct ad7146_chip *ad7146)
{
	unsigned short data;
	
	dev_info(ad7146->dev, "%s enter\n", __func__);
	mutex_lock(&ad7146->mutex);
	/* resume to non-shutdown mode */
	if (ad7146->driver_enable_state == ENABLE_AD7146) {
        enable_irq_wake(ad7146->irq);
        dev_info(ad7146->dev,
                "%s setting to non-shutdown mode: PWR_CTRL_REG=0x%x\n",
                __func__, ad7146->power_reg_value);
		ad7146->write(ad7146->dev, PWR_CTRL_REG,
				ad7146->power_reg_value);
		/* Make sure the interrupt output line is not low after resume,
		 * otherwise we will get no interrupt again again
		 */
		ad7146->read(ad7146->dev, STG_LOW_INT_STA_REG, &data, R_LEN);
		ad7146->read(ad7146->dev, STG_HIGH_INT_STA_REG, &data, R_LEN);
		ad7146->read(ad7146->dev, STG_COM_INT_STA_REG, &data, R_LEN);
	}

	mutex_unlock(&ad7146->mutex);
	return 0;
}

/**
 Common disable routine for the AD7146 driver.
 used for the shutdown of the chip
 @param ad7146 the Chip to be Disabled
 @return Returns 0 on success.
 */
static int ad7146_disable(struct ad7146_chip *ad7146)
{
	unsigned short data;

	dev_dbg(ad7146->dev, "%s enter\n", __func__);
	mutex_lock(&ad7146->mutex);
	if (ad7146->driver_enable_state == ENABLE_AD7146) {
		ad7146->read(ad7146->dev, PWR_CTRL_REG, &data, R_LEN);
		ad7146->power_reg_value = data & AD7146_WAKEUP_MASK;
		data |= AD7146_SHUTDOWN_MASK;
		ad7146->write(ad7146->dev, PWR_CTRL_REG, data);
        disable_irq_wake(ad7146->irq);
	}
	mutex_unlock(&ad7146->mutex);

	return 0;
}

/**
This functions is used to enable or disable the device in runtime.

@param dev The Device Id and Information structure(Linux Standard argument)
@param attr standard Linux Device attributes to the AD7146
@param buf The buffer to store the data to be written
@param count The count of bytes to be transfered to the Device

\note This is evoked upon an echo[write] request in the /sys/.../<Device> file.
\note This also prints the results in the console for the user.
\note Effects are immediate in this enable and disable.
\note No interrupts will occur when disabled, also the device will be shutdown.

@return count of data written
*/
static ssize_t device_enable(struct device *dev,
struct device_attribute *attr, const char *buf, size_t count)
{
	struct ad7146_chip *ad7146 = dev_get_drvdata(dev);
	int err;
	unsigned long val;
	
	err = kstrtoul(buf, DECIMAL_BASE, &val);
	if (err)
		return err;
	if (val == ENABLE_AD7146) {
		ad7146->driver_enable_state = ENABLE_AD7146;
		ad7146_enable(ad7146);
		dev_info(ad7146->dev, "AD7146 Enabled\n");
	} else if (val == DISABLE_AD7146) {
		ad7146_disable(ad7146);
		ad7146->driver_enable_state = DISABLE_AD7146;
		dev_info(ad7146->dev, "AD7146 Disabled\n");
	} else {
		dev_info(ad7146->dev, "Enable - 1 Disable - 0\n");
	}

	return count;
}

/**
This is used to display the current enable state of the device

\note This is evoked upon an cat request in the /sys/.../<Device> region.

@return The Size of the Read Data, 0 if not read
@param dev The Device Id and Information structure(Linux Standard argument)
@param attr standard Linux Device attributes to the AD7146
@param buf The buffer to store the Read data

@see dev_get_drvdata
@see store_reg_read
*/
static ssize_t show_dev_enable(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ad7146_chip  *ad7146 = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", ad7146->driver_enable_state);
}

#ifdef CONFIG_STG_HYS
/**
  This function is used check the stage & accordingly calculate the
  hysteresis compensation required.

  @param ad7146 The AD7146 chip structure pointer
  @param index The AD7146 chip structure index of the stage
  @return void - Nothing returned
 */
static void ad7146_hys_comp_neg(struct ad7146_chip *ad7146,
					unsigned short index)
{
	unsigned short u16_high_threshold = 0;
	unsigned short u16_sf_ambient = 0;
	unsigned int result = 0;

	ad7146->read(ad7146->dev, GET_HT_TH_REG(index),
			&u16_high_threshold, R_LEN);
	ad7146->read(ad7146->dev, GET_AMB_REG(index),
			&u16_sf_ambient, R_LEN);
	result = HYS(u16_sf_ambient, u16_high_threshold,
			ad7146->hys_percent);
	ad7146->write(ad7146->dev, GET_HT_TH_REG(index),
			(unsigned short)result);
	AD7146_Driver_Dbg("N STG0 HT 0x%x->0x%x\n",
			  u16_high_threshold, result);
}

/**
  This function is used check the stage & accordingly calculate the
  hysteresis compensation required.

  @param ad7146 The AD7146 chip structure pointer
  @param index The AD7146 chip structure index of the stage
  @return void - Nothing returned
*/
static void ad7146_hys_comp_pos(struct ad7146_chip *ad7146,
					unsigned short index)
{
	unsigned short u16_high_threshold = 0;
	unsigned short u16_sf_ambient = 0;
	unsigned int result = 0;

	ad7146->read(ad7146->dev, GET_HT_TH_REG(index),
			&u16_high_threshold, R_LEN);
	ad7146->read(ad7146->dev, GET_AMB_REG(index),
			&u16_sf_ambient, R_LEN);
	result = HYS_POS(u16_sf_ambient, u16_high_threshold,
			 ad7146->hys_percent);
	ad7146->write(ad7146->dev, GET_HT_TH_REG(index),
			(unsigned short)result);
	AD7146_Driver_Dbg("P STG0 HT 0x%x->0x%x\n",
			  u16_high_threshold, result);
}
#endif

#if defined(CONFIG_HAS_EARLYSUSPEND)
/**
  Suspends the Device.
  This is used to Suspend the I2C client from its operation in the system

  @param dev The Device to be suspended
  @return 0 on success

  @see ad7146_i2c_resume
 */
static int ad7146_i2c_suspend(struct early_suspend *h)
{
	struct ad7146_chip *ad7146;
	ad7146 = container_of(h, struct ad7146_chip, early_suspend);
	ad7146_disable(ad7146);

	AD7146_Driver_Dbg("%s finished\n", __func__);
	return 0;
}

/**
  Resumes the Device.
  This is used to Resume the I2C client from its suspention in the system

  @param dev The Device to be Resumed
  @return 0 on success

  @see ad7146_i2c_suspend
 */
static int ad7146_i2c_resume(struct early_suspend *h)
{
	struct ad7146_chip *ad7146;
	ad7146 = container_of(h, struct ad7146_chip, early_suspend);

	ad7146_enable(ad7146);

	return 0;
}
#endif

/**
 * This is to handle the Sensor Active Based on the interrupt mode configured,
 the data will be packed and sent to userspace via event interface.

 @param  ad7146 Device structure for ad7146 chip
 @return void Nothing Returned

*/
static inline void ad7146_sensor_state_machine(struct ad7146_chip *ad7146)
{
	struct ad7146_driver_data *sw = NULL;
	unsigned short loop_cnt;
	unsigned int event_value;

	for (loop_cnt = 0; loop_cnt < ad7146->no_of_sens_stages; loop_cnt++) {
		/* Handle sensor interrupt*/
		sw = &(ad7146->sw[loop_cnt]); /*Loop for Each Stage*/
		if ((ad7146->low_status & AD7146_BIT_MASK(sw->index)) &&
		    (ad7146->sensor_int_enable & AD7146_BIT_MASK(sw->index))) {
			unsigned short data_read_ctl;
			ad7146->read(ad7146->dev, CDC_ZERO_VALUE + sw->index,
				     &data_read_ctl, R_LEN);
			if (data_read_ctl != FULL_SCALE_VALUE ||
			    data_read_ctl != CDC_ZERO_VALUE) {
				/*Error case - Handling Proximty can't
				have a low interrupt->  Do force calibrate*/
				ad7146->read(ad7146->dev, AMB_COMP_CTRL0_REG,
					     &data_read_ctl, R_LEN);
				data_read_ctl = (data_read_ctl |
						 AD7146_FORCED_CAL_MASK);
				ad7146->write(ad7146->dev, AMB_COMP_CTRL0_REG,
					      data_read_ctl);
				msleep(MIN_FORCED_CAL_SLEEP);
				AD7146_Driver_Dbg("Low interrupt occured\n");
			}
			return;
		}
		/* Check if the Stage is SENSOR ACTIVE enabled*/
		if (ad7146->sensor_int_enable & AD7146_BIT_MASK((sw->index))) {
			switch (sw->state) {
			case IDLE:
				/*Sensor went to active*/
				if ((ad7146->high_status &
				     AD7146_BIT_MASK(sw->index))
				     == AD7146_BIT_MASK(sw->index)) {
					AD7146_Driver_Dbg(
					"proximity %d touched\n", sw->index);
					event_value = ((unsigned char)
						       (sw->index << DATA_SHT))
						       | PROX_TCH_MASK;
					if (ad7146->send_event) {
						input_event(ad7146->input,
							    EV_MSC, MSC_RAW,
							    event_value);
						input_sync(ad7146->input);
					}
					sw->state = ACTIVE;
#ifdef CONFIG_STG_HYS
					ad7146_hys_comp_neg(ad7146, sw->index);
#endif
				}
				break;
			case ACTIVE:
				/* Sensor went to inactive*/
				if ((ad7146->high_status &
				     AD7146_BIT_MASK(sw->index))
				     != AD7146_BIT_MASK(sw->index)) {
					AD7146_Driver_Dbg(
					"proximity %d released\n", sw->index);
					event_value = ((unsigned char)
						       (sw->index << DATA_SHT))
						      & PROX_REL_MASK;
					event_value |=  SENSOR_ACTIVE_MODE;
					if (ad7146->send_event) {
						input_event(ad7146->input,
							    EV_MSC, MSC_RAW,
							    event_value);
						input_sync(ad7146->input);
					}
					sw->state = IDLE;
#ifdef CONFIG_STG_HYS
					ad7146_hys_comp_pos(ad7146, sw->index);
#endif
				}
				break;

			default:
				break;
			}
		}
	}
}

/**
This is to handle the conversion completer Based on the interrupt mode configured,
the data will be packed and sent to userspace via event interface.

@param  ad7146 Device structure for ad7146 chip
@return void Nothing Returned

*/
static inline void ad7146_conv_comp_state_machine(struct ad7146_chip *ad7146)
{
	unsigned int event_value = 0;

	/* Handle conversion complete interrupt*/
	unsigned short data;
	int count;
	for (count = 0; count < ad7146->no_of_conv_stages; count++) {
		ad7146->read(ad7146->dev, CDC_RESULT_S0_REG +
			     ad7146->conv_comp_index[count], &data, R_LEN);
		event_value =  ((data << HEX_BASE) |
				(ad7146->conv_comp_index[count] << DATA_SHT));
		if (ad7146->send_event) {
			input_event(ad7146->input, EV_MSC, MSC_RAW,
				    event_value);
			input_sync(ad7146->input);
		}
	}
}

/**
 * \fn static int ad7146_hw_detect(struct ad7146_chip *ad7146)
 * This Routine reads the Device ID to confirm the existance
 * of the Device in the System.

 @param  ad7146 The Device structure
 @return 0 on Successful detection of the device,-ENODEV on err.
 */
static int ad7146_hw_detect(struct ad7146_chip *ad7146)
{
	unsigned short data;

	printk(KERN_ERR"ad7146  start hw detect");
	ad7146->read(ad7146->dev, AD7146_PARTID_REG, &data, R_LEN);
	printk(KERN_ERR"************* part id is 0x%x",data);
	switch (data & HW_DET_MASK) {
	case AD7146_PARTID:
		ad7146->product = AD7146_PRODUCT_ID;
		ad7146->version = data & LOW_NIBBLE;
		dev_info(ad7146->dev, "ad7146  found AD7146 , rev:%d\n",
			 ad7146->version);
		printk(KERN_ERR"AD7146 found AD7146");
		return 0;

	default:
		dev_err(ad7146->dev,
			"ad7146  ad7146 Not Found, read ID is %04x\n", data);
		printk(KERN_ERR"AD7146 Not found AD7146");
		return -ENODEV;
	}
}

/**
  WORK Handler -- starts proximity state machine to detect proximity.
  based on Sensor Active as well as Conversion Complete mode.
  Pack data and send to userspace for all 12 stages.

  @param work The work structure registered.

  @return void nothing returned
 */
static void ad7146_interrupt_thread(struct work_struct *work)
{

	struct ad7146_chip *ad7146 =  container_of((struct delayed_work*)work,
			struct ad7146_chip, dwork);
	unsigned short prv_high_status = ad7146->high_status;

    pr_info("%s enter\n", __func__);

	mutex_lock(&ad7146->mutex);

	ad7146->read(ad7146->dev, STG_COM_INT_STA_REG,
		     &ad7146->complete_status, R_LEN);
	ad7146->read(ad7146->dev, STG_LOW_INT_STA_REG,
		     &ad7146->low_status, R_LEN);
	ad7146->read(ad7146->dev, STG_HIGH_INT_STA_REG,
		     &ad7146->high_status, R_LEN);

	if ((prv_high_status != ad7146->high_status) || ad7146->low_status)
		ad7146_sensor_state_machine(ad7146);
	if (ad7146->complete_status & ad7146->complete_enable)
		ad7146_conv_comp_state_machine(ad7146);

	mutex_unlock(&ad7146->mutex);

    pr_info("%s exit\n", __func__);
}

/**
  IRQ Handler -- services the Interrupt and schedules the work for uploading
  of events to the user space.

  @param handle The data of the AD7146 Device
  @param irq The Interrupt Request queue to be assigned for the device.

  @return IRQ_HANDLED
 */
static irqreturn_t ad7146_isr(int irq, void *handle)
{

	struct ad7146_chip *ad7146 = handle;
    int wake_timeout = 120; /* ms */

    pr_info("%s enter\n", __func__);
	if (!delayed_work_pending(&ad7146->dwork)) {
        pr_info("%s register a work.\n", __func__);
		/*Register a work*/
		schedule_delayed_work(&ad7146->dwork, msecs_to_jiffies(100));
        pr_info("%s register cap_prox_wake_lock, timeout=%d ms.\n", __func__, wake_timeout);
        wake_lock_timeout(&cap_prox_wake_lock, msecs_to_jiffies(wake_timeout));
	} else {
        pr_info("%s read registers.\n", __func__);
	mutex_lock(&ad7146->mutex) ;
		ad7146->read(ad7146->dev, STG_COM_INT_STA_REG,
				&ad7146->complete_status, R_LEN);
		ad7146->read(ad7146->dev, STG_LOW_INT_STA_REG,
				&ad7146->low_status, R_LEN);
		ad7146->read(ad7146->dev, STG_HIGH_INT_STA_REG,
				&ad7146->high_status, R_LEN);
	mutex_unlock(&ad7146->mutex);
	}

	return IRQ_HANDLED;
}

/**
  Device probe function
  All initialization routines are handled here
  ISR registration, Work initialization, Input Event registration,
  Sysfs Attributes creation...etc.,

  @param client The I2C Device client of the AD7146
  @param id the i2c device ID from the Supported device ID list give
  during the addition of driver.

  @return 0(Zero) on Sucess,On failure -ENOMEM, -EINVAL ,etc.,
 */
static int ad7146_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct input_dev *input = NULL;
	struct ad7146_platform_data *pdata = dev->platform_data;
	int of_regs_cnt = 0;
	struct ad7146_chip *ad7146 = NULL;
	int irq = client->irq;
	int error = MINUS_VAL;
	const struct i2c_device_id *ad7146_ID = id;
	struct ad7146_driver_data *driver_data;

	if (client == NULL) {
		AD7146_Driver_Dbg("I2C Client doesn't exist\n");
		error = -EINVAL;
		goto err_out;
	}

	if (pdata == NULL)
		AD7146_Driver_Dbg("Dev Plat Data Not Found, using loc data\n");

	dev_info(dev, "%s called IRQ = %d\n", __func__, irq);

	ad7146 = devm_kzalloc(dev, (sizeof(*ad7146) + (sizeof(*driver_data) *
					STAGE_NUM)) , GFP_KERNEL);

	if (!ad7146) {
		error = -ENOMEM;
		goto err_out;
	}
	ad7146->sw = (struct ad7146_driver_data *)(ad7146 + 1);
	if (!ad7146->sw) {
		error = -ENOMEM;
		goto err_free_chip;
	}
	ad7146->hw = pdata;

	ad7146->read = ad7146_i2c_read;
	ad7146->write = ad7146_i2c_write;
	ad7146->irq = irq;
	ad7146->dev = dev;
	ad7146->filp_buffer = devm_kzalloc(dev, FILP_BUFF_SIZE, GFP_KERNEL);
	if (!(ad7146->filp_buffer)) {
		error = -ENOMEM;
		goto err_free_chip;
	}
#ifdef CONFIG_STG_HYS
	ad7146->hys_percent = HYS_PERCENT;
#endif
	ad7146->open_air_low = OPEN_AIR_LOW_VALUE;
	ad7146->open_air_high = OPEN_AIR_HIGH_VALUE;

	mutex_init(&ad7146->mutex);

	ad7146->curr_dac_status = INIT_DAC;
	ad7146->conv_enable_written = WRITTEN_CONV;
	ad7146->driver_enable_state = ad7146_en;
	/*
		To start sending events after the detection of hardware
		modify the send_event to ENABLE_AD7146.
	*/
	
	ad7146->send_event = ENABLE_AD7146;
	
	/* check if the device is existing by reading device id of AD7146 */
	error = ad7146_hw_detect(ad7146);
	if (error)
		goto err_free_mem;

	/* initialize and request sw/hw resources */
	INIT_DELAYED_WORK(&ad7146->dwork, ad7146_interrupt_thread);
	INIT_WORK(&ad7146->calib_work, dac_calibration_work);
	error =  ad7146_hw_init(ad7146, of_regs_cnt);
	if (error < 0) {
		dev_err(ad7146->dev, "H/W init failed\n");
		printk(KERN_ERR"h/w init failed");
		goto err_free_mem;
	}
	i2c_set_clientdata(client, ad7146);
#ifdef CONFIG_DEVICE_LIST
	INIT_LIST_HEAD(&ad7146->list_head);

	list_add_tail(&ad7146_device_list, &ad7146->list_head);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	ad7146->early_suspend.suspend   = (void *)ad7146_i2c_suspend;
	ad7146->early_suspend.resume    = (void *)ad7146_i2c_resume;
	ad7146->early_suspend.level     = EARLY_SUSPEND_LEVEL_DISABLE_FB-1;
	register_early_suspend(&ad7146->early_suspend);
#endif

	/*
	 * Allocate and register ad7146 input device
	 */
	/* all proximitys & CDC convereter use one input node */

	input = input_allocate_device();
	if (!input) {
		error = -ENOMEM;
		goto err_free_mem;
	}

	__set_bit(EV_MSC, input->evbit);
	__set_bit(MSC_RAW, input->mscbit);

	ad7146->input = input;

	input->id.bustype = BUS_I2C;
	input->id.product = ad7146->product;
	input->id.version = ad7146->version;
	if (ad7146_ID->name != NULL) {
		AD7146_Driver_Dbg("ad7146_ID->name %s", ad7146_ID->name);
		input->name = ad7146_ID->name;
		AD7146_Driver_Dbg("input->name %s", input->name);
	} else {
		input->name = DRIVER_NAME;
	}
	input->dev.parent = dev;

	error = input_register_device(input);
	if (error)
		goto err_free_dev;

	error = sysfs_create_group(&dev->kobj, &ad7146_attr_group);
	if (error)
		goto err_unreg_dev;
	if (irq <= 0) {
		dev_err(dev, "IRQ not configured!\n");
		error = -EINVAL;
		goto err_unreg_dev;
	}

	error = request_threaded_irq(ad7146->irq, NULL, ad7146_isr,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			dev_name(dev), ad7146);
	
	if (error) {
		dev_err(dev, "irq %d busy?\nDriver init Failed", ad7146->irq);
		goto err_free_sysfs;
	}
	/*Clear The interrupts if they have occured*/
	ad7146->read(ad7146->dev, STG_LOW_INT_STA_REG,
			(unsigned short *)&error, R_LEN);
	ad7146->read(ad7146->dev, STG_HIGH_INT_STA_REG,
			(unsigned short *)&error, R_LEN);
	ad7146->read(ad7146->dev, STG_COM_INT_STA_REG,
			(unsigned short *)&error, R_LEN);

    wake_lock_init(&cap_prox_wake_lock, WAKE_LOCK_SUSPEND,
            "cap_prox_wake_lock");

	return 0;

err_free_sysfs:
	sysfs_remove_group(&ad7146->dev->kobj, &ad7146_attr_group);
err_unreg_dev:
	input_unregister_device(input);
err_free_dev:
	input_free_device(input);
err_free_mem:
	devm_kfree(dev, ad7146->filp_buffer);
err_free_chip:
	devm_kfree(dev, ad7146);
err_out:
	dev_err(dev, "Failed to setup ad7146 device\n");
	return -ENODEV;
}

/**
  Used to remove device.
  This function is used to remove AD7146 from the system by unregistering AD7146

  @param ad7146 The Device Structure

  @return void Nothing returned
 */
void ad7146_remove(struct ad7146_chip *ad7146)
{
	struct device *dev = ad7146->dev;
	free_irq(ad7146->irq, ad7146);
	cancel_work_sync((struct work_struct*)&ad7146->dwork);
	cancel_work_sync(&ad7146->calib_work);
	sysfs_remove_group(&ad7146->dev->kobj, &ad7146_attr_group);
	input_unregister_device(ad7146->input);
	devm_kfree(dev, ad7146->filp_buffer);
	ad7146->sw = NULL;
	devm_kfree(dev, ad7146);
    wake_lock_destroy(&cap_prox_wake_lock);
	AD7146_Driver_Dbg("Event Interface Remove Done\n");
}
#ifdef CONFIG_PM
#ifndef CONFIG_HAS_EARLYSUSPEND
int ad7146_i2c_suspend(struct device *dev)
{
	dev_info(dev, "%s\n", __func__);
	return ad7146_disable_for_suspend(i2c_get_clientdata(to_i2c_client(dev)));
}

int ad7146_i2c_resume(struct device *dev)
{
	dev_info(dev, "%s\n", __func__);
	return ad7146_enable_for_resume(i2c_get_clientdata(to_i2c_client(dev)));
}
/**
  Linux device Power manager ops structure
 */
static SIMPLE_DEV_PM_OPS(ad7146_pm, ad7146_i2c_suspend, ad7146_i2c_resume);
#endif
#endif

/**
  Removes the Device.
  This is used to Remove the device or the I2C client from the system

  @param client The Client Id to be removed
  @return 0 on success
 */
static int ad7146_i2c_remove(struct i2c_client *client)
{
	struct ad7146_chip *chip = i2c_get_clientdata(client);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&chip->early_suspend);
#endif
	ad7146_remove(chip);

	return 0;
}

/**
  This is the Device ID table for the supported devices.
 */
static const struct i2c_device_id ad7146_id[] = {
	{ "ad7146_NORM", 0 },
	{ "ad7146_PROX", 1 },
	{ "ad7146", 2 }, {},
};
MODULE_DEVICE_TABLE(i2c, ad7146_id);

/**
  The file Operation Table
 */
struct i2c_driver ad7146_i2c_driver = {
	.driver = {
		.name = DRIVER_NAME,
#ifdef CONFIG_PM
#ifndef CONFIG_HAS_EARLYSUSPEND
		.pm   = &ad7146_pm,
#endif
#endif
	},
	.probe    = ad7146_probe,
	.remove   = ad7146_i2c_remove,
	.id_table = ad7146_id,
};

/**
  This is an init function called during module insertion -- calls inturn i2c driver probe function
 */
static __init int ad7146_i2c_init(void)
{
	return i2c_add_driver(&ad7146_i2c_driver);
}
module_init(ad7146_i2c_init);

/**
  Called during the module removal
 */
static __exit void ad7146_i2c_exit(void)
{
	i2c_del_driver(&ad7146_i2c_driver);
}

module_exit(ad7146_i2c_exit);
MODULE_DESCRIPTION("Analog Devices ad7146 Sensor Driver");
MODULE_AUTHOR("");
MODULE_LICENSE("GPL");
