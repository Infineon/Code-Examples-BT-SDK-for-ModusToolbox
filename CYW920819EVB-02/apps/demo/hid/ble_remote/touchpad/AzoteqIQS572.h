/*
	This file contains the required settings for IQS572, for a specific hardware setup.
	It is provided by Azoteq.
*/

#ifdef SUPPORT_TOUCHPAD

#ifndef AZOTEQ_IQS572_H
#define AZOTEQ_IQS572_H

#define RETRY_FORCE_COMM_WINDOW             10000      // try again in 10 ms on a failure

#define TP_DOTS_PER_PAD       224
#define MAX_X                 (TOTAL_TXS_VAL * TP_DOTS_PER_PAD)
#define MAX_Y                 (TOTAL_RXS_VAL * TP_DOTS_PER_PAD)

#define BOOTLOADER_READ_VER_CMD 0
#define BOOTLOADER_READ_MEM_CMD 1
#define BOOTLOADER_EXECUTE_CMD  2
#define BOOTLOADER_CRC_CMD      3

#define BOOTLOADER_MAX_READ_LEN 64
#define BOOTLOADER_WRITE_LEN 64
#define VALID_ADDR_MASK   0xffff
#define VALID_

/*
The Azoteq IQS572 employs an address-command type structure instead of a memory
map. This means that data bytes cannot be individually addressed, but can be
obtained by configuring a relevant address command on the device to specify
which blocks of data to read or write. Specific data is thus grouped together,
and identified / accessed by means of the address-command relating to the
specific group.
*/

#define AZO_IQS572_PRODUCT_ID {0x00, 0x3A,0x00, 0x00, 0x00, 0x00, 0x67, 0x69, 0x10, 0x01}

// address-command
#define VERSION_INFO_REG             0x00        // Read
#define XY_DATA_REG                  0x01        // Read
#define PROX_STATUS_REG              0x02        // Read
#define TOUCH_STATUS_REG             0x03        // Read
#define COUNT_VALUES_REG             0x04        // Read
#define LONG_TERM_AVERAGE_REGS       0x05        // Read
#define ATI_COMPENSATION_REG         0x06        // Read / Write
#define PORT_CONTROL_REG             0x07        // Write
#define SNAP_STATUS_REG              0x08        // Read

#define CONTROL_SETTINGS_REG         0x10        // Write
#define THRESHOLD_SETTINGS_REG       0x11        // Write
#define ATI_SETTINGS_REG             0x12        // Write
#define FILTER_SETTINGS_REG          0x13        // Write
#define TIMING_SETTINGS_REG          0x14        // Write
#define CHANNEL_SETUP_REG            0x15        // Write
#define HARDWARE_CONFIG_SETTINGS_REG 0x16        // Write
#define ACTIVE_CHANNELS_REG          0x17        // Write
#define DEBOUNCE_SETTINGS_REG        0x18        // Write

#define PM_PROXIMITY_STATUS_REG      0x20        // Read
#define PM_COUNT_VALUES_REG          0x21        // Read
#define PM_LONG_TERM_AVERAGES_REG    0x22        // Read
#define PM_ATI_COMPENSATION_REG      0x23        // Read / Write
#define PM_ATI_SETTINGS_REG          0x24        // Write

#define TRACKPAD_MANAGEMENT_REG      0x40
#define TOUCH_THRESH_MULTIPLIERS_REG 0x41       // thresholds for each individual channel
#define ATI_TARGET_FACTORS_REG       0x42
#define SLIDER_ATIC_REG              0x43

#define VERSION_INFO_NUM_REGS           10

#define AZO_PROXIMITY_THRESHOLD         8
#define AZO_TOUCH_THRESHOLD             14
#define AZO_TOUCH_THRESHOLD_FW          90

// Register sizes
#define PROX_STATUS_REG_SIZE              2*TOTAL_TXS_VAL                   // 0x02
#define TOUCH_STATUS_REG_SIZE             2*TOTAL_TXS_VAL                   // 0x03
#define COUNT_VALUES_REG_SIZE             2*TOTAL_TXS_VAL*TOTAL_RXS_VAL     // 0x04
#define LONG_TERM_AVERAGE_REGS_SIZE       2*TOTAL_TXS_VAL*TOTAL_RXS_VAL     // 0x05
#define ATI_COMPENSATION_REG_SIZE         TOTAL_TXS_VAL*TOTAL_RXS_VAL       // 0x06
#define SNAP_STATUS_REG_SIZE              2*TOTAL_TXS_VAL                   // 0x08
#define CONTROL_SETTINGS_REG_SIZE         2                                 // 0x10
#define THRESHOLD_SETTINGS_REG_SIZE       9                                 // 0x11
#define ATI_SETTINGS_REG_SIZE             6                                 // 0x12
#define FILTER_SETTINGS_REG_SIZE          6                                 // 0x13

#define TIMING_SETTINGS_REG_SIZE          5                                 // 0x14
#define CHANNEL_SETUP_REG_SIZE            7                                 // 0x15
#define HARDWARE_CONFIG_SETTINGS_REG_SIZE 4                                 // 0x16
#define ACTIVE_CHANNELS_REG_SIZE          2*TOTAL_TXS_VAL                   // 0x17
#define DEBOUNCE_SETTINGS_REG_SIZE        2                                 // 0x18
#define PM_PROXIMITY_STATUS_REG_SIZE      1                                 // 0x20
#define PM_COUNT_VALUES_REG_SIZE          2                                 // 0x21
#define PM_LONG_TERM_AVERAGES_REG_SIZE    2                                 // 0x22
#define PM_ATI_COMPENSATION_REG_SIZE      1                                 // 0x23
#define PM_ATI_SETTINGS_REG_SIZE          3                                 // 0x24


/*
   The following are all of the register initializations for the Azoteq IQS572
   chip. They are put into a structure that allows them to be all written to the
   chip all at once.
*/
//------------------------------------------------------------------------------
/* Control Settings (0x10) */

#define CONTROL_SETTINGS0_INDEX     0
#define CONTROL_SETTINGS1_INDEX     1

/* ControlSettings0 */
#define EVENT_MODE              (1 << 0)
#define STREAMING_MODE          (0 << 0)
#define COMM_MODE_MASK          (~EVENT_MODE)
#define TRACKPAD_RESEED         (1 << 1)
#define AUTO_ATI                (1 << 2)
#define MODE_SELECT             (1 << 3)
#define PROX_MODE               (1 << 3)    // Bit 3 toggles between Prox and Normal
//#define NORMAL_MODE           (0 << 3)
#define PM_RESEED               (1 << 4)
#define SINGLE_XY               (1 << 5)    // 0 = Multi-touch from trackpad, 1 = Single touch
#define MULTI_XY                (0 << 5)
#define AUTO_MODES              (1 << 6)
#define MANUAL_MODE             (0 << 6)
#define ACK_RESET               (1 << 7)

/* ControlSettings1 */
#define SNAP_EN                 (1 << 0)
#define LOW_POWER               (1 << 1)
#define SLEEP_EN                (1 << 2)
#define REVERSE_EN              (1 << 3)
#define DIS_PMPROX_EVENT        (1 << 4)    // 1 = Proximity on a normal channel cannot trigger an Event
#define DIS_SNAP_EVENT          (1 << 5)    // 1 = Snap cannot trigger an Event
#define DIS_TOUCH_EVENT         (1 << 6)    // 1 = Touch cannot trigger an Event
#define DIS_PROX_EVENT          (1 << 7)    // 1 = Proximity on a prox channel cannot trigger an Event

//------------------------------------------------------------------------------
/* Threshold Settings (0x11) */
// Indices for the data in the Threshold Settings block
#define PROX_THRESHOLD_INDEX            0
#define TOUCH_THRESHOLD_MULT_INDEX      1
#define TOUCH_THRESHOLD_SHIFT_INDEX     2
#define PM_PROX_THRESHOLD_INDEX         3
#define SNAP_THRESHOLD_HIGH_INDEX       4
#define SNAP_THRESHOLD_LOW_INDEX        5
#define PROX_THRESHOLD2_INDEX           6
#define TOUCH_THRESHOLD2_MULT_INDEX     7   // No Function in AZOTEQ_1_0_9
#define TOUCH_THRESHOLD2_SHIFT_INDEX    8



//------------------------------------------------------------------------------
/* ATI Settings (0x12) */
#define TP_ATI_TARGET_HIGH_INDEX			0
#define TP_ATI_TARGET_LOW_INDEX				1
#define TP_ATIC_INDEX						2
#define NONTP_ATI_TARGET_HIGH_INDEX			3
#define NONTP_ATI_TARGET_LOW_INDEX			4
#define NONTP_ATIC_INDEX					5

//------------------------------------------------------------------------------
/* Filter Settings (0x13) */
/* FilterSettings0 */
// Indices for the data in the Filter Settings block
#define FILTER_SETTINGS0_INDEX            0
#define TOUCH_FILTER_DAMPING_INDEX        1
#define HOVER_FILTER_DAMPING_INDEX        2
#define PM_COUNT_FILTER_DAMPING_INDEX     3
#define LP_PM_COUNT_FILTER_DAMPING_INDEX  4
#define NM_COUNT_FILTER_DAMPING_INDEX     5

// bits defined in FilterSettings0
#define DIS_TOUCH_FILTER        (1 << 0)
#define DIS_HOVER_FILTER        (1 << 1)
#define SELECT_TOUCH_FILTER     (1 << 2)
#define DIS_PM_FILTER           (1 << 3)
#define DIS_NM_FILTER           (1 << 4)

//------------------------------------------------------------------------------
/* Timing Settings (0x14) */
#define RESEED_TIME_INDEX     0
#define COMMS_TIMEOUT_INDEX   1
#define MODE_TIME_INDEX       2
#define LP_TIME_INDEX         3
#define SLEEP_TIME_INDEX      4

#define LPTIME_1MS      1
#define LPTIME_2MS      2
#define LPTIME_5MS      3
#define LPTIME_10MS     4
#define LPTIME_20MS     5
#define LPTIME_40MS     6
#define LPTIME_80MS     7
#define LPTIME_160MS    8
#define LPTIME_320MS    9
#define LPTIME_640MS   10
#define LPTIME_1200MS  11
#define LPTIME_2400MS  12
#define LPTIME_5000MS  13

//------------------------------------------------------------------------------
/* Channel Setup (0x15) */
#define TOTAL_RXS_INDEX					0
#define TOTAL_TXS_INDEX					1
#define TRACKPAD_RXS_INDEX				2
#define TRACKPAD_TXS_INDEX				3
#define PM_SETUP0_INDEX					4
#define TX_HIGH_INDEX					5
#define TX_LOW_INDEX					6

#define RX_SELECT_MASK      0x0F
#define SUM_OF_TP           0x10
#define CHARGE_TYPE         0x80

//------------------------------------------------------------------------------
/* Hardware Config Settings (0x16) */
#define PROX_SETTINGS0_INDEX			0
#define PROX_SETTINGS1_INDEX			1
#define PROX_SETTINGS2_INDEX			2
#define PROX_SETTINGS3_INDEX			3

//------------------------------------------------------------------------------
/* Active Channels (0x17) */


//------------------------------------------------------------------------------
/* Debounce Settings (0x18) */
#define PROX_DB_INDEX					0
#define TOUCH_SNAP_INDEX				1


//------------------------------------------------------------------------------
/* ProxMode Proximity Status (0x20) */
#define PM_PROX_INDEX					0

//------------------------------------------------------------------------------
/* ProxMode Count Data Read (0x21) */
#define PM_COUNT_HIGH_INDEX				0
#define PM_COUNT_LOW_INDEX				1

//------------------------------------------------------------------------------
/* ProxMode Long-term Average (0x22) */
#define PM_LTA_HIGH_INDEX				0
#define PM_LTA_LOW_INDEX				1


//------------------------------------------------------------------------------
/* ProxMode ATI Settings (0x24) */
#define PM_ATI_TARGET_HIGH_INDEX		0
#define PM_ATI_TARGET_LOW_INDEX			1
#define PM_ATIC_INDEX					2


//////////// Default Configuration Values /////

#define AZO_IQS572_NUM_CHANNELS             (TOTAL_RXS_VAL * TOTAL_TXS_VAL)

#ifndef AZO_LOWPOWER_INTERVAL_MSECS
#define AZO_LOWPOWER_INTERVAL_MSECS         LPTIME_320MS
#endif

#ifndef AZO_SLEEP_INTERVAL_MSECS
#define AZO_SLEEP_INTERVAL_MSECS            LPTIME_5MS
#endif

#ifndef AZO_LOWPOWER_DOZE_INTERVAL_MSECS
#define AZO_LOWPOWER_DOZE_INTERVAL_MSECS    LPTIME_160MS
#endif

#ifndef AZO_SLEEP_DOZE_INTERVAL_MSECS
#define AZO_SLEEP_DOZE_INTERVAL_MSECS       LPTIME_1MS
#endif

#ifndef AZO_LOWPOWER_SENSOR_SLEEP_INTERVAL_MSECS
#define AZO_LOWPOWER_SENSOR_SLEEP_INTERVAL_MSECS  LPTIME_5000MS
#endif

#ifndef AZO_SLEEP_SENSOR_SLEEP_INTERVAL_MSECS
#define AZO_SLEEP_SENSOR_SLEEP_INTERVAL_MSECS     LPTIME_5000MS
#endif

#define AZO_DATA_INVALID	0xFF


//------ XY Info Byte definiitopn
#define SHOW_RESET			0x80
#define MODE_INDICATOR		0x40
#define NOISE_STATUS		0x20
#define LP_STATUS			0x10
#define SNAP_OUTPUT			0x80
#define NO_OF_FINGER_MASK	0x07

#endif // AZOTEQ_IQS572_H

#endif // SUPPORT_TOUCHPAD
