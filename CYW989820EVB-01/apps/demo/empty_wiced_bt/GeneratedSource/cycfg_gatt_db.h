/***************************************************************************//**
* TODO - Use the Bluetooth Configurator to create this file which contains GATT database
* Below is sample source code created by BT Configurator
*******************************************************************************/

#if 0 // TODO

#if !defined(CYCFG_GATT_DB_H)
#define CYCFG_GATT_DB_H

#include "stdint.h"

#define __UUID_SERVICE_GENERIC_ACCESS               0x1800u
#define __UUID_CHARACTERISTIC_DEVICE_NAME           0x2A00u
#define __UUID_CHARACTERISTIC_APPEARANCE            0x2A01u
#define __UUID_SERVICE_GENERIC_ATTRIBUTE            0x1801u

/* Service Generic Access */
#define HDLS_GAP                                    0x0001u
/* Characteristic Device Name */
#define HDLC_GAP_DEVICE_NAME                        0x0002u
#define HDLC_GAP_DEVICE_NAME_VALUE                  0x0003u
/* Characteristic Appearance */
#define HDLC_GAP_APPEARANCE                         0x0004u
#define HDLC_GAP_APPEARANCE_VALUE                   0x0005u

/* Service Generic Attribute */
#define HDLS_GATT                                   0x0006u

/* External Lookup Table Entry */
typedef struct
{
    uint16_t handle;
    uint16_t max_len;
    uint16_t cur_len;
    uint8_t  *p_data;
} gatt_db_lookup_table_t;

/* External definitions */
extern const uint8_t  gatt_database[];
extern const uint16_t gatt_database_len;
extern gatt_db_lookup_table_t app_gatt_db_ext_attr_tbl[];
extern const uint16_t app_gatt_db_ext_attr_tbl_size;
extern uint8_t app_gap_device_name[];
extern const uint16_t app_gap_device_name_len;
extern uint8_t app_gap_appearance[];
extern const uint16_t app_gap_appearance_len;

#endif /* CYCFG_GATT_DB_H */

#endif // TODO