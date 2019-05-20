#ifdef OTA_SECURE_FIRMWARE_UPGRADE
#include <bt_types.h>
#include <p_256_ecc_pp.h>

// public key
Point ecdsa256_public_key =
{
    { 0x85f132ce, 0x2b9c8e25, 0xb98a0f8c, 0xcd5170e0, 0xa8de4dd0, 0x9925aefa, 0x0940d20b, 0x198a0a0a, },
    { 0x3081b01d, 0xa51ce7fc, 0x6b74c2b6, 0x0f8c8287, 0xde2d0a37, 0xdc8f6d5c, 0x5f73adb5, 0x77359045, },
};
#endif // OTA_SECURE_FIRMWARE_UPGRADE
