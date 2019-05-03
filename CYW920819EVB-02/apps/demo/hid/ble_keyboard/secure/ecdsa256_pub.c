#ifdef OTA_SECURE_FIRMWARE_UPGRADE
#include <bt_types.h>
#include <p_256_ecc_pp.h>

// public key
Point ecdsa256_public_key =
{
    { 0xa8d4daca, 0xc16a87ab, 0x677cf53b, 0xcfb978ba, 0x11a80be3, 0x7bda5c50, 0x60933ed2, 0xcb867d29, },
    { 0x984acf8b, 0x5bd98614, 0xd0baf18e, 0xec337dfe, 0x46e24545, 0x24691c0c, 0x1750d454, 0xff9edf84, },
};
#endif // OTA_SECURE_FIRMWARE_UPGRADE
