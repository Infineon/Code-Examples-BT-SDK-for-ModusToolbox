// !!! this file should be replaced...
#ifdef OTA_SECURE_FIRMWARE_UPGRADE
#include <bt_types.h>
#include <p_256_ecc_pp.h>

// public key
Point ecdsa256_public_key =
{
    { 0xb34eacf0, 0x3ec9a058, 0x9de3c962, 0x6f21ae8a, 0x0d0b3967, 0x30e901b3, 0x1b2b1931, 0x6b462309, },
    { 0x21ec2ce7, 0x3f5dbaad, 0x887b63a3, 0xed6cb229, 0x049d0642, 0xd2358dab, 0x69a2b20b, 0xc71e1d03, },
};
#endif // OTA_SECURE_FIRMWARE_UPGRADE
