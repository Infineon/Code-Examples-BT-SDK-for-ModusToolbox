-------------------------------------------------------------------------------------------------------------
Bluetooth Over the Air (OTA) Firmware Upgrade Application
--------------------------------------------------------------------------------------------------------------

 This is a snip application that demonstrates how to link with the
 OTA FW Upgrade library.  The application responsibility is to
 publish OTA FW upgrade service in the GATT database and to pass
 stack callbacks to the library.
 See libraries/fw_upgrade_lib/ota_fw_upgrade.c file for the
 description of the OTA protocol.
 This version of the OTA firmware upgrade relies on the Bluetooth
 standard security.  The application also can use ECC cryptography
 to validate the image. It is expected that full implementation
 will use application level verification that downloaded firmware is
 for this Product ID and that the image has not been tampered with.

 Features demonstrated
  - OTA Firmware Upgrade

 To use OTA, find the peer OTA applications in folder below and follow the readme.txt.
 ModusToolbox - <Install Dir>\ModusToolbox_1.1\libraries\bt_sdk-1.x\components\BT-SDK
 WICED Studio - <Install Dir>\WICED-Studio-X.X

 \common\peer_apps\ota_firmware_upgrade\readme.txt

 Project Settings
 ----------------
 Application specific project settings are as below -
 OTA_SEC_FW_UPGRADE
    This option enables secure OTA
----------------------------------------------------------------------------------------------------------------------------
