-------------------------------------------------------------------------------
BLE Keyboard
-------------------------------------------------------------------------------

Overview
--------
The BLE Keyboard application is a single chip SoC.  It provides a turnkey solution
using on-chip keyscan HW component and is compliant with HID over GATT Profile (HOGP).

During initialization the app registers with LE stack, WICED HID Device Library and
keyscan HW to receive various notifications including bonding complete, connection
status change, peer GATT request/commands and interrupts for key pressed/released.
Press any key will start LE advertising. When device is successfully bonded, the app
saves bonded host's information in the NVRAM.
When user presses/releases key, a key report will be sent to the host.
On connection up or battery level changed, a battery report will be sent to the host.
When battery level is below shutdown voltage, device will do critical shutdown.
Host can send LED report to the device to control LED.

Features demonstrated
---------------------
 - GATT database and Device configuration initialization
 - Registration with LE stack for various events
 - Sending HID reports to the host
 - Processing write requests from the host
 - Low power management
 - Over the air firmware update (OTAFWU)

Instructions
------------
To demonstrate the app, walk through the following steps.
1. Plug the keyboard HW into your computer
2. Build and download the application
3. Unplug the keyboard HW from your computer and power cycle the keyboard HW
4. Press any key to start LE advertising, then pair with a PC or Tablet
5. Once connected, it becomes the keyboard of the PC or Tablet.

In case you don't have the right hardware, eval_keyboard, which is required to support the 818
key matrix used in the BLE keyboard application, you will need to modify the key matrix to match your hardware.

You can also use the WICED board to simulate keyboard by using ClientControl tool to test the basic BLE functions.
NOTE: To use Client Control, make sure you use "TESTING_USING_HCI=1" in application settings.
In ModusToolbox, select right click on app and select 'Change Application Settings'

1. Plug the hardware into your computer
2. Build and download the application
3. Run ClientControl.exe.
4. Choose 3M as Baudrate and select the serial port in ClientControl tool window.
5. Press Reset button on the board and open the port.
6. Press "Enter Pairing Mode"or "Connect" to start LE advertising, then pair with a PC or Tablet
7. Once connected, it becomes the keyboard of the PC or Tablet.
 - Select Interrupt channel, Input report, enter the contents of the report
   and click on the Send button, to send the report.  For example to send
   key down event when key '1' is pushed, report should be
   01 00 00 1e 00 00 00 00 00.  All keys up 01 00 00 00 00 00 00 00 00.
   Please make sure you always send a key up report following key down report.

Notes
-----
The application GATT database is located in wiced_bt_cfg.c
If you create a GATT database using Bluetooth Configurator, update the
GATT database in the location mentioned above.

Application Settings
--------------------
Application specific settings are -
TESTING_USING_HCI
    Use this option for testing with Bluetooth Profile Client Control. The Client Control
    UI can be used to provide input.
OTA_FW_UPGRADE
    Use this option for Over The Air (OTA) upgrade
OTA_SEC_FW_UPGRADE
    Use this option for secure OTA firmware upgrade. When this option is used
    the above option for OTA_FW_UPGRADE shuold also be used.
ASSYMETRIC_SLAVE_LATENCY
    Use this option to set assymetric slave latency.
LE_LOCAL_PRIVACY
    Use this option to set LE local privacy.
SKIP_PARAM_UPDATE
    Use this option to set skip parameter update.
AUTO_RECONNECT
    Use this option to allow auto reconnect.

-------------------------------------------------------------------------------