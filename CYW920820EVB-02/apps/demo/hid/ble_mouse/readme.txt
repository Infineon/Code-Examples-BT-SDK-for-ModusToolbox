-------------------------------------------------------------------------------
BLE Mouse
-------------------------------------------------------------------------------

Overview
--------
The BLE Mouse application is a single chip SoC compliant with HID over GATT Profile (HOGP).

During initialization the app registers with LE stack, WICED HID Device Library and
keyscan HW to receive various notifications including bonding complete, connection
status change, peer GATT request/commands and interrupts for button pressed/released.
Press any button will start LE advertising. When device is successfully bonded, the app
saves bonded host's information in the NVRAM.
When user presses/releases button, a HID report will be sent to the host.
On connection up or battery level changed, a battery report will be sent to the host.
When battery level is below shutdown voltage, device will critical shutdown.

Features demonstrated
---------------------
 - GATT database and Device configuration initialization
 - Registration with LE stack for various events
 - Sending HID reports to the host
 - Processing write requests from the host
 - Low power management
 - Over the air firmware update (OTAFWU)

Instructions
-------------
To demonstrate the app, walk through the following steps.
1. Plug the CYW920819EVB_02 board or 20819A1 mouse HW into your computer
2. Put on jumper to bypass Serial Flash (i.e. jumper on J5 in CYW920819EVB_02 board), then power up the board or mouse HW.
3. Remove the jumper so that download procedure below can write to Serial Flash.
4. Build and download the application (to the EVAL board or the mouse HW)
5. If download failed due to not able to detecting device, just repeat step 4 again.
6. Unplug the EVAL board or the mouse HW from your computer (i.e. unplug the UART cable)
7. power cycle the EVAL board or the mouse HW.
8. Press any button to start LE advertising, then pair with a PC or Tablet
   If using the CYW920819EVB board, use a fly wire to connect GPIO P0 and P8 to simulate LEFT button press,
    and remove the wire to simulate button release.
9. Once connected, it becomes the mouse of the PC or Tablet.


In case what you have is only the WICED EVAL board, you can only use fly wire to connect to GPIOs (GPIO P0 and P8) to simulate mouse button press and release.
Or using the ClientControl tool in the tools to simulate button press and mouse movement.
1. Plug the WICED EVAL board into your computer
2. Build and download the application (to the WICED board)
3. If failed to download due to device not detected, just repeat step 2 again.
4. Press any button to start LE advertising, then pair with a PC or Tablet
    Use a fly wire to connect GPIO P0 and P8 to simulate LEFT button press,
    and remove the wire to simulate button release.
5. Once connected, it becomes the mouse of the PC. However, you can't see scroll or mouse cursor movement
    since you don't have the real HW.


You can also use the WICED board and ClientControl tool test the basic BLE functions.
NOTE: Make sure you use "TESTING_USING_HCI=1" in application settings.
In ModusToolbox, select right click on app and select 'Change Application Settings'

1~3. same download procedure as above
4. Run ClientControl.exe.
5. Choose 3M as Baudrate and select the serial port in ClientControl tool window.
6. Press Reset button on the board and open the port.
7. Press "Enter Pairing Mode"or "Connect" to start LE advertising, then pair with a PC or Tablet
8. Once connected, it becomes the mouse of the PC or Tablet.
 - Select Interrupt channel, Input report, enter the contents of the report
   and click on the Send button, to send the report.  For example:
   To send button down event when LEFT button is pushed, report should be
   02 01 00 00 00 00.  All buttons up 02 00 00 00 00 00.
   To send scroll up event, report should be
   02 00 00 00 00 ff.
   To send scroll down event, report should be
   02 00 00 00 00 01.
   To send mouse cursor (Y+8) event, report can be
   02 00 00 80 ff 00.
   To send mouse cursor (Y-8) event, report can be
   02 00 00 80 00 00.
   To send mouse cursor (X+8) event, report can be
   02 00 08 00 00 00.
   To send mouse cursor (X-8) event, report can be
   02 00 f8 0f 00 00 .
   Please make sure you always send a button up report following button down report.

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
ENABLE_SCROLL
    Use this option to enable scroll
ENABLE_MOTION
    Use this option to enable motion
ASSYMETRIC_SLAVE_LATENCY
    Use this option to set assymetric slave latency.

-------------------------------------------------------------------------------
