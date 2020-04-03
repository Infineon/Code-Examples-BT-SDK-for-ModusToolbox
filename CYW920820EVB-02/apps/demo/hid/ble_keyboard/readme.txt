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
TESTING_USING_HCI
    Use this option for testing with Bluetooth Profile Client Control. The Client
    Control UI can be used to provide input. When this option is enabled, the
    device will not enter SDS/ePDS for power saving.

OTA_FW_UPGRADE
    Use this option for enabling firmware upgrade over the Air (OTA) capability.

OTA_SEC_FW_UPGRADE
    Use this option for secure OTA firmware upgrade. OTA_FW_UPGRADE option must be
    enabled for this option to take effect.

AUTO_RECONNECT
    Use this option to enable auto reconnect. By enabling this option, the device
    will always stay connected. If it is disconnected, it try to reconnect until
    it is connected.

    This option should be used together with DISCONNECTED_ENDLESS_ADV. When this
    option is enabled, the HID device will always try to maintain connection with
    the paired HID host; therefore, if the link is down, it will continuously try
    to reconnect. To conserve power, it should allow entering SDS/ePDS while
    advertising; thus, the DISCONNECTED_ENDLESS_ADV option should be enabled;
    otherwise, it may drain battery quickly if host was not available to reconnect.

DISCONNECTED_ENDLESS_ADV
    Use this option to enable disconnected endless advertisement. When this option
    is used, the device will do advertising forever until it is connected. To
    conserve power, it allows SDS/ePDS and do the advertising in a long interval.

SKIP_PARAM_UPDATE
    Use this option to skip to send link parameter update request.
    When this option is disabled, if the peer device (master) assigned link parameter
    is not within the device's preferred range, the device will send a request for
    the desired link parameter change. This option can be enabled to stop the device
    from sending the reuqest and accept the given link parameter as is.

    Background:
    In some OS (peer host), after link is up, it continuously sends different
    parameter of LINK_PARAM_CHANGE over and over for some time. When the parameter
    is not in our device preferred range, the firmware was rejecting and renegotiating
    for new preferred parameter. It can lead up to endless and unnecessary overhead
    in link parameter change. Instead of keep rejecting the link parameter, by using
    this option, we accept peer requested link parameter as it and starts a timer to
    send the final link parameter change request later when the peer host settles down
    in link parameter change.

ASSYMETRIC_SLAVE_LATENCY
    Use this option to enable assymetric slave latency.

    Background:
    In early days, some HID host devices will always reject HID slave's link
    parameter update request. Because of this, HID device will end up consuming
    high power when slave latency was short. To work around this issue, we use
    Asymmetric Slave Latency method to save power by waking up only at multiple
    time of the communication anchor point. When this option is enabled,

    1.  We do not send LL_CONNECTION_PARAM_REQ.
    2.  We simply start Asymmetric Slave Latency by waking up at multiple times
        of given slave latency.

    Since this is not a standard protocol, we do not recommend enabling this
    option unless if it is necessary to save power to work around some HID hosts.

LE_LOCAL_PRIVACY
    When enabled, the device uses RPA (Random Private Address).
    When disabled, the device uses Public static address.

-------------------------------------------------------------------------------