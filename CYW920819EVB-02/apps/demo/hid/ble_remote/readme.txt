-------------------------------------------------------------------------------
BLE Remote Control
-------------------------------------------------------------------------------

Overview
--------

The BLE Remote Control application is a single chip SoC compliant with HID over GATT Profile (HOGP).
Supported features include key, microphone (voice over HOGP), Infrared Transmit (IR TX), TouchPad.

During initialization the app registers with LE stack, WICED HID Device Library and
keyscan and external HW peripherals to receive various notifications including
bonding complete, connection status change, peer GATT request/commands and
interrupts for key pressed/released, ADC audio, and Touchpad.
Press any key will start LE advertising. When device is successfully bonded, the app
saves bonded host's information in the NVRAM.
When user presses/releases any key, a key report will be sent to the host.
On connection up or battery level changed, a battery report will be sent to the host.
When battery level is below shutdown voltage, device will critical shutdown.
When user presses and holds microphone key, voice streaming starts until user releases
microphone key.
When user presses and holds power key, IR TX starts until power key is released.

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
To demonstrate the app, walk through the following steps -
1. Plug the CYW920819EVB_02 board or the 20819A1 Remote Control HW into your computer
2. Put on jumper to bypass Serial Flash (i.e. jumper on J5 in CYW920819EVB_02 board), then power up the board or Remote Control HW.
3. Remove the jumper so that download procedure below can write to Serial Flash.
4. Build and download the application (to the EVAL board or Remote Control HW).
5. If download failed due to not able to detecting device, just repeat step 4 again.
6. Unplug the EVAL board or Remote Control HW from your computer (i.e. unplug the UART cable)
7. Power cycle the EVAL board or Remote Control HW.
8. Press any key to start LE advertising, then pair with a TV
   If using the CYW920819EVB_02 board, use a fly wire to connect GPIO P0 and P11 to simulate key '0' press,
    and remove the wire to simulate key release.
9. Once connected, it becomes the remote control of the TV.
10. If you have the 20819A1 Remote Control HW:
    - Press and hold microphone key, voice streaming starts until the key is released.
    - Touch touchpad, touchpad report will be sent to the TV.

In case what you have is the WICED EVAL board, you can either use fly wire to connect to GPIOs to simulate key press and release.
Or using the ClientControl tool in the tools to simulate key press and release.
1. Plug the WICED EVAL board into your computer
2. Build and download the application (to the WICED board).
3. If failed to download due to device not detected, just repeat step 2 again.
4. Press any key to start LE advertising, then pair with a TV
    Use a fly wire to connect GPIO P0 and P11 to simulate key '0' press,
    and remove the wire to simulate key release.
5. Once connected, it becomes the remote control of the TV.


To use ClientControl tool + WICED EVAL board to simulate key press and release.
NOTE: Make sure you use "TESTING_USING_HCI=1" in application settings.
In ModusToolbox, select right click on app and select 'Change Application Settings'

1~3. same download procedure as above
4. Run ClientControl.exe.
5. Choose 3M as Baudrate and select the serial port in ClientControl tool window.
6. Press Reset button on the board and open the port.
7. Press "Enter Pairing Mode"or "Connect" to start LE advertising, then pair with a PC or Tablet
8. Once connected, it becomes the remote control of the TV.
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

START_ADV_ON_POWERUP
    Use this option to start advertisement after power up (cold boot). By enabling
    this option, after power up, the device will automatically try to connect to the
    paired host if it is already paired. If it is not paried, it will enter discovery
    mode for pairing.

ENABLE_CONNECTED_ADV
    Use this option to allow advertisement for new host pairing while
    connected to a host.

ENABLE_EASY_PAIR
    Use this option to enable easy pairing.

    This is a proprietary method of pairing to bond connection between HID host
    and HID device. Instead sending advertising, the HID device will do scanning
    to find the preferred hosts in a list. Then select the strongest RSSI
    preferred host to bond the connection. To enable this function, the HID
    hosts must also need to enable EASY_PAIRING method.

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

ENABLE_FINDME
    Use this option to enable Find Me profile.

ENABLE_AUDIO
    Use this option to enable audio function to send voice over HOGP (HID over
    GATT Protocol). By default, mSBC encoding method is used unless one of the
    following option is enabled:

  OPUS_CELT_ENCODER: use Opus Celt encoder
  ADPCM_ENCODER: use ADPCM encoder

    If both OPUS_CELT_ENCODER and ADPCM_ENCODER are defined, Opus Celt encoding
    method is used.

  ENABLE_DIGITAL_MIC
    Use this option to enable digital microphone. ENABLE_AUDIO option must be
    enabled for this option to take effect.

ENABLE_TOUCHPAD
    Use this option to enable touchpad functions. The option requires actual
    demo remote hardware to be functional.

ENABLE_IR
    Use this option to enable IR functions. The option requires actual demo
    remote hardware to be functional. A sample dummy IR protocol is used. The
    developer will need to decide and implement the actual IR protocol.

ENABLE_MOTION
    Use this option to enable motion function. The option requires actual demo
    remote hardware to be functional.

  POLL_MOTION_WHILE_CONNECTED
    Use this option to enable poll motion while connected. This option takes
    effect only if ENABLE_MOTION option is enabled. When this option is used,
    the motion interrupt will be used only to wake up the device. The motion
    data will be polled by activity polling routine. When this option is disabled,
    the motion data will be collected by motion interrupt. This option takes
    effect only if ENABLE_MOTION option is enabled.

  ENABLE_MOTION_AS_AIR_MOUSE
    Use this option to enable motion as Air Mouse. This option takes effect
    only if ENABLE_MOTION option is enabled.

    When this option is used, the device will send mouse report instead of motion
    report to HID host. This sample code uses 3rd party proprietary library for
    motion to mouse data conversion. Cypress semiconductor does not own this
    conversion algorithm. Who wish to enable this option, will need to contact
    the 3rd party or to develop their own conversion algorithm.

-------------------------------------------------------------------------------
