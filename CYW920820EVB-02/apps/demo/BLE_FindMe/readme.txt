-------------------------------------------------------------------------------
CE226123 - BLE Find Me Profile
-------------------------------------------------------------------------------
Supported Kits
--------------
Check modus.mk file for supported kits information.

Overview
--------
 This code example implements a Bluetooth Low Energy (BLE) Find Me Profile (FMP)
 that consists of an Immediate Alert Service (IAS). FMP and IAS are BLE
 standard Profile and Service respectively, as defined by the Bluetooth SIG.

 The design uses the two LEDs (red LED, yellow LED). The red LED (LED2)
 displays the IAS alert level – no alert (LED OFF), mild alert (LED blinking),
 high alert (LED ON). The yellow LED (LED1) indicates whether the Peripheral
 device is advertising (LED blinking), connected (LED ON) or disconnected (LED OFF).

 Use any PC or iOS/Android mobile device running the CySmart application
 that acts as the BLE Central device, which locates the Peripheral device.
 Peripheral device name is "Find Me Target" and is defined in the file
 app_bt_cfg.c.

 See chip specific readme for more information about the BT SDK.

Instructions
------------
 To evaluate this code example, do the following steps.
 1. Connect any one of the supported kits to your PC using the provided USB cable.
 2. Build and Program the Application: In the project explorer, select the
    <App Name>_mainapp project. In the Quick Panel, scroll to the Launches
    section, and click the <App Name> Build + Program configuration.
 3. Turn ON Bluetooth on your Android or iOS device.
 4. Launch the CySmart app.
 5. Press the reset switch on the kit to start sending advertisements.
    The yellow LED (LED1) starts blinking to indicate that advertising has
    started. Advertising will stop after 90 seconds if a connection has not
    been established.
 6. Swipe down on the CySmart app home screen to start scanning for BLE
    Peripherals; your device (Find Me Target) appears in the CySmart app
    home screen. Select your device to establish a BLE connection. Once the
    connection is established, the yellow LED (LED1) changes from blinking
    state to always ON state.
 7. Select the 'Find Me' Profile from the carousel view.
 8. Select an Alert Level value on the Find Me Profile screen. Observe the
    state of the red LED (LED2) on the device changes based on the alert level.
 9. Use a serial terminal application like Tera Term to view the Bluetooth
    stack and application trace messages. Open the "WICED Peripheral UART"
    serial port with the below settings.
    [Baud rate: 115,200bps; Data: 8 bits; Parity: None; Stop bit: 1 bit]

For more details on this code example including the design and implementation,
please refer to the CE document available under this example's folder at GitHub.
https://github.com/cypresssemiconductorco/Code-Examples-BT-SDK-for-ModusToolbox
---------------------------------------------------------------------------------------