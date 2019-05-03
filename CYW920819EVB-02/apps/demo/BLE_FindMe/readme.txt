-------------------------------------------------------------------------------
CE226123 - CYW20819 BLE Find Me Profile
-------------------------------------------------------------------------------

Overview
--------
 This code example implements a Bluetooth Low Energy (BLE) Find Me Profile
 (FMP) that consists of an Immediate Alert Service (IAS). FMP and IAS are BLE
 standard Profile and Service respectively, as defined by the Bluetooth SIG.

 The design uses the two LED’s (red LED, yellow LED) on the CYW920819EVB-02
 kit. The red LED (LED2) displays the IAS alert level – no alert (LED OFF),
 mild alert (LED blinking), high alert (LED ON). The yellow LED (LED1)
 indicates whether the Peripheral device (CYW20819) is advertising
 (LED blinking), connected (LED ON), or disconnected (LED OFF).

 Use any PC or iOS/Android mobile device running the CySmart application
 that acts as the BLE Central device, which locates the Peripheral device.
 Peripheral device name is "Find Me Target" and is defined in the file
 app_bt_cfg.c.

 See 20819_readme.txt for more information about the Bluetooth SDK.

Instructions
------------
 To evaluate this code example, do the following steps.
 1. Connect the CYW920819EVB-02 kit to your PC using the provided USB cable.
 2. Build and Program the Application: In the project explorer, select the
    <App Name>_mainapp project. In the Quick Panel, scroll to the Launches
    section, and click the <App Name> Build + Program configuration
 3. Turn ON Bluetooth on your Android or iOS device.
 4. Launch the CySmart app.
 5. Press the reset switch on the CYW920819EVB-02 kit to start sending
    advertisements. The yellow LED (LED1) starts blinking to indicate that
    advertising has started. Advertising will stop after 90 seconds if a
    connection has not been established.
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

 For more details on the code example including the design and implementation
 details, refer to the CE document available under this example's folder at github
 https://github.com/cypresssemiconductorco/Code-Examples-BT-20819A1-1.0-for-ModusToolbox-1.1

 For more details on the hardware connections in the CYW920819EVB-02 Evaluation
 kit, please refer to the ModusToolbox CYW920819EVB-02 Evaluation Kit User Guide.pdf
 (http://www.cypress.com/CYW920819EVB-02)

---------------------------------------------------------------------------------------