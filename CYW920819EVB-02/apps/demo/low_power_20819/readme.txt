 -------------------------------------------------------------------------------
 CE225540: CYW20819 Low Power Application
 -------------------------------------------------------------------------------

 Overview
 --------
 This code example implements low power modes (ePDS and HID-Off) in CYW20819
 device. During initialization, the code example registers with BLE stack
 to receive various notifications and connection status change as well as
 registers the callback for sleep permission with the PMU.
 Advertisements can be started by pressing button SW3. The device can be
 connected to using any BLE client such as CySmart mobile application or CySmart
 PC application with CySmart dongle. Upon disconnection, the device will enter
 HID-Off mode for 10 seconds

 See 20819_readme.txt for more information about the BT SDK.

 Instructions
 ------------
 To demonstrate this code example, work through the following steps.
 1. Plug the CYW920819EVB-02 Evaluation Kit to your PC.
 2. Build and download the application.
 3. Use terminal emulation tools like Tera Term to view the log/trace messages from PUART.
    [Baud rate: 115,200bps; Data: 8 bits; Parity: None; Stop bit: 1 bit]
 4. Press button SW3 to start advertisements.
 5. Scan and connect to this device advertising as "low_power_20819" from a
    central device (e.g. CySmart mobile app).
 6. Enable notifications or read values (battery values) from the central
    device.
 7. Disconnect the bluetooth connection through CySmart application or by
    pressing button SW3 on the Kit. The device will now enter HID-Off for 10
    seconds

For more details on this code example including the design and implementation,
please refer to the CE document available under this example's folder at GitHub
(https://github.com/cypresssemiconductorco/Code-Examples-BT-20819A1-1.0-for-ModusToolbox-1.1)

For more details on the hardware connections in the CYW920819EVB-02 Evaluation
kit, please refer to the ModusToolbox CYW920819EVB-02 Evaluation Kit User Guide.pdf
(http://www.cypress.com/CYW920819EVB-02)

 Application Settings
 --------------------
 N/A

 Notes
 -----
 N/A

-------------------------------------------------------------------------------
