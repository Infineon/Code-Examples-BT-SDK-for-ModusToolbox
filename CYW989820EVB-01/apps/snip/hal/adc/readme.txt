-------------------------------------------------------------------------------
ADC Snippet Application
-------------------------------------------------------------------------------

This application demonstrates how to configure and use
ADC in WICED Eval. boards to measure DC voltage on
various DC input channels.


See chip specific readme for more information about the BT SDK.

Instructions
------------
- Connect a PC terminal to the serial port of the WICED Eval board,
  then build and download the application as described in the WICED
  Quick Start Guide.

- Every 5 seconds voltage is measured on 2 channels
     1) Vdd Core
     2) selected GPIO pin (CHANNEL_TO_MEAS_DC_VOLT)

When the selected GPIO pin is connected to ground, the output on the terminal emulator will show 0 to ~2mV.
When the selected GPIO pin is connected to 3.3V, the output on the terminal emulator will show approximately 3300mV +/- (3% of 3300mV).
When the selected GPIO pin is left unconnected, the output will capture environmental noise resulting in some lower voltage levels in the terminal emulator.

Please refer to the datasheet for more electrical specifications of ADC like Full scale voltage, Bandgap reference.,etc.

Usage
- Follow the prompts printed on the terminal
-------------------------------------------------------------------------------