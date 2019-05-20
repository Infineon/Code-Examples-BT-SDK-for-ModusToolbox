
/******************************************************************************
 *                                Overview
 ******************************************************************************/
 WICED ADC sample application.

 This application demonstrates how to configure and use
 ADC in WICED Evaluation boards to measure DC voltage on
 various DC input channels.

 Once in every 5 seconds, voltage is measured on 4 channels
       1) VDD_CORE, ADC_BGREF, ADC_INPUT_VDDIO
       2) selected GPIO pin (CHANNEL_TO_MEAS_DC_VOLT)


 When the selected GPIO pin is connected to ground, the output on the terminal
 emulator will show 0 to ~2mV.

 When the selected GPIO pin is connected to 3.3V, the output on the terminal
 emulator will show approximately 3300mV +/- (3% of 3300mV).

 When the selected GPIO pin is left unconnected, the output will capture environmental
 noise resulting in some lower voltage levels in the terminal emulator.

 Please refer to the datasheet for more electrical specifications of ADC like
 Full scale voltage, Bandgap reference.,etc.

/******************************************************************************
 *                                Features demonstrated
 ******************************************************************************/
  - Functionality of ADC and corresponding WICED APIs
  - How to convert raw ADC sample into sampled voltage in millivolts.
  - How to initialise and start timers.

/******************************************************************************
 *                                Instructions
 ******************************************************************************/
 To demonstrate the app, work through the following steps.
 1. Plug CYW920819EVB-02 Wiced Evaluation board into your PC.
 2. Build and download the application (See CYW920819EVB-02 Kit User Guide).
 3. Use Terminal emulation tools like Teraterm to view the log/trace messages (See CYW920819EVB-02 Kit User Guide).
 4. The user can notice the raw sample values and voltage values in the terminal emulation tool.

