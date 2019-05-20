/******************************************************************************
 *                                Overview
 ******************************************************************************/
 WICED PWM sample application.

 This application demonstrates how to configure and use
 PWM in WICED Eval. boards to control the brightness of the
 LED's onboard

/******************************************************************************
 *                                Features demonstrated
 ******************************************************************************/
 - PWM WICED API's
 - LED Breathing effect

/******************************************************************************
 *                                Instructions
 ******************************************************************************/
 To demonstrate the app, work through the following steps.
 1. Plug the CYW920819EVB-02 evaluation board to your computer.
 2. Build and download the application. (See CYW920819EVB-02 User Guide)
 3. Use Terminal emulation tools like Teraterm or Putty to view the
    trace messages(See Kit User Guide).
 4. The user can notice the LED breathing effect on the LEDs.
 5. On pressing user button, the application toggles between starting and
    stopping PWM.
 6. When stopping the PWM, the application stays in the last duty cycle and
    retains the same illumination level in the LED.