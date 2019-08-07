-------------------------------------------------------------------------------
GPIO Snippet Application
-------------------------------------------------------------------------------

This application demonstrates how to use WICED GPIO APIs to,

 a) configure GPIO to be an input pin (and receive interrupt upon change of pin state)
 b) configure GPIO to be an output pin (by toggling LED's available on reference board)

The GPIO's in the array output_pin_list[] are configured as an output
pin and toggled at predefined frequency(APP_TIMEOUT_IN_SECONDS_A).
The GPIO's in the array input_pin_list[] are configured as an interrupt
enabled input pin, upon pressing this button the blink rate
of LED's is toggled between LED_BLINK_FREQ_A_IN_SECONDS and
LED_BLINK_FREQ_B_IN_SECONDS

See chip specific readme for more information about the BT SDK.

Instructions
------------
To demonstrate the snip, work through the following steps.
1. Plug the WICED evaluation board to your computer.
2. Build and download the application.
3. Use Terminal emulation tools like Teraterm or Putty to view the trace messages
   (See Kit User Guide).
-------------------------------------------------------------------------------