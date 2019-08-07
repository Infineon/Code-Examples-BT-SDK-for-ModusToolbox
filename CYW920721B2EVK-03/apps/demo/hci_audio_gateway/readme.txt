-------------------------------------------------------------------------------
Audio Gateway app
-------------------------------------------------------------------------------

Overview
--------
This app demonstrates use of Bluetooth Audio Gateway profile.
It also demonstrates GATT/GAP and SPP functionality.

See chip specific readme for more information about the BT SDK.

Instructions
------------
To demonstrate the app, follow these steps -

1. Build and download the application to the WICED board
2. Use ClientControl application to send various commands


BLE
 - To find Gatt devices: Click on the "Start Ble Discovery" button
 - To start adverterisements: Click on the "Start Adverts" button
 - To connect Ble device: Choose device from the drop down combo box and
   click "Connect" button
 - To discover services: Click on the "Discover Services" button
 - To discover characteristics: Enter the handles in the edit box and click
   on "Discover Characteristics"
 - To discover descriptors: Enter the handles in the edit box and click on
   "Discover Characteristics"
 - Enter the Handle and Hex Value to write to the remote device using buttons
    "Write" : Write hex value to remote handle
    "Write no rsp" : Write hex value without response to remote handle
    "Value Notify" : Write notification value to the remote handle
    "Value Indicate" : Write indication value to the remote handle

BR/EDR
- To find BR/EDR devices: Click on "Start BR/EDR Discovery"

SPP
- To create an SPP Connection to remote SPP server choose the bluetooth address
  of the remote device from the BR/EDR combo box

AG Connection
- To create audio gateway connection to remote handsfree controller, choose the bluetooth
  address of the remote device from the BR/EDR combo box
- Click "Connect" button under AG
- To establish SCO connection, click on Audio connect
- Check SCO audio quality
- NOTE : Default WBS is disabled, update hf_control_esco_params structure to enabled it.
-------------------------------------------------------------------------------