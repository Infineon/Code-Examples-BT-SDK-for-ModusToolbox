-------------------------------------------------------------------------------
Hello Sensor app
-------------------------------------------------------------------------------

Overview
--------
BLE Vendor Specific Device

During initialization the app registers with LE stack to receive various
notifications including bonding complete, connection status change and
peer write.  When device is successfully bonded, application saves
peer's Bluetooth Device address to the NVRAM.  Bonded device can also
write in to client configuration descriptor of the notification
characteristic.  That is also saved in the NVRAM.  When user pushes the
button, notification/indication is sent to the bonded and registered host.
User can also write to the characteristic configuration value. Number
of LED blinks indicates the value written.

See chip specific readme for more information about the BT SDK.

Features demonstrated
----------------
 - GATT database and Device configuration initialization
 - Registration with LE stack for various events
 - NVRAM read/write operation
 - Sending data to the client
 - Processing write requests from the client

Instructions
------------
To demonstrate the app, work through the following steps.
1. Plug the WICED eval board into your computer
2. Build and download the application (to the WICED board)
3. Pair with client.
4. On the client side register for notifications
5. Push a button on the tag to send notifications to the client
6. Write the hello sensor characteristic configuration value from client
7. Number of LED blinks on hello sensor indicates value written by client
-------------------------------------------------------------------------------