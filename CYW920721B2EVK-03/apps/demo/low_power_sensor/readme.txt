-------------------------------------------------------------------------------
Low power BLE Vendor Specific Device
-------------------------------------------------------------------------------

Overview
--------
During initialization the application registers with LE stack to receive various
notifications including bonding complete, connection status change,
peer write and configure to enter to sleep on application idle time out(10 seconds).
If no interaction from peer until 10 seconds of application start, device enter to sleep.
Device can be woken up using GPIO (Please check device_wake_gpio_num
to get to know  which pin configure as wake source) toggle or by interacting with peer.
When device is successfully bonded, application saves
peer's Bluetooth Device address to the NVRAM.  Bonded device can also
write in to client configuration descriptor of the notification
characteristic.  That is also saved in the NVRAM.  When user pushes the
button(when device in wake up state), notification/indication is sent to the bonded and
registered host.

See chip specific readme for more information about the BT SDK.

Instructions
------------
To demonstrate the application, use the following steps -
1. Plug the WICED eval board into your computer
2. Build and download the application (to the WICED board)
   (a) To configure shutdown sleep, set SLEEP_TYPE=SLEEP_TYPE_SHUTDOWN in application settings.
   (b) To configure non shutdown sleep, set SLEEP_TYPE=SLEEP_TYPE_NOT_SHUTDOWN in application settings.
   (c) To configure sleep with transport, set SLEEP_MODE=SLEEP_MODE_TRANSPORT in application settings.
       Note: Device cannot sleep without connecting to transport. Hence expecting device should be connected
       to transport after application boot.
   (d) To configure sleep without transport, set SLEEP_MODE=SLEEP_MODE_NO_TRANSPORT in application settings..
   Note: Default configuration is sleep without transport and Shut Down Sleep.

3. Starts advertisements.
4. If no one attempt to pair, app enter to sleep after idle timeout(10 seconds).
5. Pair with a client. App should wake from sleep if it is in sleep and should allow pairing
    Note: when app is advertising in sleep mode, peer need to re attempt connection,
    if fails first time
6. On the client side register for notifications
7. Push a button on the board to send notifications to the client
8. If no activity on app, enters sleep on idle time out.
9. Wake the device.
    (a) Wake using configured wake source (Please check device_wake_gpio_num
        to get to know  which pin configure as wake source).
        Initially wake source need to connect to ground. To wake the device, connect wake source to High.
        to allow device to sleep again, connect wake source back to ground.
        Note: In this sample, button configured using LHL GPIO. Hence device can be woken up using button press also. But cannot
        guaranteed that on button press wake, interrupt handler get called, but guaranteed that device wake up. Correct button usage is
        wake device using wake source then press button to send notification/to start ADV to get to connect etc.
    (b) If device is in connected state and in sleep, peer interactions (for example GATT attribute Read/write) cause device wake.

Note: When sleep configured with SLEEP_MODE_TRANSPORT, without connecting to transport(i.e. to WICED_HCI UART)
     device does not allowed sleep. Expectation is after power up, transport should be connected immediately.

Features demonstrated
---------------------
 - Configuring for Shut Down Sleep and non Shut Down Sleep
 - GATT database and Device configuration initialization
 - Registration with LE stack for various events
 - NVRAM read/write operation
 - Configuring for sleep on hello_sensor idle.
 - Saving and restoring application context on entering and exiting from sleep.
 - Sending data to the client
 - Processing write requests from the client

Application Settings
--------------------
Application specific settings are -
SLEEP_MODE
    SLEEP_MODE_NO_TRANSPORT - Use this option to configure sleep with no transport.
    SLEEP_MODE_TRANSPORT - Use this option to configure sleep with transport enabled.
SLEEP_TYPE
    SLEEP_TYPE_SHUTDOWN - Use this option to configure shutdown sleep.
    SLEEP_TYPE_NOT_SHUTDOWN - Use this option to configure non shutdown sleep.

-------------------------------------------------------------------------------