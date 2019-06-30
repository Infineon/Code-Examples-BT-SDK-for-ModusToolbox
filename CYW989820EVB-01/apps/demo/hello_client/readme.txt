-------------------------------------------------------------------------------
Hello Client app
-------------------------------------------------------------------------------

Overview
--------
BLE Vendor Specific Client Device

The Hello Client application is designed to connect and access services
of the Hello Sensor device.  Hello Client can connect up to three
Hello Sensor Device's.  Because handles of the all attributes of
the Hello Sensor are well known, Hello Client does not perform GATT
discovery, but uses them directly.  In addition to that Hello Client
allows another master to connect, so the device will behave as a slave
in one Bluetooth piconet and a master in another.  To accomplish that
application can do both advertisements and scan.  Hello Client assumes
that Hello Sensor advertises a special UUID and connects to the device
which publishes it.

See chip specific readme for more information about the BT SDK.

Features demonstrated
---------------------
 - Registration with LE stack for various events
 - Connection to a master and a slave
 - As a master processing notifications from the server and
   sending notifications to the client
 - As a slave processing writes from the client and sending writes
   to the server

Instructions
------------
To demonstrate the app, work through the following steps.
1. Plug the WICED eval board into your computer
2. Build and download the application (to the WICED board)
3. Connect from some client application (for example LightBlue on iOS)
4. From the client application register for notifications
5. Make sure that your slave device (hello_sensor) is up and advertising
6. Push a button on the tag board for 6 seconds.  That will start
   connection process.
7. Push a button on the hello_sensor to deliver notification through
   hello_client device up to the client
8. Repeat the steps 5 to 7 for connecting to multiple hello_sensor
    device's
-------------------------------------------------------------------------------