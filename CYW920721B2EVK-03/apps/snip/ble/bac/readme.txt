-------------------------------------------------------------------------------
Battery Service Client application
-------------------------------------------------------------------------------

Overview
--------
The Battery Service Client application is designed to connect and access services of the Battery Server.
Battery Client connects to a device advertising Battery Service UUID.

Features demonstrated
---------------------
 - Registration with LE stack for various events.
 - Read characteristic from a Battery Service Server device.
 - Process the notifications received from the server.

Instructions
------------
To demonstrate the app, work through the following steps.
1. Plug the WICED eval board into your computer
2. Build and download the application (to the WICED board)
3. On start of the application, push the button on the tag board and release with in 2 seconds,so that
   Battery Client App scans and connects to the Battery Service Server, which would have
   UUID_SERVICE_BATTERY in it's advertisements.
   Note:- If no Battery Service Server device is found nearby for 90secs, then scan stops automatically.
   To restart the scan, push the button on the tag board and release within 2 secs.
4. Upon successful Connection, the Battery Client App would discover all the characteristics/descriptors
   of the server device.
5. Once the connection is established with the BLE peripheral (Battery Service found in the Peripheral),
   the application can enable/disable for notifications, to receive the change in the battery level.
   To enable/disable notifications from the server, push the button on the tag board and release after 5 secs.
6. To read the battery level of the server, push the button on the tag board and release between 2-4 secs.

-------------------------------------------------------------------------------