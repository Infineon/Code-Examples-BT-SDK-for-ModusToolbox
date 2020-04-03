-------------------------------------------------------------------------------
Heart Rate Client (HRC) snippet application
-------------------------------------------------------------------------------

Overview
--------
The HRC snippet application shows how to initialize and use WICED BT Heart Rate
Client library. This snippet implements GAP central role

On initialization, the application starts scanning and connects to a nearby peripheral that advertises
Heart Rate Service UUID. After connecting successfully, application calls HRC library
to start GATT discovery for HRS characteristics and descriptors. The library then issues
callbacks to notify status of the discovery operation. On successful discovery,
application configures HRS to send heart rate notifications. User can use application button
to un-resgister and re-register to notifications and to reset energy expended value.
Application tracks the duration of button pressed using a timer and performs the actions as explained below.

Features demonstrated
---------------------
 - Initialize and use WICED BT HRC library

Instructions
------------
To demonstrate the app, work through the following steps.
1. Plug the WICED eval board into your computer
2. Build and download the application (to the WICED board)
3. On start of the application, push the button on the tag board and release with in 2 seconds, so that
   Heart Rate Client scans and connects to the Heart Rate Server which would have
   UUID_SERVICE_HEART_RATE in it's advertisements.
   Note:- If no Heart Rate server device is found nearby for 90secs, then scan stops automatically.
   To restart the scan, push the button on the tag board and release within 2 secs.
4. Once connection established with BLE peripheral and heart rate service found in BLE peripheral,
   automatically application receive heart rate notification from server on every 1 minute.
5. To start or stop notifications, user should press the button and release between 2 to 4 seconds.
6. During notifications if user finds energy expended value reched maximum value, i.e. 0xffff and want to reset
   the value, user should press the application button and release after 5 seconds.
-------------------------------------------------------------------------------