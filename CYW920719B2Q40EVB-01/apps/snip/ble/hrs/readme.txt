-------------------------------------------------------------------------------
Heart Rate Server (HRS) snippet application
-------------------------------------------------------------------------------

Overview
--------
The HRS snippet application shows how to initialize and use WICED BT Heart Rate
Server library.This snippet implements GAP peripheral role. On application start,
calls HRS library APIs to register callbacks for receiving HRC requests. HRS allows a client
to register/un-register for notifications and to reset Energy expended values in the server.

Once application receive BTM_ENABLED event, it loads bonded keys and enters
Limited GAP ADV mode.i.e. enters high duty GAP ADV mode for 30 seconds, after which it
switches to Low duty GAP ADV mode and remains in this mode for 30 seconds. If client(HRC)
does not attempt connection with in timeout, application exits from ADV mode.
Application includes Heart Rate Service UUID in its ADV data to get connected by HRC.
User has to push the button once to enter to Limited GAP ADV mode whenever HRC required to connect.
After connection establishes with HRC, application calls Library APIs to inform GATT connection status,
to process HRC GATT read, write requests for Heart Rate notifications.
Application is notified by library whenever HRC configures heart rate notifications.
Application stores bonded HRC address and notification configuration in NVRAM. This information is
used in reconnection, after successful encryption, to start heart rate notifications automatically.
Application sends heart rate notifications to HRC on every 1 minute, until HRC stops.

Features demonstrated
---------------------
 - Initialize and use WICED BT HRS library
 - GATTDB with Heart Rate notification service and characteristics, Device Information Service and characteristics

Instructions
------------
To demonstrate the app, work through the following steps.
1. Plug the WICED eval board into your computer
2. Build and download the application (to the WICED board)
3. Start tracing to monitor the activity (see Kit Guide for details)
4. Application automatically enter to Limited GAP ADV mode.
4. After connection with HRC, based on HRC request, start, stop Heart Rate notifications and
   resets energy expended value accumulated during Heart Rate Notification since last HRC Energy expended reset request.
5. If heart rate client not connected and HRS exited GAP ADV mode, push the application button to enter Limited GAP ADV mode.
-------------------------------------------------------------------------------