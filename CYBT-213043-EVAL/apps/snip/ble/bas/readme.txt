-------------------------------------------------------------------------------
BLE Battery Service Server Sample Application
-------------------------------------------------------------------------------

Overview
--------
Battery Service implementation. For details refer to BT SIG Battery Service Profile 1.0 spec.

On startup this demo:
 - Initializes the Battery Service GATT database
 - Begins advertising
 - Waits for GATT clients to connect

Instructions
------------
To test the app, work through the following steps.
1. Plug the WICED eval board into your computer
2. Build and download the application (to the WICED board)
3. On application start the device acts as a GATT server and advertises itself as Battery Service.
4. Connect to Battery Service using one of the LE clients (LEExplorer(android)) or (BLE Utility(Apple Store))
   or battery_service_client application.
5. Once connected the client can read Battery levels.
6. If the client enables the notification, the Battery Server will send battery level every 30secs.
7. Battery Level starts from 100 and keeps decrementing to 0. The Battery Server App is designed such that,
   once it hits 0, it will be rolled back to 100.

-------------------------------------------------------------------------------