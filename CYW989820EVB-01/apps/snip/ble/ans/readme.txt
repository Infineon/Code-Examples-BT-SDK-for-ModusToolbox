-------------------------------------------------------------------------------
Alert Notification Server (ANS) snippet application
-------------------------------------------------------------------------------

Overview
--------
The ANS snippet application shows how to initialize and use WICED BT Alert
Notification Server library. ANS implemented in GAP central mode and provides ANS services
to ANC using WICED BT ANS library.

On ClientControl Connect, ANS automatically does Scan and connect to Alert Notification Client (ANC),if
ANC is advertising with in the range and ANC includes its name (i.e "ANC") in its ADV.
if ANC does not found with in 90 seconds, Scan stopped automatically. User has to push application
button to restart the scan to connect to ANC whenever ANC ready for connection.
User need to use ClientControl ANS GUI option to generate the alerts and control the alerts.
Application currently support simple alerts, email and SMS/MMS alert categories and notifies this
information to ClientControl whenever transport connected.
User can inturn enable subset of supported new alerts and unread alerts when ANS not connected to ANC.
After connection to ANC, ANS sends new alerts and unread alerts to ANC based on ANC configuration.
Below explains how new alerts and unread alerts get generated.
-  ClientControl enables the GUI option to generate the alert when ANS in connection with ANC.
-  When user Generate the Alert using GUI, ANC receives the new alert and unread alert.
-  User can clear alert count using GUI Clear Alert button.
Application calls ANS library APIS based on user requests (i.e. on updating supported alert categories,
generate and clear the alert requests).
Read/Write requests received from ANC passed to WICED BT ANS library to update the ANC
configuration (for example to start or stop new alerts/unread alerts and to configure to send
only requested alert categories.)

To test this snippet app use Bluetooth Profile Client Control application.

See chip specific readme for more information about the BT SDK.

Features demonstrated
---------------------
 - Initialize and use WICED BT ANS library
 - GATTDB with Alert notification service and characteristics

Instructions
------------
To demonstrate the app, work through the following steps.
1.Plug the WICED eval board into your computer
2.Build and download the application (to the WICED board)
3.Start tracing to monitor the activity (see Kit Guide for details)
4 Use ClientControl to update supported alerts.
5 Pair with ANC.
6.Send alerts to ANC using ClientControl.

-------------------------------------------------------------------------------