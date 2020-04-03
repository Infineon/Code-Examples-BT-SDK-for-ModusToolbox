

-------------------------------------------------------------------------------
Alert Notification Client(ANC) Snippet Application
-------------------------------------------------------------------------------

 The ANC snippet application shows how to initialize and use WICED Alert Notification Client Library.
 ANC is implemented in GAP peripheral role and configures Alert Notification Server(ANS)
 to receive Alerts based on user requests using ClientControl(CC) Alert GUI.

 On application init, ANC starts advertisements. The advertisement data would include "ANC".
 The ANS would scan for this name and connects itself automatically when found.
 Once connection is established ANC UI would be enabled.
 User has to perform the following steps after connection:
 1) To get to know the Server Supported New Alerts and Unread Alerts using New Alert and Unread Alert radio buttons,
    Read Alerts button has to be clicked. Once this operation is completed successfully,
    ANC GUI shows server supported New Alert and Unread Alert categories.
    NOTE: For now, The App is designed to support Simple ALert, SMS/MMS and Email.The remaining ALert types will not be highlighted.
 2) After ANC knows the Server Supported Alerts, ANC should configure for Alert notifications as mentioned below:
    2.1 -- Irrespective of Alert type, all New/Unread Alerts should be enabled by using EnableNewAlerts and EnableUnreadAlerts buttons.
    2.2 -- Using Control Alerts Button User can selectively Enable/disable Alerts or "Notify immediately all the pending Alerts".
           This can be done based on selection of function from drop down which is present right above the Control Alerts button.
           In this use case the functionality for New Alerts or Unread Alerts is based on which Radio Button is enabled.
 3) Once configured for Alert Notifications as mentioned above, Alerts generated from Server(i.e ANS) can be monitored through traces.

See chip specific readme for more information about the BT SDK.

-------------------------------------------------------------------------------