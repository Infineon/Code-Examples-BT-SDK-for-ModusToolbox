-------------------------------------------------------------------------------
PBAP Client app
-------------------------------------------------------------------------------

Overview
--------
This app performs as a Bluetooth PBAP Client. It can connect to mobile phone that
supports PBAP Server profile and download phone book and call logs.

See chip specific readme for more information about the BT SDK.

Instructions
------------
To demonstrate the app, follow these steps -

1. Build and download the application to the WICED board
2. From the phone, initiate device discovery and find the pbap_client application
   and perform pairing.
3. From the Bluetooth profile Client Control UI, select the paired phone and click Connect
4. Use the UI to download phone book contacts, and incoming, outgoing or missed calls.
5. Run BTSpy app, capture protocol and snoop traces and view the traces via viewer such as Frontline.

Note: only the first 100 records will be displayed. The application can be updated to get
more records if desired.

Note for iPhone users: Upon pairing, iPhone will show error in esablishing connection.
This can be ignored. After establishing connection from Client Control UI, go to Settings->Blueooth
UI on iPhone, select the paired device (pbap client) and enable UI to 'Sync Contacts' before
attempting to use the Client Control buttons for accessing contacts or call logs.

-------------------------------------------------------------------------------