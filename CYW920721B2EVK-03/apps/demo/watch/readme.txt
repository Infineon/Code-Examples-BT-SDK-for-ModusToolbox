-------------------------------------------------------------------------------
Watch app
-------------------------------------------------------------------------------

Overview
--------
This app demonstrates Bluetooth A2DP source, AVRCP Controller/Target, Apple Media Service (AMS) and
Apple Notification Center Service (ANCS).
Features demonstrated
 - WICED BT A2DP Source APIs
 - WICED BT AVRCP (Controller/Target) APIs
 - WICED BT GATT APIs
 - Apple Media Service and Apple Notification Client Services (AMS and ANCS)
 - Handling of the UART WICED protocol
 - SDP and GATT descriptor/attribute configuration

See chip specific readme for more information about the BT SDK.

Instructions
------------
To demonstrate the app, follow these steps -

1. Build and download the application to the WICED board.
2. Open the BT/BLE Profile Client Control application and open the port for WICED HCI for the device.
   Default baud rate configured in the application is 3M.
3. Use Client Control application to send various commands mentioned below.
4. Run the BTSpy program to view protocol and application traces.

See "BT/BLE Profile Client Control" and "BT Spy" in chip-specifc readme.txt for more information about these apps.

BR/EDR Audio Source and AVRC Target:
- The Watch app can demonstrate how use to BR/EDR Audio Source and AVRC TG profiles.
- Use buttons in AV Source tab.
- To play sine wave sample, set the audio frequency to desired value (48kHz, 44.1kHz, etc.)
  and select the Media type as 'Sine Wave' in UI. In this case, built-in sine wave audio is played.
- To play music from .wav file, select the Media type as File, browse and select a .wav file.
  In this case, audio for .wav file is routed over WICED HCI UART to the WICED board.
- Put an audio sink device such as BT headphone/speaker in pairable mode.
- Click on "Start" button for "BR/EDR Discovery" combo box to find the audio sink device.
- Select the peer device in the BR/EDR Discovery combo box.
- Click "Connect" button under AV Source tab.
- Click "Start Streaming" button. Music will start playing on peer device.
- The watch app uses AVRCP Target role. Once connected to headset/speaker,
  the app can send notifications for play status change (Play, Pause, Stop) and
  setting change (Repeat, Shuffle) to peer AVRCP controller (such as headset/speaker).
  Note: the songs shown in the AVRC TG UI and some settings such Repeat/Shuffle are for testing
  AVRC commands only, do not indicate the actual media played and will not change the media played.

BR/EDR AVRCP Controller:
- The Watch app can demonstrate how use to AVRC CT profile.
- Disconnect all devices if any connected.
- Make an audio source device such as iPhone discoverable/pairable from Bluetooth Settings UI on phone.
- Using "BR/EDR Discovery" "Start" button, search and select the device.
- Use buttons in AVRC CT tab. Click Connect button and accept pairing.
- Play music on audio source device and control the music via buttons in AVRC CT tab.
- In Controller mode, pass-thru commands are executed via Play, Pause, Stop, etc. buttons.
- Absolute volume change can be done via the drop down Volume or Vol Up/Down buttons.
- Note that iPhone does does not support Vol buttons.
- Note that music will continue to play on audio source device.

iOS ANCS and AMS GATT Services:
- The Watch app can demonstrate how to use AMS and ANCS iOS services as below.
- Disconnect all devices if any connected.
- Select Pairable if it not checked.
- Click on the "Start Adverts" button in GATT tab.
- From the iPhone app such as 'LightBlue', find and connect to 'Watch' app.
- Allow pairing with the iPhone.
- AMS:
  - Play media on the iPhone.
  - Use buttons in AVRC CT tab to control the music.
  - Note that music will continue to play on iPhone.
- ANCS:
  - Incoming calls and messages to the iPhone will be displayed on the ANCS buttons.
  - Make an incoming call to your iPhone. See notification displayed on UI to accept
    or reject the call. Send SMS to your iPhone to see notification. Similarly missed
    call notifications are seen.

BLE Client:
- The Watch app can demonstrate BLE Client functionality as below.
- Make sure there is a BT device with GATT services that is advertising. For example use app
  such as 'LightBlue' on your phone and create a 'Virtual Peripheral' such as 'Blood Pressure'
- To find GATT devices: Click on the "Start" button for "BLE Discovery" combo box.
  Click on "Stop" button to end discovery.
- To connect BLE device: Choose device from the "BLE Discovery" drop down combo box and
  click "Connect" button.
- To discover services: Click on the "Discover Services" button
- To discover characteristics: Enter the handles in the edit box and click
  on "Discover Characteristics"
- To discover descriptors: Enter the handles in the edit box and click on
  "Discover Characteristics"
- Enter the Handle and Hex Value to write to the remote device using buttons
   "Write" : Write hex value to remote handle
   "Write no rsp" : Write hex value without response to remote handle
   "Value Notify" : Write notification value to the remote handle
   "Value Indicate" : Write indication value to the remote handle

Project Settings
----------------
Application specific project settings are as below -

SLEEP_SUPPORTED
    This option allows device to enter low power mode.

COEX_SUPPORTED
    This option enables BT and Wi-Fi Coexistence.
-------------------------------------------------------------------------------