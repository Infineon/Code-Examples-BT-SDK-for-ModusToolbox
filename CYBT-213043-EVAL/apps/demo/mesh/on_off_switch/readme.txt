-------------------------------------------------------------------------------
BLE Mesh OnOff Switch application
-------------------------------------------------------------------------------

Overview
--------
This demo application shows an OnOff switch implementation using CYBT-213043-MESH EZ-BT Mesh Evaluation Kit
/ CYW920819EVB-02 Evaluation Kit.
The app is based on the BLE Mesh Generic OnOff Client model.
Normally a switch has 2 buttons to turn the light (or any other device) On and Off.
This application performs On and Off functionality using a single user button
available on the kits. Based on current state, an On or Off command is sent.
For example, if current LED state is Off, on first button push the On command is sent.
On the following push the Off command is sent.

By default application does not support Relay, Proxy or Friend features
The application can be compiled to support a Low Power Node feature by adding
#define LOW_POWER_NODE 1

Features demonstrated
 - Button usage on the CYBT-213043-MESH EZ-BT Mesh Evaluation Kit / CYW920819EVB-02 Evaluation Kit
 - Controlling of a BLE Mesh light bulb using BLE Mesh On/Off messages

See chip specific readme for more information about the BT SDK.

Instructions
------------
To demonstrate the app, work through the following steps.
1. Build and download the application to the Mesh Evaluation Kit / CYW920819EVB-02 Evaluation Kit
2. Build and download BLE Mesh LightDimmable application to another Mesh Evaluation Kit
   / CYW920819EVB-02 Evaluation Kit
3. Use Mesh Client or Mesh Client Control to provision the LightDimmable node as
   light bulb and the OnOffSwitch node as on_off_switch.
   (Note: Mesh Client Control requires an additional board that must be programmed with provision client
   app. Please refer to Mesh Client and Mesh Client Control user guides to learn about
   these helper applications. User guide is available in ModusToolbox
   documentation. Also, this user guide
   is available under documentation tab at www.cypress.com/ble-mesh.)
4. Configure on_off_switch to control the light bulb by configuring publication.
   (note that if the bulb and the on_off_switch were provisioned in the same group,
   the on_off_switch will be automatically configured to send messages to the group
   and this step can be skipped.)
5. Push/release the user button (SW3) on the on_off_switch board.  The LED (LED1 on Mesh Evaluation Kit
   / LED 2 on CYW920819EVB-02 Evaluation Kit) on the light side should turn on.
6. Push/release the user button (SW3) on the on_off_switch board.  The LED on the light
   side should turn off.

Notes
-----
1. The board will factory reset if you press and hold the user button on
   the board for more than 15 seconds.
2. The application GATT database is located in -
   bt_sdk-1.1\components\BT-SDK\common\libraries\mesh_app_lib\mesh_app_gatt.c
   If you create a GATT database using Bluetooth Configurator, update the
   GATT database in the location mentioned above.

Project Settings
----------------
Application specific project settings are as below -

MESH_MODELS_DEBUG_TRACES
    Turn on debug trace from Mesh Models library
MESH_CORE_DEBUG_TRACES
    Turn on debug trace from Mesh Core library
MESH_PROVISIONER_DEBUG_TRACES
    Turn on debug trace from Mesh Provisioner library
REMOTE_PROVISION_SRV
    Enable device as Remote Provisioning Server
LOW_POWER_NODE
    Enable device as Low Power Node

-------------------------------------------------------------------------------
