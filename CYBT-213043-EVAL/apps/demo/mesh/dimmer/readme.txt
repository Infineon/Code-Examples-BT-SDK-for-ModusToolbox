-------------------------------------------------------------------------------
BLE Mesh Dimmer demo application
-------------------------------------------------------------------------------


Overview
--------
This demo application shows a simple dimmer implementation using CYBT-213043-MESH EZ-BT Mesh Evaluation Kit
/ CYW920819EVB-02 Evaluation Kit.
The app implements BLE Mesh Generic Level Client model.
Normally a dimmer has at least 2 buttons to turn the light on and off.
This application performs dimming using a single user button available on the
kits.  On a short push, the level is toggled between 0% and 100%.
When a button is pushed and not released, the level is changed
every 0.5 seconds from 0 to 100 or vice versa in 8 steps of 12.5% each based on the previous LED state.
The button can be released before the level reaches 100% and in this case
consecutive push or long push will continue increasing the level.
If level reaches 100% the next button control will decrease the level.

Features demonstrated
 - Button usage on the CYBT-213043-MESH EZ-BT Mesh Evaluation Kit / CYW920819EVB-02 Evaluation Kit
 - Controlling of a BLE Mesh light bulb using BLE Mesh Set Level messages

See 20819_readme.txt for more information about the BT SDK.

Instructions
------------
To demonstrate the app, work through the following steps.
1. Build and download the application to the Mesh Evaluation Board / CYW920819EVB-02 Evaluation Kit
2. Build and download Mesh_LightDimmable application (Mesh Evaluation Board / CYW920819EVB-02 Evaluation Kit)
3. Use Mesh Client or Mesh Client Control to provision the LightDimmable node as light bulb and the
   Dimmer node as dimmer. (Note: Mesh Client Control requires an additional board that must be programmed
   with provision client app. Please refer to Mesh Client and Mesh Client Control
   user guides to learn about these helper applications. User guide is available in ModusToolbox
   documentation. Also, this user guide is available under documentation
   tab at www.cypress.com/ble-mesh.)
4. Configure dimmer to control the light bulb.
   (note that if the light bulb and the dimmer were provisioned in the same group,
   the dimmer will be automatically configured to send messages to the group
   and this step can be skipped.)
5. Push/release the user button (SW3) on the dimmer board.  The LED (LED1 on Mesh Evaluation Kit
   / LED 2 on CYW920819EVB-02 Evaluation Kit) on the light bulb   side should turn on.
6. Push/release the user button (SW3) on the dimmer board.  The LED on the light bulb
   side should turn off.
7. Push and hold the user button (SW3) on the dimmer board.  The LED on the light bulb
   side should gradually go from Off to On within 4 seconds.
8. Push and hold the application button on the dimmer board.  The LED on the light bulb
   side should gradually go from On to Off within 4 seconds.
9. Try pushing and holding button for less than 4 seconds, and all other
   combinations.

Details
-------
By default application does not support Relay, Proxy or Friend features
It can also be compiled to support a Low Power Node by adding
#define LOW_POWER_NODE  1

Notes
-----
1. The board will factory reset if you press and hold the user button (SW3) on
   the board for more than 15 seconds.
2. The application GATT database is located in -
   bt_sdk-1.x\components\BT-SDK\common\libraries\mesh_app_lib\mesh_app_gatt.c
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
