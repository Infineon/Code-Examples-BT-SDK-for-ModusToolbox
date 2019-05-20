-------------------------------------------------------------------------------
BLE Mesh LightDimmable application
-------------------------------------------------------------------------------

Overview
--------
This demo application shows a simple implementation of a dimmable light.
The app is based on the BLE Mesh Light Lightness Server model. Because Light Lightness
Server model extends Generic OnOff and Generic Level, the dimmable
light can be controlled by a Switch (Generic OnOff Client), a Dimmer
(Generic Level Client), or by an application which implements Light
Lightness Client.  The WICED Mesh Models library takes care of the
translation of the OnOff and Level messages and the only messages
that the application layer needs to process is those of the Light
Lightness Model.

Features demonstrated
 - LED usage on the CYBT-213043-MESH EZ-BT Mesh Evaluation Kit / CYW920819EVB-02 Evaluation Kit
 - Processing of the Light Lightness messages

See chip specific readme for more information about the BT SDK.

Instructions
------------
To demonstrate the app, work through the following steps.
1. Build and download the application to the Mesh Evaluation Kit / CYW920819EVB-02 Evaluation Kit
2. Build and download a controlling application such as BLE Mesh Dimmer to another Mesh Evaluation Kit
   / CYW920819EVB-02 Evaluation Kit
3. Use Mesh Client or Mesh Client Control to provision the LightDimmable node as light bulb and the
   Dimmer node as dimmer. (Note: Mesh Client Control requires an additional board that must be
   programmed with provision client app. Please refer to Mesh Client and Mesh Client Control user guides
   to learn about these helper applications. User guide is available in ModusToolbox
   documentation. Also, this user guide is available under documentation tab at www.cypress.com/ble-mesh.)
4. Configure dimmer to control the light bulb.
   (note that if the light bulb and the dimmer were provisioned in the same group,
   the dimmer will be automatically configured to send messages to the group
   and this step can be skipped.)
5. Push/release the  user button (SW3) on the dimmer board.  The LED (LED1 on Mesh Evaluation Kit
   / LED 2 on CYW920819EVB-02 Evaluation Kit) on the light bulb side should turn on.
6. Push/release the user button (SW3) on the dimmer board.  The LED on the light bulb
   side should turn off.
7. Push and hold the user button (SW3) on the dimmer board.  The LED on the light bulb
   side should gradually go from Off to On within 4 seconds.
8. Push and hold the user button (SW3) on the dimmer board.  The LED on the light bulb
   side should gradually go from On to Off within 4 seconds.
9. Try pushing and holding button for less than 4 seconds, and all other
   combinations.

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
