-------------------------------------------------------------------------------
Light Dimmable app
-------------------------------------------------------------------------------

Overview
--------
This demo application shows a simple implementation of a dimmable light.
The app is based on the snip/mesh/mesh_light_lightness sample which
implements BLE Mesh Light Lightness Server model. Because Light Lightness
Server model extends Generic OnOff and Generic Level, the dimmable
light can be controlled by a Switch (Generic OnOff Client), a Dimmer
(Generic Level Client), or by an application which implements Light
Lightness Client.  The WICED Mesh Models library takes care of the
translation of the OnOff and Level messages and the only messages
that the application layer needs to process is those of the Light
Lightness Model.

Features demonstrated
 - LED usage on the EVK
 - Processing of the Light Lightness messages

See chip specific readme for more information about the BT SDK.

Instructions
------------
To demonstrate the app, work through the following steps.
1. Build and download the application (to the WICED board)
2. Build and download a controlling application (to another WICED board
   using same computer/IDE or different computer). For example Mesh Dimmer.
3. Use Mesh Client or Mesh Client Control to provision a light bulb and a dimmer
4. Configure dimmer to control the light bulb.
   (note that the bulb and the dimmer were provisioned in the same group,
   the dimmer will be automatically configured to send messages to the group
   and this step can be skipped.
5. Push/release the application button on the dimmer board.  The LED on the light
   side should turn on.
6. Push/release the application button on the dimmer board.  The LED on the light
   side should turn off.
7. Push and hold the application button on the dimmer board.  The LED on the light
   side should gradually go from off to on within 4 seconds.
8. Push and hold the application button on the dimmer board.  The LED on the light
   side should gradually go from on to off within 4 seconds.
9. Try pushing and holding button for less than 4 seconds, and all other
   combinations.

Notes
-----
The application GATT database is located in -
bt_sdk-1.x\components\BT-SDK\common\libraries\mesh_app_lib\mesh_app_gatt.c
If you create a GATT database using Bluetooth Configurator, update the
GATT database in the location mentioned above.

Project Settings
----------------
Apllication specific project settings are as below -

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
