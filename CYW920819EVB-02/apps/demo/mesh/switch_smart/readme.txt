-------------------------------------------------------------------------------
Sensor Motion app
-------------------------------------------------------------------------------

Overview
--------
This demo application shows a  implementation of a motion sensor combined.
with the on off button functionality.  The app is based on the
snip/mesh/mesh_sensor_server sample which implements BLE Mesh Sensor
Server model and snip/mesh/mesh_onoff_server sample which implements BLE Mesh
Generic OnOff Server model.  The app is designed to work in concert with a
Light Controller app.  The Switch will trigger Light Controller to go on
when motion is detected.  When user pushes a button, the Switch will send
On command if the Light is Off, and Off command if the light is On.

Features demonstrated
 - Configuring and receiving interrupts from the PIR motion sensor
 - Configuring and receiving interrupts from the button
 - Publishing motion data over BLE mesh
 - Publishing on/off commands over BLE mesh

See 20819_readme.txt for more information about the BT SDK.

Instructions
------------
To demonstrate the app, work through the following steps.
1. Build and download the application (to the WICED board)
2. Use Android MeshController or Windows Mesh Client and provision the Smart Switch
3. Use Android MeshController or Windows Mesh Client and provision the Smart Light
4. After successful provisioning, user can use the Android MeshController/ Windows Mesh
   Client to configure the below parameters of the sensor
        Configure Smart Light to publish OnOff Status to the Smart Switch (or to the group)
        Configure Smart Switch to publish OnOff to the group
        Configure Smart Switch to publish Sensor Data to the group
        When sensor is provisioned, it is configured to publish data to a group, if
        current group was configured, or to "all-nodes", if there were no group.
        Modify publication to configuration to publish to "all-nodes".  Also set
        up the sensor to publish data with period 320000msec (5 minutes). The default
        configuration of the sensor is to publish data with publish period when
        presence is not detected. When presence is detected the period is divided by
        30. A message will be sent as soon as motion is detected and every 10 second
        while presence is being detected.
5. Wave your hand in front of the CYBT-213043-MESH board to show some motion.

Notes
-----
The application GATT database is located in -
bt_20819A1-1.0\components\BT-SDK\common\libraries\mesh_app_lib\mesh_app_gatt.c
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