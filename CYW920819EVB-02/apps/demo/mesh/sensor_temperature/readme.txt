-------------------------------------------------------------------------------
BLE Mesh Sensor Temperature application
-------------------------------------------------------------------------------

Overview
--------
This demo application shows an implementation of a BLE Mesh temperature sensor.
The app is based on the BLE Mesh Sensor Server model.

Features demonstrated
- Temperature measurement using the on board Thermistor on the CYBT-213043-MESH EZ-BT Mesh Evaluation Kit
  / CYW920819EVB-02 Evaluation Kit
- Usage of BLE Mesh Sensor Server model

See chip specific readme for more information about the BT SDK.

Instructions
------------
To demonstrate the app, work through the following steps.
1. Build and download the application to the CYBT-213043-MESH EZ-BT Mesh Evaluation Kit
   / CYW920819EVB-02 Evaluation Kit
2. Use Android MeshController or Windows Mesh Client and provision the temperature sensor
3. After successful provisioning, use the Android MeshController/ Windows Mesh
   Client to configure the below parameters of the sensor
     a> configure sensor to publish the sensor data to a group(all-nodes, all-relays).
     b> configure publish period : publish period defines how often the user wants the
        sensor to publish the data. For testing, you can set it to 5000 msec.
     c> set cadence of the sensor (optional step):
        set minimum interval in which sensor data has to be published.
        set the range in which the fast cadence has to be observed.
        set the fast cadence period (how fast the data has to be published with respect
        to publish period).
        set the unit in which if the values change the data should be published and
        trigger type (Native or percentage).
           example : publish data if the data changes by 2 units/10%
4. Observe the temperature value on the Android app / trace window in Mesh Client app.
5. To change the temperature on the thermistor, you can keep your finger on the onboard
   sensor and see the changes.

Notes
-----
The application GATT database is located in -
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
