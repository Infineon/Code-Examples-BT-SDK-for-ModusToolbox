-------------------------------------------------------------------------------
Mesh Low Power Led Application
-------------------------------------------------------------------------------

Overview
--------
This demo application shows an implementation of a low_power_led system.
The app is based on the snip/mesh/mesh_power_onoff_server sample which implements generic BLE Mesh Power Onoff Server model.

Features demonstrated
showcase a LPN + Server as well as a Friend node implementation in conjunction with the Proxy/Relay + Server (3* "light bulb").

See chip specific readme for more information about the BT SDK.

Instructions
------------
To demonstrate the app, follow these steps -

1. Build and download the application (to the WICED board)
   1) For low_power_led lpn system node, add LOW_POWER_NODE=1 in Make Target
   2) For lighting element, add LOW_POWER_NODE=0 in Make Target
2. Use Android MeshController or Windows Mesh Client to provision all boards.
3. After successful provisioning, the low_power_led lpn node will choose one lighting node as its friend automatically.
   User can use the Android MeshController/ Windows Mesh Client to light on/off all the nodes:
   1) For lighting nodes, on/off the them directly and they will reflect the result immediately;
   2) For low_power_led nodes, it can't receive on/off command directly, it will get command from friend node.
      So it needs to create friendship before sleep;
   3) When low_power_led node is sleeping, send on/off command to it using Android MeshController or Windows Mesh Client,
      the lighting nodes will relay(if distance is far) and cache it(friend node do this). When the low_power_led node
      wakes up from sleep, it will poll the friend and then friend node will send the cached on/off command to it.
   4) When low_power_led node is in sleep mode, it can maintain its gpio state. You can choose maintain or close it.
      When low_power_led node comes back from sleep mode, it can restore the led state.

Application Settings
--------------------
Application specific settings are -

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