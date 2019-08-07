-------------------------------------------------------------------------------
Beacon app
-------------------------------------------------------------------------------

Overview
--------
This app demonstrates use of Google Eddystone and Apple iBeacons via the
beacon library. It also demonstrates uses of multi-advertisement feature.

During initialization the app configures advertisement packets for Eddystone and iBeacon
and starts advertisement via multi-advertisement APIs.
It also sets up a 1 sec timer to update Eddystone TLM advertisement data.

See chip specific readme for more information about the BT SDK.

Instructions
------------
To demonstrate the app, follow these steps -

1. Build and download the application to the WICED board
2. Monitor advertisement packets using any of the methods below -
       - On Android, download  app such as 'Beacon Scanner' by Nicholas Briduox.
       - On iOS, download app such as 'Physical Web or 'Locate Beacon'.
         In the 'Locate Beacon' phone app, enter UUID for iBeacon.
         (see UUID_IBEACON in file beacon.c for UUID declaration).
       - or use over the air sniffer
3. Run BTSpy app, capture protocol and snoop traces and view the traces via viewer such as Frontline.

Application Settings
--------------------
Application specific settings are -
OTA_SEC_FW_UPGRADE
    Use this option for secure OTA firmware upgrade

Notes
-----
This application supports OTA Firmware Upgrade. To update the application,
see the OTA application note in chip specific readme.txt. Also see note at the end
of beacon.c for Bluetooth Configurator and OTA.
-------------------------------------------------------------------------------