------------------------------------------------------------------------------------
BT SDK - CYW20719
------------------------------------------------------------------------------------

Overview
--------
The Cypress CYW20719 is an ultra-low-power dual-mode Bluetooth 5.0 wireless
MCU device. It has a stand-alone baseband processor with an integrated 2.4 GHz
transceiver supporting BR/EDR/BLE.

SDK Software Features
----------------------
- Dual mode Bluetooth stack included the ROM (BR/EDR and BLE).
- BT stack and profile level APIs for embedded BT application development.
- WICED HCI protocol to simplify host/MCU application development.
- APIs and drivers to access on board peripherals
- Bluetooth protocols include GAP, GATT, SMP, RFCOMM, SDP, AVDT/AVCT, BLE Mesh
- BLE and BR/EDR profile APIs, libraries and sample apps
- Support for Over-The-Air (OTA) upgrade.
- Device Configurator for creating custom pin mapping.
- Bluetooth Configurator for creating BLE GATT Database.
- Documentation for APIs, datasheet, profiles and features.

Kits
----
CYW920719B2Q40EVB-01:
    40-QFN package, Arduino compatible headers, 9-axis motion sensor and thermistor,
    user switches and LEDs, USB connector for power, programming and USB-UART bridge.
    Note: Max UART baud rate is 3M

Software Tools
--------------
Following applications are installed with ModusToolbox on your computer.

BT Spy :
    BTSpy is a trace viewer utility that can be used in the WICED BT platforms to
    view protocol and application trace messages from the embedded device. The
    utility is located in folder below. For more information, see readme.txt
    in the same folder. (This utility can also be run from IDE Launches menu).
    It is supported on Windows, Linux and macOS.
    <Install Dir>\ModusToolbox_1.1\tools\wiced-tools-1.0\BT\BTSpy

BT/BLE Profile Client Control:
    This application emulates the host MCU applications for BLE and BR/EDR profile.
    It demonstrates WICED BT APIs. It application communicates with embedded apps
    over the WICED HCI interface. The application is located in folder below.
    See readme.txt in the same folder. (This utility can also be run from IDE Launches menu).
    It is supported on Windows, Linux and macOS.
    <Install Dir>\ModusToolbox_1.1\libraries\bt_sdk-1.x\components\BT-SDK\common\client_control

BLE Mesh Client Control:
    Similar to the above app, this application emulates the host MCU applications
    for BLE Mesh models. It can configure and provision mesh devices and create mesh
    network. The application is located in folder below. (Currently for Windows OS only).
    <Install Dir>\ModusToolbox_1.1\libraries\bt_sdk-1.x\components\BT-SDK\common\apps\snip\mesh\ClientControl
    See readme.txt in the same folder.

Peer apps:
    Application that run on Windows, iOS or Android and act as peer
    BT apps to demonstrate specific profiles or features.
    BT/BLE apps location -
    <Install Dir>\ModusToolbox_1.1\libraries\bt_sdk-1.x\components\BT-SDK\common\peer_apps
    BLE Mesh apps location -
    <Install Dir>\ModusToolbox_1.1\libraries\bt_sdk-1.x\components\BT-SDK\common\apps\snip\mesh\peerapps

Device Configurator:
    Use this tool to create custom pin mapping for your device. Run this tool from ModusToolbox IDE
    "Configure Device" menu. It is supported on Windows, Linux and macOS.
    Note: The pin mapping is based on wiced_platform.h for your board.

Bluetooth Configurator:
    Use this application to create and configure BLE GATT Database for your application.
    Run this tool from ModusToolbox IDE "Configure Device" menu -> Peripherals -> Bluetooth
    -> External tools. It is supported on Windows, Linux and macOS.

Tracing
-------
To view application traces, there are 2 methods available. Note that the application
needs to configure the tracing options.
1. WICED Peripheral UART - Open this port on your computer using serial port utility
such as Tera Term or PuTTY (usually baud rate of 115200).
2. WICED HCI UART - Open this port on your computer using Client Control application
mentioned above (usually baud rate of 3M). Then run BT Spy utility mentioned above.

Application Settings
--------------------
Application settings can be changed via 'Change Application Settings...' menu (right click on
application in IDE). Options below are available for all applications. Other application
specific options might also be available and are documented in the readme.txt for those
applications.

BT_DEVICE_ADDRESS
    Set BT device address for your BT device. The BT address is 6 bytes,
    for example 20719B20FFEE. By default, the SDK will set random BDA for your device.
UART
    Select the UART port you want the application to be downloaded. For example 'COM6'
    on Windows or '/dev/ttyWICED_HCI_UART0' on Linux or '/dev/tty.usbserial-000154' on macOS.
    By default, the SDK will auto detect the port.
ENABLE_DEBUG
    For HW debugging, select the option '1'. See the document WICED-Hardware-Debugging.pdf
    for more information. This setting configures GPIO for SWD.
    CYW920719B2Q40EVB-01: PUART RX/TX signals are shared with SWDCK and SWDIO. Remove RX and TX
    jumpers on J10 when using SWD. PUART and SWD cannot be used simultaneously on this board
    unless these pins are changed from the default configuration.

Downloading application to kit
------------------------------
If you have issues downloading to the kit, follow the steps below -
- Press and hold the 'Recover' button on the kit.
- Press and hold the 'Reset' button on the kit.
- Release the 'Reset' button.
- After one second, release the 'Recover' button.

After downloading the application, press the 'Reset' button on the kit.

Over The Air (OTA) Firmware Upgrade
-----------------------------------
Application that support OTA upgrade can be updated via peer OTA apps located in the folder -
<Install Dir>\ModusToolbox_1.1\libraries\bt_sdk-1.x\components\BT-SDK\common\peer_apps\ota_firmware_upgrade
See the readme.txt file located in the above folder for instructions.
To generate OTA image for the app, append command line OTA_FW_UPGRADE=1 to the build, for example
> make PLATFORM=CYW920719B2Q40EVB-01 OTA_FW_UPGRADE=1
This will generate <app>.bin file in the 'build' folder.


------------------------------------------------------------------------------------
