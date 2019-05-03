# Code-Examples-BT-20819A1-1.0-for-ModusToolbox

## Overview

These examples demonstrate the functionality of the CYW920819 device. Apps in the 'demo' folder are reference applications and more complex in nature. Apps in the 'snips' folder demonstrate a specific feature or library. These examples support ModusToolbox v1.1. They do not work with ModusToolbox v1.0. 

Each example has a readme.txt that explains what do and what to observe when running the example. Also review the 20819_readme.txt for an overvivew of the BT-20819A1 SDK features and tools. 

Supported kits for the CYW920819 device are CYW920819EVB-02 (http://www.cypress.com/CYW920819EVB-02) and CYBT-213043-MESH (http://www.cypress.com/CYBT-213043-MESH).

Additional PSoC 6-related code examples that leverage connectivity are available in other repos. See all examples at [Code Examples for Modus Toolbox](https://github.com/cypresssemiconductorco/Code-Examples-for-ModusToolbox-Software).

You can browse the code examples in the repository, or use GitHub's search capabilities to find a particular symbol or term. Type the term into the search box at the top of this page and search in this repository.

## Adding the Code Example to the IDE

Download and unzip this repository onto your local machine. You can also clone the repository into a location on your local machine. This puts all the code examples on your local machine.

In the ModusToolbox IDE, click the **New Application** link in the Quick Panel (or, use **File > New > ModusToolbox IDE Application**). Pick your kit (or a custom board). You must use a kit or device supported by the code example.

In the **Starter Application** window, click the **Import** button and navigate to the *modus.mk* file for the example. Click **Next** and complete the application creation process.

## BT-20819A1 Code Examples

The  *CYBT-213043-MESH/apps* folder contains demos and snips set up for the CYBT-213403-MESH platform. The same mesh examples exist in the CYW920819EVB-02 folder, for that platform. 

The *CYW920819EVB-02/apps* folder contains additional examples. Use the examples in the CYW920819EVB-02 folder if you have that platform.

| Application | Description | Kits Supported | Installed w/ ModusToolbox 1.1 |
| ----- | ----- | ----- | ----- |
| demo/mesh/on_off_switch|Application shows an OnOff switch implementation using BLE Mesh libraries. |CYBT-213043-MESH, CYW920819EVB-02 |Yes |
| demo/mesh/light_dimmable|Application shows a simple implementation of a dimmable light using BLE Mesh libraries. |CYBT-213043-MESH, CYW920819EVB-02 |Yes |
| demo/mesh/dimmer|Application shows a simple dimmer implementation using BLE Mesh libraries. |CYBT-213043-MESH, CYW920819EVB-02 | Yes |
| demo/mesh/sensor_temperature|Application shows an implementation of a BLE Mesh temperature sensor.| CYBT-213043-MESH, CYW920819EVB-02 |Yes |
| snip/mesh/|Applications in this folder demonstrate use of BLE Mesh library to implement various SIG Mesh models such as power on/off, level, battery, light control, transition time, location, property, time, scene, scheduler, vendor, provision and sensor (client & server).|CYBT-213043-MESH, CYW920819EVB-0 | No |
| **CYW920819EVB-02/apps/demo**|*These application are provided as reference application for BT 20819 SDK*| | |
| BLE_FindMe|Application implements a BLE Find Me Profile that consists of an Immediate Alert Service.|CYW920819EVB-02|No
| beacon|Application demonstrates use of Google Eddystone and Apple iBeacons via the beacon library and multi-advertisement feature.|CYW920819EVB-02|Yes
| env_sensing_temp|Application implements BLE Environmental Sensing Service with temperature characteristics.|CYW920819EVB-02|Yes
| watch|Application demonstrates BR/EDR Audio Source and AV Remote Control Profiles, BLE Apple Media Service (AMS) and Apple Notification Center Services (ANCS), WICED BT GATT APIs, handling of the WICED HCI protocol and SDP configuration.|CYW920819EVB-02|Yes
| empty_wiced_bt|Application is provided as a template application for CYW20819 family of devices, a starting point for adding new code and functionality.|CYW920819EVB-02|Yes
| hello_sensor|Application demonstrates BLE Vendor Specific Server Device that is is designed to connect Hello Client device and receive notifications.|CYW920819EVB-02|No
| hello_client|Application demonstrates BLE Vendor Specific Client Device that is is designed to connect and access services of the Hello Sensor device.|CYW920819EVB-02|No
| hid/ble_keyboard|Application provides a turnkey BLE keyboard solution using on-chip keyscan HW component and is compliant with HID over GATT Profile (HOGP)|CYW920819EVB-02|No
| hid/ble_mouse|Application provides a turnkey BLE mouse solution compliant with HID over GATT Profile (HOGP)|CYW920819EVB-02|No
| hid/ble_remote|Application provides a turnkey BLE remote control solution compliant with HID over GATT Profile (HOGP)|CYW920819EVB-02|No
| throughput_test|Application demonstrates how to measure and achieve better throughput via GATT notifications|CYW920819EVB-02|No
| datalogger/dual_spi_master|Application demonstrates two SPI master instances each running in a separate thread on a CYW20819 device.|CYW920819EVB-02|No
| datalogger/spi_slave_sensor|Application demonstrates how to use the SPI driver interface to send and receive bytes or a stream of bytes over the SPI hardware as a slave|CYW920819EVB-02|No
| low_power_20819|Application demonstrates low power modes (ePDS and HID-Off) in CYW20819 device.|CYW920819EVB-02|No
| **CYW920819EVB-02/apps/snip/**|*These application demonstrate use of specific features or libraries of BT 20819 SDK*||
| ble/bac & bas|Applications implement BLE Battery Service Client and Server profiles.|CYW920819EVB-02|No
| ble/anc & ans|Applications implement BLE Alert Notification Client and Server profiles.|CYW920819EVB-02|No
| ble/hrc & hrs|Applications implement BLE Heart Service Client and Server profiles.|CYW920819EVB-02|No
| ble/le_coc|Application demonstrate BT LE L2CAP APIs for Connection Oriented Channels.|CYW920819EVB-02|No
| hal/adc, gpoi, puart, pwm|Applications that demonstrate use of WICED HAL APIs to use peripherals - ADC, GPIO, PUART, PWM|CYW920819EVB-02|No
| ota_firmware_upgrade|Application demonstrates how to implement Over The Air Firmware Upgrade.|CYW920819EVB-02|No
