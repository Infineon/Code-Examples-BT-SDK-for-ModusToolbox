----------------------------------------------------------------------------------------------------------
CE226301: Throughput Application
----------------------------------------------------------------------------------------------------------

Overview
--------
This application demonstrates how to measure and achieve better throughput via GATT
notifications on a CYW920819EVB-02 kit. It sends GATT notifications every
millisecond and calculates the throughput. The GATT notifications are sent only
when notifications are enabled.

This app also provides a option to switch between 1M and 2M PHY via user button1.
On power up, the CYW920819EVB-02 comes up in 1M PHY mode. Once the BLE connection
established, a peer with 2M PHY support may request the device to switch to the 2M PHY.
If the peer allows both 1M and 2M PHY, then on every press of user button SW3, the
CYW920819EVB-02 switches between 1M and 2M PHY alternately.

LED2:
 LED2 is ON when a GAP connection is established; OFF otherwise.

LED1:
 LED1 indicates active data transmission.
 It will be OFF when there is congestion or when data transfer is disabled.
 It will be ON when the device is actively transferring the data.

Connection Interval:
 As per this app, the CYW920819EVB-02 will request a BLE connection interval
 value of 26.25ms after the connection has established. The peer may accept or
 reject this value and may select a different connection interval.

 See 20819_readme.txt for more information about the BT SDK.

Instructions
------------
To demonstrate this code example, work through the following steps.

1. Plug the CYW920819EVB-02 into your computer.

2. Build and download the application to the board.

3. To measure GATT throughput, open the CySmart PC application (can be downloaded from
   http://www.cypress.com/documentation/software-and-drivers/cysmart-bluetooth-le-test-and-debug-tool)
   and connect to the CySmart CY5677 dongle. Check out this link for more details on the
   dongle: http://www.cypress.com/cy5677
    - Scan and Connect to 'TPUT' device
    - If asked, click Yes to update the connection parameters
    - Discover all attributes
    - Enable 'Notifications'
    - GATT throughput results(in Bytes/second) will be displayed in the device's PUART serial console. Use
      any terminal emulator to view PUART messages (Baud=115200, Data=8, Parity=N, Stop=1, Flow=None).
    - Disable 'Notifications' to stop measuring GATT throughput

4. It is also possible to use Android or iOS apps to connect and enable GATT notifications.
   One option is to use the mobile version of CySmart. Check out details at
   http://www.cypress.com/documentation/software-and-drivers/cysmart-mobile-app

For more details on this code example including the design and implementation,
please refer to the CE document available under this example's folder at GitHub
(https://github.com/cypresssemiconductorco/Code-Examples-BT-20819A1-1.0-for-ModusToolbox-1.1)

For more details on the hardware connections in the CYW920819EVB-02 Evaluation
kit, please refer to the ModusToolbox CYW920819EVB-02 Evaluation Kit User Guide.pdf
(http://www.cypress.com/CYW920819EVB-02)

Project Settings
----------------
Application specific project settings are as below -
LE_COC_SUPPORT
    Allow LE Connection Oriented Channel Support
VERBOSE_THROUGHPUT_OUTPUT
    Display verbose output
----------------------------------------------------------------------------------------------------------
