-------------------------------------------------------------------------------
BLE Connection Oriented Channel (COC) (Server/Client)application
-------------------------------------------------------------------------------

Features demonstrated
---------------------
 - WICED BT LE L2CAP APIs for Connection Oriented Channels

Server:
 - Start advertisements from client control application
 - Waits for connection from peer application (on the chosen l2cap psm)
 - On connection, waits for data from peer

Client:
 - Scan from the Client control for the Server
 - Connect to server and send data

See chip specific readme for more information about the BT SDK.
-------------------------------------------------------------------------------