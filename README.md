The example code basically does everything we need it to (except for JSON), but we need to reformat it to fit out project. Mostly it's the stuff listed in the demo document, making things more generic so we can reuse them (like publish), and creating a thread that can handle all the messages we'll want to send.

*Things to change:*
Stuff I found while looking it over (no guarentees all of it is 100% accurate):
1. Mqtt_IF_Connect: establishes internett connection
2. Mqtt_start: creates message queue and starts MQTT thread, also handles subscription
3. MqttClient: loops forever, blocking on queue. Handles publishing of messages
4. MqttClientCallback (in client_cbs.h): example of how a call back should work (will need to handle out message and call parse JSON messages and stuff)

*Setup:*

1. Clone this repo into your CCS workspace (probably c:/users/<username>/workspace_v9)
2. In CCS, use "project > Import CCS Project" to add it to your work space
3. You might have to reset the freertos dependency by going to: "project > properites > build > dependencies" and then removing and re-adding the freertos project.

At this point you should be able to build and modify the project. Before you make your changes, make a branch using "git branch <branch-name>" and "git checkout <branch-name>" to make integration easier.

*Running notes:*

1. network_if.h/.c is the file that contains network info (name, password of network to connect to)
	- The easiest way to run this is to download mosquitto's broker (I put the installer on the shared drive) and run "mosquitto.exe -v" from command line. Then just modify SSID_NAME and SECURITY_KEY in network_if and SERVER_IP_ADDRESS in mqtt_client_app.c to your computer's IP