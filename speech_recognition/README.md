# Speech recognition ROS package #

## Set up the access key ##

To make the google speech API work  you will need to possess the secret access key. This is available as a Json file in the package.
You will need to set up the path in the .bashrc by writing :
`export GOOGLE_APPLICATION_CREDENTIALS=$HOME/catkin_ws/src/kmr19/speech_recognition/gcsapi_user_credentials.json`

## Set up the environment and the API ##

We base our program on the Google Speech API and some other python libraries. We will store them in a virtual environment
Follow the steps to set up the virtual environment and installing the API as indicated here : https://cloud.google.com/python/setup

You will probably have to install more libraries to make it work, especially portaudio, pyaudio, pyyaml, rospkg. Install them in the environment.

Reminder, the programs will only work in this virtual environment. So when using the nodes, be sure to have executed the command :
`source env/bin/activate`
To leave the environment, just do `deactivate`

Nodes should be already set as executables. If not, go in the nodes folder and do :
`chmod +x speech_request_node.py` and `chmod +x parser_client_node.py`

## How it works ##

Launch the commands `roscore`, `rosrun speech_recognition speech_request_node.py` and `rosrun speech_recognition parser_client_node.py` in 3 different terminals. Don't forget the `source devel/setup.bash` from the catkin workspace if you don't see the nodes.

When the user will say "hello Baxter", the speech recog node will print "Listening...". This means that the next request will be sent to the other node, the subscriber. You always have to say "hello Baxter" and to wait for the "Listening..." message before sending a request.

## TODO ##

Use the stanford parser to parse correctly the information from the request. Once this is done, return a correct request based on the semantics set up in StackGoalService.srv and send it to the reasoning and planning part.

