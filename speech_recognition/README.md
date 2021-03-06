# Speech recognition ROS package #

## Set up the access key ##

To make the google speech API work  you will need to possess the secret access key. This is available as a Json file in the package.
You will need to set up the path in the .bashrc by writing :
`export GOOGLE_APPLICATION_CREDENTIALS=$HOME/catkin_ws/src/kmr19/speech_recognition/gcsapi_user_credentials.json`

## Set up the Google Speech environment ##

We base our program on the Google Speech API and some other python libraries. We will store them in a virtual environment
Follow the steps to set up the virtual environment and installing the API as indicated here : https://cloud.google.com/python/setup
We're working with python3 here, so install pip3 and everytime it says to do a "pip", do a "pip3" instead.
If you want, you don't have to set up and install the programs in a virtual environment so you can skip some steps.

Once the python environment is set up, install the speech-to-text python libraries with : `pip3 install --upgrade google-cloud-speech`
You will probably have to install more libraries to make it work, especially portaudio, pyaudio, pyyaml, rospkg. Install them in the virtual environment if you chose to work with that.

Reminder, if you use the virtual environment to install your python libraries, when using the nodes be sure to have executed the command :
`source env/bin/activate`
To leave the virtual environment, just do `deactivate`

Nodes should be already set as executables. If not, go in the nodes folder and do :
`chmod +x speech_request_node.py` and `chmod +x parser_client_node.py`

## Set up the Stanford Parser environment ##

To make the Stanford parser work, we need to download the server files, install Java and run the server in the background to parse the data.
Download the archive at https://stanfordnlp.github.io/CoreNLP/download.html and extract it where you want (I would personnaly put it in the speech reco package to have everything together)

Install the Java jre and jdk to make it work properly : 
`sudo apt-get update`
`sudo apt-get install default-jre`
`sudo apt-get install default-jdk`

You will also need some libraries such as nltk. After activating your virtual env, do :
`pip3 install -U nltk`

In a separate terminal, go inside the StanfordNLP folder and launch the NLP server in the background with the command :
`java -mx4g -cp "*" edu.stanford.nlp.pipeline.StanfordCoreNLPServer -port 9000 -timeout 15000`

## Set up the Text to speech environment ##
Finally, you will need to install the programs and libraries to handle the text to speech
You will need to install sox :
`sudo apt-get install sox soxlib-fmt-mp3`

And install google text to speech for python with :
`pip3 install gTTs`

## How it works ##

Launch the commands `roscore`, `rosrun speech_recognition speech_request_node.py` and `rosrun speech_recognition parser_client_node.py` in 3 different terminals. Don't forget the `source devel/setup.bash` from the catkin workspace if you don't see the nodes.

Remark : The speech recognition node will close itself after 305 seconds.

When the user will say "hello Baxter", the speech recog node will print qnd say "Listening...". This means that the next request will be sent to the other node, the subscriber. You always have to say "hello Baxter" and to wait for the "Listening..." message before sending a request.

The request will be parsed and set up with correct semantics to be sent to the reasoning and planning part.

You will be returned a printed and voice message telling if the request is correct or has errors

## TODO ##

Add the stanford server to the launch file
