#Speech recognition ROS package

##Set up the access key

To make the google speech API work  you will need to possess the secret access key. This is available as a Json file in the package.
You will need to set up the path in the .bashrc by writing :
`export GOOGLE_APPLICATION_CREDENTIALS=$HOME/catkin_ws/src/kmr19/speech_recognition/gcsapi_user_credentials.json`

##Set up the dependancies

To make this package work properly, you will need to download the Google-speech API for ROS called ros_speech_us
[(http://wiki.ros.org/speech_recog_uc)]

This is composed of two packages, `speech_recog_uc` and `speech_recog_uc_basic`. Here we will just need to set up `speech_recog_uc_basic`.
As explained on the page, go to the speech_recog_uc_basic/scripts folder and execute the script :
`./speech_install_script.sh`

This will install PortAudio and the files from the speech-recognition API
It seems like it also creates a "speech" folder in the home directory. Go check in your .bashrc if the GOOGLE_APPLICATION_CREDENTIALS haven't been overwritten because it seems like it's set up here by default. Delete the new definition of GOOGLE_APPLICATION_CREDENTIALS if necessary.

### FOR NOW NO SPEECH RECOGNITION NODE IS IMPLANTED IN OUR speech_recognition PACKAGE, WERE WORKING ON IT ###
