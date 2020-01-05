#!/usr/bin/env python3

import os
import sys
import subprocess
import rospy

'''os.system('roscd speech_recognition/stanford-corenlp-full-2018-10-05 && java -mx4g -cp "*" edu.stanford.nlp.pipeline.StanfordCoreNLPServer -port 9000 -timeout 15000')'''


p = subprocess.Popen(["java", "-mx4g", '-cp "*" edu.stanford.nlp.pipeline.StanfordCoreNLPServer', "-port 9000", "-timeout 15000"], cwd="~/catkin_ws/src/kmr19/speech_recognition/stanford-corenlp-full-2018-10-05")
p.wait()
