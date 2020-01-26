#!/bin/bash

cd $HOME/catkin_ws/src/kmr19/speech_recognition/stanford-corenlp-full-2018-10-05
java -mx4g -cp "*" edu.stanford.nlp.pipeline.StanfordCoreNLPServer -port 9000 -timeout 15000
