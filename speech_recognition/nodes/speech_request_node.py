#!/usr/bin/env python3

from __future__ import division

import re
import sys
import os
import rospy
import subprocess

from std_msgs.msg import String

from google.cloud import speech
from google.cloud.speech import enums
from google.cloud.speech import types
import pyaudio
from six.moves import queue

from reasoning_planning.srv import StackGoalService, StackGoalServiceResponse, StackGoalServiceRequest
from speech_recognition.srv import CommSpeechParser, CommSpeechParserResponse, CommSpeechParserRequest

# Audio recording parameters
RATE = 16000
CHUNK = int(RATE / 10)  # 100ms

class MicrophoneStream(object):
    """Opens a recording stream as a generator yielding the audio chunks."""
    def __init__(self, rate, chunk):
        self._rate = rate
        self._chunk = chunk

        # Create a thread-safe buffer of audio data
        self._buff = queue.Queue()
        self.closed = True

    def __enter__(self):
        self._audio_interface = pyaudio.PyAudio()
        self._audio_stream = self._audio_interface.open(
            format=pyaudio.paInt16,
            # The API currently only supports 1-channel (mono) audio
            # https://goo.gl/z757pE
            channels=1, rate=self._rate,
            input=True, frames_per_buffer=self._chunk,
            # Run the audio stream asynchronously to fill the buffer object.
            # This is necessary so that the input device's buffer doesn't
            # overflow while the calling thread makes network requests, etc.
            stream_callback=self._fill_buffer,
        )

        self.closed = False

        return self

    def __exit__(self, type, value, traceback):
        self._audio_stream.stop_stream()
        self._audio_stream.close()
        self.closed = True
        # Signal the generator to terminate so that the client's
        # streaming_recognize method will not block the process termination.
        self._buff.put(None)
        self._audio_interface.terminate()

    def _fill_buffer(self, in_data, frame_count, time_info, status_flags):
        """Continuously collect data from the audio stream, into the buffer."""
        self._buff.put(in_data)
        return None, pyaudio.paContinue

    def generator(self):
        while not self.closed:
            # Use a blocking get() to ensure there's at least one chunk of
            # data, and stop iteration if the chunk is None, indicating the
            # end of the audio stream.
            chunk = self._buff.get()
            if chunk is None:
                return
            data = [chunk]

            # Now consume whatever other data's still buffered.
            while True:
                try:
                    chunk = self._buff.get(block=False)
                    if chunk is None:
                        return
                    data.append(chunk)
                except queue.Empty:
                    break

            yield b''.join(data)

def textToSpeech(s):
    ps = subprocess.Popen(('gtts-cli', s), stdout=subprocess.PIPE)
    output = subprocess.Popen(('play', '-t', "mp3","-"), stdin=ps.stdout)
    ps.wait()

def responseHandler(listOfErrors):
    
    responseToUser = ""
    indexInList = 0
    nbErrors = 0
    for error in listOfErrors:
        
        if (error != 0):
            if (indexInList == 0):
                responseToUser += "\nTOP BLOCK : "
            elif (indexInList == 1):
                responseToUser += "\nBELOW BLOCK : "
            elif (indexInList == 2):
                responseToUser += "\nGENERAL REQUEST : "
            
            if (error == 1):
                responseToUser += "Multiple same objects are defined. Be more precise about the scene."
            elif (error == 2):
                responseToUser += "No object found. Are you sure it is on the scene ?"    
            elif (error == 3):
                responseToUser += "Request unsolvable. Please do another request or be more precise."
            elif (error == 4):
                responseToUser += "Other error. Please try again or do another request."
            elif (error == 5):
                responseToUser += "Blocks unsuccessfully received from perception. Check camera."
            elif (error == 6):
                responseToUser += "Less than two blocks on the scene. Action impossible."
            nbErrors += 1
        indexInList += 1
        
    if (nbErrors == 0):
        responseToUser += "This is a valid request."
            
    textToSpeech(responseToUser)   
        
    return responseToUser
                
                
def publish_request(our_request):
    rospy.wait_for_service("speech_parser_request")
    speech_parser_request = rospy.ServiceProxy('speech_parser_request', CommSpeechParser)
    resp = speech_parser_request(our_request)
    
    print(responseHandler(resp.errorsResponse))
    return resp


def listen_print_loop(responses):
    """Iterates through server responses and prints them.

    The responses passed is a generator that will block until a response
    is provided by the server.

    Each response may contain multiple results, and each result may contain
    multiple alternatives; for details, see https://goo.gl/tjCPAU.  Here we
    print only the transcription for the top alternative of the top result.

    In this case, responses are provided for interim results as well. If the
    response is an interim one, print a line feed at the end of it, to allow
    the next result to overwrite it, until the response is a final one. For the
    final one, print a newline to preserve the finalized transcription.
    """
    num_chars_printed = 0
    publish_next_request = False

    for response in responses:
        if not response.results:
            continue

        # The `results` list is consecutive. For streaming, we only care about
        # the first result being considered, since once it's `is_final`, it
        # moves on to considering the next utterance.
        result = response.results[0]
        if not result.alternatives:
            continue

        # Display the transcription of the top alternative.
        transcript = result.alternatives[0].transcript

        # Display interim results, but with a carriage return at the end of the
        # line, so subsequent lines will overwrite them.
        #
        # If the previous result was longer than this one, we need to print
        # some extra spaces to overwrite the previous result
        overwrite_chars = ' ' * (num_chars_printed - len(transcript))

        if not result.is_final:
            #sys.stdout.write(transcript + overwrite_chars + '\r')
            #sys.stdout.flush()

            num_chars_printed = len(transcript)

        else:
            #print(transcript + overwrite_chars)
            # Exit recognition if any of the transcribed phrases could be
            # one of our keywords.
            print(transcript + overwrite_chars)

            if re.search(r'\b(exit|quit)\b', transcript, re.I):
                print('Exiting..')
                break
            
            if (publish_next_request):
                #Publishing
                print("Request sent\n")
                publish_request(transcript)
                print("\nReady for a new request...\n")
                
                publish_next_request = False
                
            if re.search(r'\b(hello Baxter| Hello Baxter)\b', transcript, re.I):
                publish_next_request = True
                
                textToSpeech("I am listening")
                print("Listening ...\n")
            
            num_chars_printed = 0

def main():
    # See http://g.co/cloud/speech/docs/languages
    # for a list of supported languages.
    language_code = 'en-US'  # a BCP-47 language tag

    client = speech.SpeechClient()
    single_utterance = True
    config = types.RecognitionConfig(
        encoding=enums.RecognitionConfig.AudioEncoding.LINEAR16,
        sample_rate_hertz=RATE,
        language_code=language_code,
        enable_automatic_punctuation=False)
    streaming_config = types.StreamingRecognitionConfig(
        config=config,
        interim_results=True)

    with MicrophoneStream(RATE, CHUNK) as stream:
        audio_generator = stream.generator()
        requests = (types.StreamingRecognizeRequest(audio_content=content)
                    for content in audio_generator)

        responses = client.streaming_recognize(streaming_config, requests)

        # Now, put the transcription responses to use.
        listen_print_loop(responses)

if __name__ == '__main__':
    main()
