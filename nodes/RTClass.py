"""PyAudio example: Record a few seconds of audio and save to a WAVE file."""

import pyaudio
import wave
import argparse
import base64
import json

from googleapiclient import discovery
import httplib2
from oauth2client.client import GoogleCredentials
from EEClass import EYEEMOTIONS


DISCOVERY_URL = ('https://{api}.googleapis.com/$discovery/rest?'
                 'version={apiVersion}')




class RECORDANDTRANSCRIBE:

    def __init__(self, proxy):

        tag = "=WordNum"
        #self.eyeProxy = proxy
        self.recordingReaction = EYEEMOTIONS(proxy)

    def recordTheOutput(self, output):
    	CHUNK = 1024
    	FORMAT = pyaudio.paInt16
    	CHANNELS = 1
    	RATE = 44100
    	RECORD_SECONDS = 5
    	WAVE_OUTPUT_FILENAME = output
        self.fileName = output

    	p = pyaudio.PyAudio()

    	stream = p.open(format=FORMAT,
    	                channels=CHANNELS,
    	                rate=RATE,
    	                input=True,
    	                frames_per_buffer=CHUNK)

    	print("* recording")
        self.recordingReaction.set_emotion("listen")

    	frames = []

    	for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
    	    data = stream.read(CHUNK)
    	    frames.append(data)

    	print("* done recording")

    	stream.stop_stream()
    	stream.close()
    	p.terminate()

        wf = wave.open(WAVE_OUTPUT_FILENAME, 'wb')
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(p.get_sample_size(FORMAT))
        wf.setframerate(RATE)
        wf.writeframes(b''.join(frames))
        wf.close()



    def getSpeechService(self):
        credentials = GoogleCredentials.get_application_default().create_scoped(
            ['https://www.googleapis.com/auth/cloud-platform'])
        http = httplib2.Http()
        credentials.authorize(http)

        return discovery.build(
            'speech', 'v1beta1', http=http, discoveryServiceUrl=DISCOVERY_URL)


    def transcribeTheOutput(self):
        """Transcribe the given audio file.

        Args:
            speech_file: the name of the audio file.
        """

        with open(self.fileName, 'rb') as speech:
            speech_content = base64.b64encode(speech.read())

        service = self.getSpeechService()
        service_request = service.speech().syncrecognize(
            body={
                'config': {
                    'encoding': 'LINEAR16',  # raw 16-bit signed LE samples
                    'sampleRate': 44100,  # 16 khz
                    'languageCode': 'en-US',  # a BCP-47 language tag
                },
                'audio': {
                    'content': speech_content.decode('UTF-8')
                    }
                })
        response = service_request.execute()
        print(json.dumps(response))

        return response
