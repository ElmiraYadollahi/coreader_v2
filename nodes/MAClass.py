#!/usr/bin/env python
# -*- encoding: UTF-8 -*-
"""
Created on Mon, Apr  3 2017 15:11:09

@author: Elmira
"""

import animations.embarassed_seated_pose
import animations.scratchHead_seated_pose
import animations.IdontKnow_seated_pose
import animations.IdontKnow2_seated_pose
import animations.hesitation_seated_pose
import animations.hesitation2_seated_pose
import animations.hesitation3_seated_pose
import animations.thinking5_seated_pose
import animations.thinking6_seated_pose
import animations.thinking7_seated_pose
import animations.thinking8_seated_pose
import animations.monster_pose
import animations.disappointed_seated_pose
import animations.excited_seated_pose
import animations.excited2_seated_pose
import animations.happy_seated_pose
import animations.happy2_seated_pose
import animations.happy3_seated_pose
import animations.introduction_pose
import animations.proud_seated_pose
import animations.nod_pose
import animations.relieved_seated_pose
import animations.winner_seated_pose
import animations.winner2_seated_pose
import animations.pensive_seated_pose

from naoqi import ALProxy 
import codecs
import time
import re
import random

# Debug
import rospy
from memory.msg import Animation

import sys
import motion
import almath
import math
from std_msgs.msg import Int64, Int64MultiArray, String
from geometry_msgs.msg import Point, PoseStamped, PoseArray, Pose
from ar_track_alvar_msgs.msg import AlvarMarkers
import numpy as np
from numpy.linalg import inv
from EEClass import EYEEMOTIONS
from MTClass import TRANSFORMATION
from WPClass import WORDPROCESSING

class MOTIONANIMATIONSELECTION:
	
	def __init__(self, motionProxy, proxy, audioReactionProxy, blinkThread, ):
		n = 1

	def reactionToREDCard(self):
		""" Select a motionProxy from the available motionProxys after receiving 

		"""
		
		motionProxy.setExternalCollisionProtectionEnabled("All", True)
		motionProxyNum = random.randint(1,5)
		#motionProxyNum = 1

		if motionProxyNum == 6:
			wordsBefore = "\\rspd=90\\ Oh Really???"
			sleepTime = 2
			wordsAfter = "\\rspd=90\\wait!! \\pau=500\\ I'll try again"
			emotion = "sad"
			reactToTheMistake(emotion, animations.embarassed_seated_pose, wordsBefore, wordsAfter, sleepTime)

		if motionProxyNum == 2:
			wordsBefore = "\\rspd=60\\ Aaahhh!! \\pau=600\\ \\rspd=90\\ I didn't know!!"		
			sleepTime = 1
			wordsAfter = "\\rspd=80\\ let me try again"
			emotion = "surprise"
			reactToTheMistake(emotion, animations.scratchHead_seated_pose, wordsBefore, wordsAfter, sleepTime, 0.8)

		if motionProxyNum == 7:
			wordsBefore = "\\rspd=80\\ Oh!! sorry!!"		
			sleepTime = 1
			wordsAfter = "\\rspd=90\\ I will read it again"
			emotion = "sad"
			reactToTheMistake(emotion, animations.disappointed_seated_pose, wordsBefore, wordsAfter, sleepTime, 0.8)

		if motionProxyNum == 4:
			wordsBefore = "\\rspd=70\\ Oh!! sorry!!"		
			sleepTime = 1
			wordsAfter = "\\rspd=80\\ I will read it again"
			emotion = "sad"
			reactToTheMistake(emotion, animations.pensive_seated_pose, wordsBefore, wordsAfter, sleepTime, 0.9)

		if motionProxyNum == 5:
			wordsBefore = "\\rspd=70\\ hmm!!"		
			sleepTime = 1
			wordsAfter = "\\rspd=80\\ I need to read it again"
			emotion = "sad"
			reactToTheMistake(emotion, animations.thinking6_seated_pose, wordsBefore, wordsAfter, sleepTime, 0.8)

		if motionProxyNum == 1:
			wordsBefore = "\\rspd=80\\ oh  \\pau=700\\ \\rspd=60\\  really?!!"		
			sleepTime = 1
			wordsAfter = " \\rspd=80\\ \\pau=200\\ let me read it again"
			emotion = "surprise"
			reactToTheMistake(emotion, animations.hesitation2_seated_pose, wordsBefore, wordsAfter, sleepTime, 0.8)

		if motionProxyNum == 3:
			wordsBefore = "\\rspd=60\\ Oh!!! \\rspd=80\\ \\pau=700\\ I was wrong"		
			sleepTime = 1
			wordsAfter = "\\rspd=90\\ I will try again"
			emotion = "sad"
			reactToTheMistake(emotion, animations.thinking5_seated_pose, wordsBefore, wordsAfter, sleepTime, 0.8)


	def reactionToGREENCard(self):
		""" Select a motionProxy from the available motionProxys after receiving the green card 

		"""
		
		motionProxy.setExternalCollisionProtectionEnabled("All", True)
		motionProxyNum = random.randint(1,5)
		#motionProxyNum = 4
		emotion = "happy"

		if motionProxyNum == 1:
			pitch_angle = 0.0
			LookAtTheBook(pitch_angle)
			wordsBefore = "\\rspd=80\\ Yeaaah!!!"
			sleepTime = 3
			wordsAfter = "\\rspd=70\\ Thank you"
			reactToTheMistake(emotion, animations.winner_seated_pose, wordsBefore, wordsAfter, sleepTime, 0.8)

		if motionProxyNum == 2:
			pitch_angle = 0.0
			LookAtTheBook(pitch_angle)
			wordsBefore = "\\rspd=60\\ Yeaaah!!!"		
			sleepTime = 2
			wordsAfter = "\\rspd=80\\ Thank you"
			reactToTheMistake(emotion, animations.winner2_seated_pose, wordsBefore, wordsAfter, sleepTime, 1.0)

		if motionProxyNum == 3:
			pitch_angle = 0.0
			LookAtTheBook(pitch_angle)
			wordsBefore = "\\rspd=80\\ Yeaaah!!!"		
			sleepTime = 2
			wordsAfter = "\\rspd=80\\ I made it "
			reactToTheMistake2(animations.relieved_seated_pose, wordsBefore, wordsAfter, sleepTime, 0.8)

		if motionProxyNum == 4:
			pitch_angle = 0.0
			LookAtTheBook(pitch_angle)
			wordsBefore = "\\rspd=80\\ Yeaaah!!!"		
			sleepTime = 2
			wordsAfter = "\\rspd=80\\ "
			reactToTheMistake(emotion, animations.proud_seated_pose, wordsBefore, wordsAfter, sleepTime, 0.9)

		if motionProxyNum == 5:
			wordsBefore = "\\rspd=80\\ Yeaaah!!!"		
			sleepTime = 3
			wordsAfter = "\\rspd=80\\ "
			reactToTheMistake(emotion, animations.happy_seated_pose, wordsBefore, wordsAfter, sleepTime, 0.8)

		if motionProxyNum == 6:
			wordsBefore = "\\rspd=70\\ Yeaaah!!!"		
			sleepTime = 2
			wordsAfter = "\\rspd=70\\ "
			reactToTheMistake(emotion, animations.happy2_seated_pose, wordsBefore, wordsAfter, sleepTime, 0.8)
			pitch_angle = 0.0
			LookAtTheBook(pitch_angle)

		if motionProxyNum == 7:
			wordsBefore = "\\rspd=70\\ Yeaaah!!!"		
			sleepTime = 2
			wordsAfter = "\\rspd=80\\ "
			reactToTheMistake(emotion, animations.happy3_pose, wordsBefore, wordsAfter, sleepTime)



	def reactionIdleMovement(self):
		motionProxy.setExternalCollisionProtectionEnabled("All", True)
		motionProxyNum = random.randint(1,4)
		#motionProxyNum = 4
		emotion = "happy"

		if motionProxyNum == 1:
			#pitch_angle = 0.0
			#LookAtTheBook(pitch_angle)
			wordsAfter = "\\rspd=70\\ hmm"
			sleepTime = 3
			reactToBoredness(emotion, animations.scratchHead_seated_pose, sleepTime, wordsAfter, 0.7)

		if motionProxyNum == 2:
			#pitch_angle = 0.0
			#LookAtTheBook(pitch_angle)
			wordsAfter = "\\rspd=70\\ hmm"			
			sleepTime = 2
			reactToBoredness(emotion, animations.scratchHand_seated_pose, sleepTime, wordsAfter, 0.7)

		if motionProxyNum == 3:
			#pitch_angle = 0.0
			#LookAtTheBook(pitch_angle)
			wordsAfter = "\\rspd=70\\ hmm"
			sleepTime = 2
			reactToBoredness(emotion, animations.lookHand_seated_pose, sleepTime, wordsAfter, 0.7)

		if motionProxyNum == 4:
			#pitch_angle = 0.0
			#LookAtTheBook(pitch_angle)
			wordsAfter = "\\rspd=70\\ I'm so \\rspd=50\\ bored, \\rspd=80\\ \\pau=50\\ you don't want to read with me?"
			sleepTime = 2
			reactToBoredness(emotion, animations.relaxation_seated_pose, sleepTime, wordsAfter, 0.7)



	def reactionToSpecificTags(self):
		motionProxy.setExternalCollisionProtectionEnabled("All", True)
		
		wordsAfter = ""
		sleepTime = 2
		emotion = "anger"
		reactWithSpecificPoses(emotion, animations.monster_seated_pose, sleepTime, wordsAfter, 0.7)


	def reactWithSpecificPoses(emotion, pose, pause, wordsAfter , factorSpeed = 1.0):

		global blinkThread
		#idleMovementMode("OFF")
		#blinkingModeOFF()
		#blinkThread.sleep()
		motionProxy.setBreathEnabled('Arms', False)

		emotionReaction = EYEEMOTIONS(proxy)

		times = changeSpeed(pose.times, factorSpeed)
		id = motionProxy.post.angleInterpolationBezier(pose.names, times, pose.keys)
		audioReactionProxy.post.playFile("/home/nao/audio/wav/monsterGrowl.wav")
		#story.post.say(wordsBefore)
		emotionReaction.set_emotion(emotion)
		#turn_on_eye()
		#time.sleep(pause)
		#motionProxy.wait(id,0)
		#time.sleep(1)
		#story.say(wordsAfter)
		#time.sleep(5)
		correctFlag = True
		pitch_angle = 0.3
		LookAtTheBook(pitch_angle)
		motionProxy.setBreathEnabled('Arms', True)
		#if wordsAfter != None:
		#story.say(wordsAfter)
		#blinkThread.start()
		#motionProxy.setBreathEnabled('Head', True)
		#postureProxy.goToPosture("Stand", 1.0)
		#readTheTaggedStory(selectedStory, correctFlag)
		#readTheTaggedStoryWithLevel(selectedStory, correctFlag)
		#idleMovementMode("ON")


	def reactToBoredness(emotion, pose, pause, wordsAfter , factorSpeed = 1.0):

		global blinkThread
		#idleMovementMode("OFF")
		#blinkingModeOFF()
		#blinkThread.sleep()
		motionProxy.setBreathEnabled('Arms', False)

		emotionReaction = EYEEMOTIONS(proxy)

		times = changeSpeed(pose.times, factorSpeed)
		id = motionProxy.post.angleInterpolationBezier(pose.names, times, pose.keys)
		#story.post.say(wordsBefore)
		#emotionReaction.set_emotion(emotion)
		#turn_on_eye()
		#time.sleep(pause)
		#motionProxy.wait(id,0)
		#time.sleep(1)
		#story.say(wordsAfter)
		#time.sleep(5)
		correctFlag = True
		pitch_angle = 0.3
		LookAtTheBook(pitch_angle)
		motionProxy.setBreathEnabled('Arms', True)
		#if wordsAfter != None:
		story.say(wordsAfter)
		#blinkThread.start()
		#motionProxy.setBreathEnabled('Head', True)
		#postureProxy.goToPosture("Stand", 1.0)
		#readTheTaggedStory(selectedStory, correctFlag)
		#readTheTaggedStoryWithLevel(selectedStory, correctFlag)
		#idleMovementMode("ON")


	def reactToTheMistake( emotion, pose, wordsBefore, wordsAfter, pause, factorSpeed = 1.0):
		""" If the keys pressed are due to detection of mistake, the robot reacts.
			The motionProxy is a physical movement and certain words which shows robot's remorse

		"""
		global blinkThread
		#idleMovementMode("OFF")
		blinkingModeOFF()
		idleMovementModeOFF()
		#blinkThread.sleep()
		motionProxy.setBreathEnabled('Arms', False)

		emotionReaction = EYEEMOTIONS(proxy)

		times = changeSpeed(pose.times, factorSpeed)
		id = motionProxy.post.angleInterpolationBezier(pose.names, times, pose.keys)
		story.post.say(wordsBefore)
		emotionReaction.set_emotion(emotion)
		turn_on_eye()
		#time.sleep(pause)
		#motionProxy.wait(id,0)
		#time.sleep(1)
		story.say(wordsAfter)
		#time.sleep(5)
		correctFlag = True
		pitch_angle = 0.3
		LookAtTheBook(pitch_angle)
		motionProxy.setBreathEnabled('Arms', True)
		#blinkThread.start()
		#motionProxy.setBreathEnabled('Head', True)
		#postureProxy.goToPosture("Stand", 1.0)
		#readTheTaggedStory(selectedStory, correctFlag)
		#readTheTaggedStoryWithLevel(selectedStory, correctFlag)
		blinkingModeON("ON")
		idleMovementModeON("ON")


	def reactToTheMistake2( pose, wordsBefore, wordsAfter, pause, factorSpeed = 1.0):
		""" If the keys pressed are due to detection of mistake, the robot reacts.
			The motionProxy is a physical movement and certain words which shows robot's remorse

		"""
		global blinkThread
		#blinkThread.sleep()
		#idleMovementMode("OFF")
		blinkingModeOFF()
		idleMovementModeOFF()
		motionProxy.setBreathEnabled('Arms', False)

		emotionReaction = EYEEMOTIONS(proxy)

		times = changeSpeed(pose.times, factorSpeed)
		id = motionProxy.post.angleInterpolationBezier(pose.names, times, pose.keys)
		story.post.say(wordsBefore)
		emotionReaction.set_emotion("happy")
		turn_on_eye()
		#time.sleep(pause)
		#motionProxy.wait(id,0)
		#time.sleep(1)
		story.say(wordsAfter)
		#time.sleep(5)
		correctFlag = True
		pitch_angle = 0.3
		LookAtTheBook(pitch_angle)
		motionProxy.setBreathEnabled('Arms', True)
		#motionProxy.setBreathEnabled('Head', True)
		#postureProxy.goToPosture("Stand", 1.0)
		#readTheTaggedStory(selectedStory, correctFlag)
		#readTheTaggedStoryWithLevel(selectedStory, correctFlag)
		blinkingModeON("ON")
		idleMovementModeON("ON")


	def turn_on_eye():
		section1 = ["FaceLeds", "ChestLeds" ]
		proxy.createGroup("turn",section1)
		proxy.fadeRGB("turn", 0x00FFFFFF, 0.3)


	def changeSpeed(times, factor):
		""" It changes the speed of predefined times for each pose movement

		"""

		for i in xrange(len(times)):
			times[i] = [x / float(factor) for x in times[i]]

		return times



	def LookAtTheBook(pitch_angle, yaw_angle=0):
		""" Move the robot's head to look at the camera

		"""

		motionProxy.setStiffnesses("Head", 1.0)

		# Example showing how to set angles, using a fraction of max speed
		isAbsolute = True
		names  = ["HeadYaw", "HeadPitch"]
		angles  = [yaw_angle, pitch_angle]
		fractionMaxSpeed  = 0.05
		moveTime = 1
		motionProxy.angleInterpolation(names, angles, moveTime, isAbsolute)

		"""names               = "HeadYaw"
		changes             = -0.5
		fractionMaxSpeed    = 0.05
		motionProxy.changeAngles(names, changes, fractionMaxSpeed)"""

		time.sleep(1.0)
		motionProxy.setStiffnesses("Head", 0.0)