#!/usr/bin/env python
# -*- encoding: UTF-8 -*-

import time
import re
import random
import threading



global blinkThread
global skip
skip = 0 

class IDLEMOVEMENTTHREADING:
	
	def __init__(self, motionProxy, proxy, audioReactionProxy, blinkThread, ):
		n = 1

	
	def blinkingModeON(mode):

		global blinkThread
		blinkThread = threading.Timer(5, blinkingModeON, [mode])
		emotionReaction = EYEEMOTIONS(proxy)
		animationSelection = MOTION_ANIMATION_SELECTION()
		if mode == "ON":
			blinkThread.start()	
			
			emotionReaction.blink_eyes()
			#animationSelection.reactionIdleMovement()
			
				

		if mode == "OFF":
			emotionReaction.turn_off_eye()
			blinkThread.cancel()


	def blinkingModeOFF():
		global blinkThread
		#emotionReaction.turn_off_eye()
		blinkThread.cancel()



	def idleMovementModeON(mode):
		global skip
		global idleThread
		idleThread = threading.Timer(50, idleMovementModeON, [mode])
		animationSelection = MOTION_ANIMATION_SELECTION()
		idleThread.start()
		skip += 1
		#print k
		if mode == "ON":		
			#emotionReaction.blink_eyes()
			if skip >= 2: 
				motionProxy.setBreathEnabled('Arms', False)
				animationSelection.reactionIdleMovement()
				motionProxy.setBreathEnabled('Arms', True)
			


	def idleMovementModeOFF():
		global idleThread
		global skip
		#emotionReaction.turn_off_eye()
		idleThread.cancel()
		skip = 0 


	def twoPageModeON(mode):
		global k
		global twoPageThread
		twoPageThread = threading.Timer(40, idleMovementModeON, [mode])
		animationSelection = MOTION_ANIMATION_SELECTION()
		idleThread.start()
		k += 1
		#print k
		if mode == "ON":		
			#emotionReaction.blink_eyes()
			if k >= 2: 
				motionProxy.setBreathEnabled('Arms', False)
				animationSelection.reactionIdleMovement()
				motionProxy.setBreathEnabled('Arms', True)