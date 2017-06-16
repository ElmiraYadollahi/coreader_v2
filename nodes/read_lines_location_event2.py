#!/usr/bin/env python
# -*- encoding: UTF-8 -*-

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
import animations.monster_seated_pose
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


# Idle Movement
import animations.lookHand_seated_pose
import animations.relaxation_seated_pose
import animations.scratchHand_seated_pose
import animations.scratchHead_seated_pose


from naoqi import ALProxy 
import codecs
import time
import re
import random
import threading 

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
from RTClass import RECORDANDTRANSCRIBE
from IMClass import IDLEMOVEMENTTHREADING
from MAClass import MOTIONANIMATIONSELECTION
from logger import logger

""" It is a dictionary for the states of the tag:
	False: 						means the story have not be read yet, it will go into reading with wrong tag 
	"Waiting for Red-card":		means the robot is waiting to recieve a green card
	"Waiting for Yellow-card":	robot repeats reading the text
	"Waiting for Green-card":	robot reacts happily
	"correcting Mode": 			means the story have been read and the robot have recieved a red card
	"Corrected Mode":		
	"Correct Happy Mode"
	"Wrong Happy Mode"	
	True:						means the story have been read and the robot have recieved a green card

"""
pairs_dict = {	'[0, 1]' : False,
				'[2, 3]' : False,
				'[4, 5]' : False,
				'[6, 7]' : False,
				'[8, 9]' : False,
				'[10, 11]' : False,
				'[12, 13, 14, 15]' : False,
				'[12, 13]' : False,
				'[ 14, 15]' : False,
				'[16, 17]' : False,
				'[18, 19, 20, 21]' : False,
				'[18, 19]' : False,
				'[ 20, 21]' : False,
				'[22, 23]' : False,
				'[24, 25, 26, 27]' : False,
				'[24, 25]' : False,
				'[ 26, 27]' : False,
				'[28, 29]' : False,
				'[30, 31, 32, 33]' : False,
				'[30, 31]' : False,
				'[ 32, 33]' : False,
				'[34, 35]' : False,
				'[36, 37, 48, 49]' : False,
				'[36, 37]' : False,
				'[ 48, 49]' : False,
				'[41, 46]' : False,
				'[42, 43]' : False,
				'[44, 45]' : False,

				'[50, 51]' : False,
				'[52, 53]' : False,
				'[54, 55, 56, 57]' : False,
				'[54, 55]' : False,
				'[ 56, 57]' : False,
				'[58, 59, 60, 61]' : False,
				'[58, 59]' : False,
				'[ 60, 61]' : False,
				'[62, 63]' : False,
				'[64, 65]' : False,
				'[66, 67, 68, 69]' : False,
				'[66, 67]' : False,
				'[ 68, 69]' : False,
				'[70, 71, 72, 73]' : False,
				'[70, 71]' : False,
				'[ 72, 73]' : False,
				'[74, 75, 76, 77]' : False,
				'[74, 75]' : False,
				'[ 76, 77]' : False,
				'[78, 79]' : False,



				'[250, 251]' : False,
				'[252, 253]' : False,
				'[200, 201, 202, 203]' : False,
				'[200, 201]' : False,
				'[ 202, 203]' : False,
				'[204, 205, 206, 207]' : False,
				'[204, 205]' : False,
				'[ 206, 207]' : False,
				'[208, 209, 210, 211]' : False,
				'[208, 209]' : False,
				'[ 210, 211]' : False,
				'[212, 213, 214, 215]' : False,
				'[212, 213]' : False,
				'[ 214, 215]' : False,
				'[216, 217, 218, 219]' : False,
				'[216, 217]' : False,
				'[ 218, 219]' : False,
				'[220, 221, 222, 223]' : False,
				'[220, 221]' : False,
				'[ 222, 223]' : False,
				
				'[256, 257]' : False,
				'[258, 259]' : False,
				'[260, 261]' : False,
				'[262, 263]' : False,
				'[264, 265]' : False,
				'[266, 267]' : False,
				'[268, 269]' : False,
				'[270, 271]' : False,
				'[272, 273]' : False,
				'[274, 275]' : False,
				'[276, 277]' : False,
				'[278, 279]' : False
			 }


story_dict ={	'[200, 201]': 1, 
				'[210, 211]': 2,
				'[220, 221]': 3,
				'[230, 231]': 4,
				'[240, 241]': 5,
				'[2, 3]': 7
			}

taskLevel_dict ={	'CurrLevel'		: "TaskNONE",
					'PrevLevel'		: "TaskNONE",
					'LevelONE'		: 1,
					'LevelTWO'		: 1,
					'LevelTHREE'	: 1,
				}


card_id = { 38, 39, 40}


instruction_tags_dict = {	'=LineNum',
							'=wordNum',
							'=MistakeNum',
							'=NextLine',
							'=Skip',
							'=L1',
							'=L2',
							'=L3',
							'=L4',
							'=L5',
							'=L6',
							'=B1',
							'=B1',
							'=B2',
							'=B3',
							'=B4',
							'=B5',
							'=B6'
							
						}


motionProxy_states = { "low": [1, 2, 3, 4, 5],
				  	"medium": [6, 7, 8, 9, 10],
				  	"high": [11, 12, 13, 14, 15] }


mistake_level_states = { "low": [1],
						 "medium": [3],
						 "high": [5]}

story_selection_state = {}

""" Recieve the key stroke from key publisher node
	[1]: tell the robot made a mistake in reading the story
	[2]: tell the robot to choose a new story and repeat the process
	[3]: tell the robot to stop the tracker and go to resting mode

"""
KEY_MAPPING = { 'f': [2], 
				'g': [2], 
				'h': [2],
				'b': [3],
				'n': [3],
				'm': [3],
				'e': [4],
				'r': [4],
				't': [4],
				's': [1]  
				}

global counter
counter = 0


global midP
midP = []
global path
path = []
global relatPath
relatPath = []
global poses
poses = []
global tag_pairs
tag_pairs = '[0, 0]'
global moveHand 
moveHand = 0
global selectedStory
global trajComplete
trajComplete = []
global correctPermission
correctPermission = False
global cardPosition
cardPosition = 0
global reactionPermission 
reactionPermission = False
global ARTag
global REDCardAllert
REDCardAllert = False
global NumOfGreenCard
NumOfGreenCard = 0
global effector
effector = "LArm"

class MOTION_ANIMATION_SELECTION:
	def __init__(self):
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
			sleepTime = 1
			wordsAfter = "\\rspd=70\\ Thank you"
			reactToTheMistake(emotion, animations.winner_seated_pose, wordsBefore, wordsAfter, sleepTime, 0.8)

		if motionProxyNum == 2:
			pitch_angle = 0.0
			LookAtTheBook(pitch_angle)
			wordsBefore = "\\rspd=60\\ Yeaaah!!!"		
			sleepTime = 1
			wordsAfter = "\\rspd=80\\ this is great"
			reactToTheMistake(emotion, animations.winner2_seated_pose, wordsBefore, wordsAfter, sleepTime, 1.0)

		if motionProxyNum == 3:
			pitch_angle = 0.0
			LookAtTheBook(pitch_angle)
			wordsBefore = "\\rspd=80\\ Yeaaah!!!"		
			sleepTime = 1
			wordsAfter = "\\rspd=80\\ I made it "
			reactToTheMistake2(animations.relieved_seated_pose, wordsBefore, wordsAfter, sleepTime, 0.8)

		if motionProxyNum == 4:
			pitch_angle = 0.0
			LookAtTheBook(pitch_angle)
			wordsBefore = "\\rspd=80\\ Wow!!!"		
			sleepTime = 1
			wordsAfter = "\\rspd=80\\ I was right"
			reactToTheMistake(emotion, animations.proud_seated_pose, wordsBefore, wordsAfter, sleepTime, 0.9)

		if motionProxyNum == 5:
			wordsBefore = "\\rspd=80\\ Yeaaah!!!"		
			sleepTime = 1
			wordsAfter = "\\rspd=80\\ I read it correctly"
			reactToTheMistake(emotion, animations.happy_seated_pose, wordsBefore, wordsAfter, sleepTime, 0.8)

		if motionProxyNum == 6:
			wordsBefore = "\\rspd=70\\ I'm so happy !!!"		
			sleepTime = 1
			wordsAfter = "\\rspd=70\\ I was right"
			reactToTheMistake(emotion, animations.happy2_seated_pose, wordsBefore, wordsAfter, sleepTime, 0.8)
			pitch_angle = 0.0
			LookAtTheBook(pitch_angle)

		if motionProxyNum == 7:
			wordsBefore = "\\rspd=70\\ Yeaaah!!!"		
			sleepTime = 1
			wordsAfter = "\\rspd=80\\ this is great"
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
			wordsAfter = "\\rspd=70\\ hmm, I'm sleepy"			
			sleepTime = 2
			reactToBoredness(emotion, animations.scratchHand_seated_pose, sleepTime, wordsAfter, 0.7)

		if motionProxyNum == 3:
			#pitch_angle = 0.0
			#LookAtTheBook(pitch_angle)
			wordsAfter = "\\rspd=70\\ hmm, I'm tired"
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




def cardPointedAt(msg):
	"""

	"""
	global cardPosition
	cardPosition += 1
	print "cardPosition"
	print cardPosition

	if cardPosition == 10:

		cameraName = 'CameraBottom'
		#changeCoordinate = TRANSFORMATION(motionProxy)
		#pointedWord = Pose()
		#pointedWord.position = msg.position
		PW = []
		PW.append([msg.position.x])
		PW.append([msg.position.y])
		PW.append([msg.position.z])
		PW.append([1])

		#P1 = changeCoordinate.transformPoint(PW, cameraName)
		print "P1"
		#print P1



global breakPoint
breakPoint = [0, 0]

def changeToPose(res):
	middle_pose = Pose()
	middle_pose.position.x = res[0]
	middle_pose.position.y = res[1]
	middle_pose.position.z = res[2]
	middle_pose.orientation.x = res[3]
	middle_pose.orientation.y = res[4]
	middle_pose.orientation.z = res[5]
	middle_pose.orientation.w = 1

	return middle_pose

def calculateWhereToPointAt(pointsAB, effector, frame, changeCoordinate):

	global correctPermission
	global selectedStory
	global trajComplete
	global story
	trajComplete = []
	global ARTag
	print "ARTag" 
	print ARTag
	global REDCardAllert
	global reactionPermission
	global breakPoint
	global robot_hand_pose_pub
	useSensorValues = True

	if(effector == "LArm"):
		whichHand = "LHand"
	else:
		whichHand = "RHand"

	for j in range(len(ARTag)):
		print j 
		wordProc = WORDPROCESSING(story, ARTag[j], taskLevel_dict, proxy)

		#INTag = "=WordNum"
		#wordCount = wordProc.getTheInstructionTagData(INTag)
		INTag = "=LineNum"
		lineCount = wordProc.getTheInstructionTagData(INTag)
		INTag = "=Skip"
		skipOrNot = wordProc.getTheInstructionTagData(INTag)
		print "skipOrNot"
		print skipOrNot
		INTag = "=Read"
		readOrNot = wordProc.getTheInstructionTagData(INTag)
		print "readOrNot"
		print readOrNot
		LiWoCount2 = []
		lineDistanceCoef = []
		#if lineCount > 1:
		for i in range(lineCount):
			INTag = "=L" + str((i+1))
			LiWoCount2.append(wordProc.getTheInstructionTagData(INTag))

			INTag = "=S" + str((i+1))
			lineDistanceCoef.append(0.01 * wordProc.getTheInstructionTagData(INTag))

		LiWoCount = wordProc.getNumberOfWords(lineCount)
		#elif lineCount == 1:
			#LiWoCount.append(wordCount)
		print "LiWoCount"
		print LiWoCount
		print lineDistanceCoef

		print "LiWoCount2"
		print LiWoCount2

		print "lineCount"
		print lineCount

		wordProc.toCorrectOrNotTo(ARTag)


		wordProc.getTheLineMatrix()
		#wordCount = wordProc.getTheWordCount()
		wordProc.clearAllTheInstructionTags()

		#changeCoordinate = TRANSFORMATION( motionProxy)
		#wordProc = WORDPROCESSING()
		print "LiWoCount = wordProc.getLineWordCount()"
		print LiWoCount
		print "pointsAB"
		print pointsAB

		maxSpeed = 0.1
		maxSpeedHead = 0.05
		useWholeBody = False

		trajComplete2 = []

		PA = Pose()
		PB = Pose()
		PC = Pose()
		PD = Pose()
		
		PA.position.x = pointsAB[2*j].item(0, 0)
		PA.position.y = pointsAB[2*j].item(1, 0)
		PA.position.z = pointsAB[2*j].item(2, 0)

		PB.position.x = pointsAB[2*j+1].item(0, 0)
		PB.position.y = pointsAB[2*j+1].item(1, 0)
		PB.position.z = pointsAB[2*j+1].item(2, 0)
		trajComplete2 = changeCoordinate.calculateEachWordPosition(PA, PB, effector, LiWoCount, lineDistanceCoef)



		print "PA"
		print PA
		print "PB"
		print PB
		print "PC"
		print PC
		print "PD"
		print PD

		hardWord = 4

		global current_word_pub
		global robot_state_pub
		robot_state_pub.publish("Reading Mode Started")

		print "len trajComplete2"
		print len(trajComplete2)
		#wordProc.readTheTaggedStory(correctPermission)

		if taskLevel_dict['CurrLevel'] == "TaskONE":
			if readOrNot == 1:
				for i in range(len(trajComplete2)):

					print "len(trajComplete2[i][0]"
					print len(trajComplete2[i][0])

					for j in range(len(trajComplete2[i][0])):
						if REDCardAllert == True:
							break
				
						#tracker.lookAt(trajComplete2[i][0][j], frame, maxSpeed, useWholeBody)
						#tracker.pointAt(effector, trajComplete2[i][0][j], frame, maxSpeedHead)
				
						#print result

						if j == 0:
							#wordProc.readFromWordMatrix(correctPermission, j, i+1, REDCardAllert)
							if REDCardAllert == True:
								break
							#wordProc.readFromMatrixLine(correctPermission, i+1, REDCardAllert)
							print "correct permission"
							print correctPermission
							x = True
							if REDCardAllert == True:
								break
						if (j ) %4 == 0:

							result          = motionProxy.getPosition(whichHand, frame, useSensorValues)
							handPose = changeToPose(result)
							#print handPose
							robot_hand_pose_pub.publish(handPose)		

							correctFlag = wordProc.readFromWordMatrix(correctPermission, j/4, i+1, breakPoint, REDCardAllert, taskLevel_dict['CurrLevel'], ARTag, current_word_pub)
							currReadingPoint = [i+1, j/4]
							print "j"
							print j
							print "j/4"
							print j/4
			
							#if j/4 == (hardWord-1):
								#stopAndAsk()
								#time.sleep(2)
							time.sleep(0.2)
			elif readOrNot == 0:
				animationSelection = MOTION_ANIMATION_SELECTION()
				animationSelection.reactionToSpecificTags()
				correctFlag = True

			#correctPermission = True

			if REDCardAllert == True:
				correctPermission = "Partial Correction"
				breakPoint = currReadingPoint
				print "breakPoint"
				print breakPoint
				break


			if correctFlag == "Partial Correction":
				correctPermission = "Partial Correction"
				breakPoint = [0, 0]


			elif correctFlag == True:
				correctPermission = True
				breakPoint = [0, 0]

				
			print "correctPermission"
			print correctPermission
			print "breakPoint"
			print breakPoint

		elif taskLevel_dict['CurrLevel'] == "TaskTWO":
			
			for i in range(len(trajComplete2)):

				print "len(trajComplete2[i][0]"
				print len(trajComplete2[i][0])

				for j in range(len(trajComplete2[i][0])):
					if REDCardAllert == True:
						break
			
					#tracker.lookAt(trajComplete2[i][0][j], frame, maxSpeed, useWholeBody)
					#tracker.pointAt(effector, trajComplete2[i][0][j], frame, maxSpeedHead)
					result          = motionProxy.getPosition(whichHand, frame, useSensorValues)
					#robot_hand_pose_pub.publish(result)		
					print result
					handPose = changeToPose(result)
					#print handPose
					robot_hand_pose_pub.publish(handPose)	
					

					if j == 0:
						#wordProc.readFromWordMatrix(correctPermission, j, i+1, REDCardAllert)
						if REDCardAllert == True:
							break
						#wordProc.readFromMatrixLine(correctPermission, i+1, REDCardAllert)
						print "correct permission"
						print correctPermission
						x = True
						if REDCardAllert == True:
							break
					if (j ) %4 == 0:

						correctFlag = wordProc.readFromWordMatrix(correctPermission, j/4, i+1, breakPoint, REDCardAllert, taskLevel_dict['CurrLevel'], ARTag, current_word_pub)
						currReadingPoint = [i+1, j/4]
						print "j"
						print j
						print "j/4"
						print j/4
		
						#if j/4 == (hardWord-1):
							#stopAndAsk()
							#time.sleep(2)
						time.sleep(0.4)

			#correctPermission = True

			if REDCardAllert == True:
				correctPermission = "Partial Correction"
				breakPoint = currReadingPoint
				print "breakPoint"
				print breakPoint
				break


			if correctFlag == "Partial Correction":
				correctPermission = "Partial Correction"
				breakPoint = [0, 0]


			elif correctFlag == True:
				correctPermission = True
				breakPoint = [0, 0]



			print "correctPermission"
			print correctPermission
			print "breakPoint"
			print breakPoint





		elif taskLevel_dict['CurrLevel'] == "TaskTHREE":

			#correctFlag = wordProc.toCorrectOrNotTo(correctPermission, j/4, i+1, breakPoint, REDCardAllert, taskLevel_dict['CurrLevel'], ARTag)
			if readOrNot == 1:
				for i in range(len(trajComplete2)):

					for j in range(len(trajComplete2[i][0])):
						if REDCardAllert == True:
							break

						#tracker.lookAt(trajComplete2[i][0][j], frame, maxSpeed, useWholeBody)
						#tracker.pointAt(effector, trajComplete2[i][0][j], frame, maxSpeedHead)
						result          = motionProxy.getPosition(whichHand, frame, useSensorValues)
						print result
						#robot_hand_pose_pub.publish(result)		
						print result
						handPose = changeToPose(result)
						#print handPose
						robot_hand_pose_pub.publish(handPose)	
						
						if j == 0:
							if REDCardAllert == True:
								break
							print "correct permission"
							print correctPermission
							x = True

						if j % 4 == 0:

							correctFlag = wordProc.readFromWordMatrix(correctPermission, j/4, i+1, breakPoint, REDCardAllert, taskLevel_dict['CurrLevel'], ARTag, current_word_pub)
							currReadingPoint = [i+1, j/4]
							print "j", j
							print "j/4", j/4
							time.sleep(0.4)

			elif readOrNot == 0:
				animationSelection = MOTION_ANIMATION_SELECTION()
				animationSelection.reactionToSpecificTags()
				correctFlag = True

			if REDCardAllert == True:
				correctPermission = "Partial Correction"
				breakPoint = currReadingPoint
				wordProc.saveAndUpdate(ARTag, readOrNot)
				print "breakPoint"
				print breakPoint
				break

			global robot_state_pub
			robot_state_pub.publish("Reading Mode Ended")
			
			wordProc.saveAndUpdate(ARTag, readOrNot)

			if correctFlag == "Partial Correction":
				correctPermission = "Partial Correction"
				breakPoint = [0, 0]

			elif correctFlag == True:
				correctPermission = True
				breakPoint = [0, 0]

			elif correctFlag == False:
				correctPermission = False



				
			print "correctPermission"
			print correctPermission
			print "breakPoint"
			print breakPoint



		time.sleep(0.2)		
		# Empty the arrays
		pitch_angle = 0.3
		LookAtTheBook(pitch_angle)

		if skipOrNot == 0:
			time.sleep(0.7)
			print "NotSkip"
			if correctPermission == False:
				valuationAfterFirst() 

			elif correctPermission == True:
				valuationAfterRepeat()

		trajComplete2 = []
		wordProc.cleanStory()
		changeCoordinate.clearCoordinate()
		reactionPermission = True
		blinkingModeON("ON")
		idleMovementModeON("ON")

def valuationAfterFirst():
	Num = random.randint(1,5)

	if Num == 1:
		story.post.say("\\rspd=90\\ did I read it \\pau=40\\ correctly?") 

	if Num == 2:
		story.post.say("\\rspd=80\\ what do you think? \\pau=40\\ was I good?")

	if Num == 3:
		story.post.say("\\rspd=80\\ Should I read it again?")

	if Num == 4:
		story.post.say("\\rspd=80\\ can I know your opinion?")

	if Num == 5:
		story.post.say("\\rspd=80\\ I'm done, was it good?")

	if Num == 6:
		story.post.say("\\rspd=80\\ did I read it \\pau=50\\ correctly?")



def valuationAfterRepeat():
	Num = random.randint(1,5)

	if Num == 1:
		story.post.say("\\rspd=80\\ what about \\pau=20\\ this time?")  

	if Num == 2:
		story.post.say("\\rspd=80\\ I hope I read it correctly?")

	if Num == 3:
		story.post.say("\\rspd=80\\ did I correct my mistake?")

	if Num == 4:
		story.post.say("\\rspd=80\\ Did I make a mistake again?")

	if Num == 5:
		story.post.say("\\rspd=80\\ can I know your opinion again?")



def stopAndAsk():

	story.say("\\rspd=80\\ Oh")
	story.say("\\rspd=80\\ I don't know how to read this word") 
	story.say("\\rspd=80\\ can you help me?")
	# activate Speech recognition
	# record the audio
	# get the text back 
	if empty:
		story.say("\\rspd=80\\ sorry I couldn't hear you?")

	if full:
		if confidenceLevel >= threshold:
			story.say("\\rspd=80\\ Oh! now I understood. It is " + recognizedWord + ".")
		elif confidenceLevel < threshold:
			story.say("\\rspd=80\\ did you say" + recognizedWord + "?")





def getTagLocations(msg):
	
	global trajComplete
	#trajComplete = [1]
	global ARTag
	changeCoordinate = TRANSFORMATION(motionProxy)
	global moveHand
	moveHand += 1
	cameraName = 'CameraBottom'
	global whichPage
	#whichPage = "Left"



	if moveHand == 1:
		k = 0
		if whichPage == "Left":
			k = 0
			effector = "LArm"

		elif whichPage == "Right":
			k = 2
			effector = "RArm"


		if(effector == "LArm"):
			motionProxy.openHand("LHand")
		else:
			motionProxy.openHand("RHand")


		
		P1 = []
		P1.append([msg.poses[k].position.x])
		P1.append([msg.poses[k].position.y])
		P1.append([msg.poses[k].position.z])
		P1.append([1])
		P2 = []
		P2.append([msg.poses[k+1].position.x])
		P2.append([msg.poses[k+1].position.y])
		P2.append([msg.poses[k+1].position.z])
		P2.append([1])


		""" Transformation Matrix"""
		#tranAC = changeCoordinate.transformMatrix(cameraName)
		newP1 = changeCoordinate.transformPoint(P1, cameraName)
		newP2 = changeCoordinate.transformPoint(P2, cameraName)

		""" to make sure the robot's hand move from left to write
			start from the tag located in the left toward the one in the right
			compare their y value
		"""
		pointsAB = []
		if newP1[1] >= newP2[1]:
			pointsAB.append(newP1)
			pointsAB.append(newP2)
		else:
			pointsAB.append(newP2)
			pointsAB.append(newP1)
		


		calculateWhereToPointAt(pointsAB, effector, space, changeCoordinate)

		#time.sleep(2)
		postureProxy.goToPosture("Crouch", 0.5)
		pitch_angle = 0.3
		LookAtTheBook(pitch_angle)
		pairs_dict[tag_pairs] = True
		print pairs_dict[tag_pairs]



def readAndMoveInstruction( cameraName, handMovePermission, detected_tag):

	global reactionPermission
	global ARTag
	ARTag = []
	ARRec = detected_tag
	ARRec2 = ARRec.replace('[', '').replace(']', '')
	foundAR = re.split(",", ARRec2)
	print foundAR
	if len(foundAR) == 2:
		tagNum = 2
		ARTag.append(foundAR[0] +","+foundAR[1])

	elif len(foundAR) == 4:
		tagNum = 4
		ARTag.append(foundAR[0] +","+foundAR[1])
		ARTag.append(foundAR[2] +","+foundAR[3])
	print "tagNum"
	print tagNum
	print ARTag
	#wordProc = WORDPROCESSING(story, ARTag)
	motionProxy.setBreathEnabled('Arms', False)
	#motionProxy.setBreathEnabled('Head', False)
	#wordProc.readTheTaggedStory(selectedStory, correctPermission)
	if handMovePermission == True:
		reactionPermission = False
		blinkingModeOFF()
		idleMovementModeOFF()


		rospy.Subscriber("target_pose", PoseArray, getTagLocations)

		#motionProxy.setBreathEnabled('Arms', True)

global delaytime
delaytime = 0


def tagDetection(msg):
	"""

	"""
	#faceTrackingEnded()
	global delaytime
	#global wordCount
	global moveHand
	global correctPermission
	global reactionPermission
	global REDCardAllert
	global whichPage
	global breakPoint
	global NumOfGreenCard
	global ARRec
	global robot_state_pub
	#global effector
	animationSelection = MOTION_ANIMATION_SELECTION()
	
	# initializing classes

	taskSelection()
	cameraName = 'CameraBottom'

	#if taskLevel_dict['CurrLevel'] == "TaskFINISH":
		#break

	# Tilt the roobot's head to the front
	pitch_angle = 0.3
	global counter
	if counter == 0:
		yaw_angle = 0.3
		LookAtTheBook(pitch_angle)
		counter = 1


	ARRec = msg.data
	#print "ARRec"
	#print ARRec
	#robot_state_pub.publish("Tags " + ARRec + " Detected")
	ARRec2 = ARRec.replace('[', '').replace(']', '')
	foundAR = re.split(",", ARRec2)
	if len(foundAR) == 2:
		tagNum = 2
		RecTag = msg.data
		tagLeft = msg.data


	elif len(foundAR) == 4:
		tagNum = 4
		tagLeft	 = '[' + foundAR[0] + "," + foundAR[1] + ']'
		tagRight = '[' + foundAR[2] + "," + foundAR[3] + ']'

		#print "left right"
		#print tagLeft, tagRight
		if (pairs_dict[tagLeft] == True or pairs_dict[ARRec] == "Switch" or pairs_dict[ARRec] == "Right"):
			print "I'm in  right"
			RecTag = tagRight
			whichPage = "Right"
			pairs_dict[ARRec] = "Right"
			#effector == "RArm"
		else:
			RecTag = tagLeft
			whichPage = "Left"
			pairs_dict[ARRec] = "Left"
			delaytime += 1
			#effector == "LArm"
			print "I'm in left "
		#print delaytime



	for i in pairs_dict:
		if not (i == RecTag or i == tagLeft):
			if pairs_dict[i] == "Waiting for Red-card":
				pairs_dict[i] = False

			elif pairs_dict[i] == True:
				pairs_dict[i] = "Corrected Mode"

			elif pairs_dict[i] == "Waiting for Green-card":
				pairs_dict[i] = "Corrected Mode"

			#elif pairs_dict[i] == "Correct Happy Mode":
				#pairs_dict[i] = "Corrected Mode"

			#elif pairs_dict[i] == "Wrong Happy Mode":
				#pairs_dict[i] = False
			
			elif pairs_dict[i] == "Repeat Correct Mode":
				pairs_dict[i] = "Corrected Mode"

			elif pairs_dict[i] == "Repeat Wrong Mode":
				pairs_dict[i] = False

			elif pairs_dict[i] == "Repeated Correct Mode":
				pairs_dict[i] = "Corrected Mode"	

			elif pairs_dict[i] == "Repeated Wrong Mode":
				pairs_dict[i] = False					



	rospy.Subscriber('card_id_state', String, cardDetection, RecTag)
	print pairs_dict[RecTag]
	print pairs_dict[ARRec]
	print effector 
	#print pairs_dict
	#print reactionPermission


	if pairs_dict[RecTag] == False:
		moveHand = 0
		print "False added"
		correctPermission = False
		handMovePermission = True
		readAndMoveInstruction(cameraName, handMovePermission, RecTag)
		reactionPermission = True
		REDCardAllert = False
		# story.say(selectedStory)
		pairs_dict[RecTag] = "Waiting for Red-card"



	elif pairs_dict[RecTag] == "Correcting Mode":
				
		if reactionPermission == True:
			animationSelection.reactionToREDCard()
			moveHand = 0
			print "Correcting Mode added"
			#correctPermission = True
			handMovePermission = True
			REDCardAllert = False
			readAndMoveInstruction(cameraName, handMovePermission, RecTag)
			pairs_dict[RecTag] = "Waiting for Green-card"
			if correctPermission == "Partial Correction":
				pairs_dict[RecTag] = "Waiting for Red-card"
				#breakPoint = [0, 0]
			elif correctPermission == True:
				breakPoint = [0, 0]
				pairs_dict[RecTag] = "Waiting for Green-card"

		
		elif reactionPermission == False:
			animationSelection.reactionToREDCard()
			moveHand = 0
			print "Correcting Mode added"
			#correctPermission = True
			handMovePermission = True
			REDCardAllert = False
			readAndMoveInstruction(cameraName, handMovePermission, RecTag)
			pairs_dict[RecTag] = "Waiting for Red-card"
		#wordProc.readTheTaggedStory(selectedStory, True)
		#pairs_dict[msg.data] = True
		#pairs_dict[RecTag] = "Waiting for Green-card"


	elif pairs_dict[RecTag] == "Correct Happy Mode":

		pairs_dict[RecTag] = True
		motionProxy.setBreathEnabled('Arms', False)
		if reactionPermission == True:
			NumOfGreenCard += 1
			animationSelection.reactionToGREENCard()
		#story.say("\\rspd=70\\ Lets go to the next page \\pau=500\\ ")


	elif pairs_dict[RecTag] == "Wrong Happy Mode":
		pairs_dict[RecTag] = True
		motionProxy.setBreathEnabled('Arms', False)
		if reactionPermission == True:
			animationSelection.reactionToGREENCard()
		#story.say("\\rspd=70\\ Lets go to the next page \\pau=500\\ ")

	

	elif pairs_dict[RecTag] == "Corrected Mode":
		
		moveHand = 0
		print "corrected Mode added"
		correctPermission = True
		handMovePermission = True
		REDCardAllert = False
		readAndMoveInstruction(cameraName, handMovePermission, RecTag)
		#pairs_dict[msg.data] = True
		pairs_dict[RecTag] = "Waiting for Green-card"



	elif pairs_dict[RecTag] == "Repeat Correct Mode":
				
		moveHand = 0
		print "Repeat Correct Mode added"
		correctPermission = True
		handMovePermission = True
		REDCardAllert = False
		readAndMoveInstruction(cameraName, handMovePermission, RecTag)
		pairs_dict[RecTag] = "Repeated Correct Mode"



	elif pairs_dict[RecTag] == "Repeat Wrong Mode":
				
		moveHand = 0
		print "Repeat Wrong Mode added"
		correctPermission = False
		handMovePermission = True
		REDCardAllert = False
		readAndMoveInstruction( cameraName, handMovePermission, RecTag)
		pairs_dict[RecTag] = "Repeated Wrong Mode"



	elif pairs_dict[RecTag] == "Repeated Wrong Mode" :

		moveHand = 1
		pairs_dict[RecTag] = "Waiting for Red-card"



	elif pairs_dict[RecTag] == "Repeated Correct Mode" :

		moveHand = 1
		pairs_dict[RecTag] = "Waiting for Green-card"



def cardDetection(msg, tag):
	global cardPosition
	global REDCardAllert
	global NumOfGreenCard
	global correctPermission
	global ARRec
	global detected_tags_pub

	red_card = "40"
	green_card = "39"
	yellow_card = "38"

	card_id = msg.data
	global red_counter
	#print red_counter
	if card_id == red_card:
		
		detected_tags_pub.publish("RED Card Detected")

		if pairs_dict[tag] == "Waiting for Red-card":
			pairs_dict[tag] = "Correcting Mode"
			REDCardAllert = True
			if correctPermission == False:
				correctPermission = True
			#correctPermission = True

		elif pairs_dict[tag] == "Waiting for Green-card":
			pairs_dict[tag] = "Repeat Correct Mode"
			#reac = 2
			#motionProxySelection(reac)

	elif card_id == yellow_card:
		detected_tags_pub.publish("YELLOW Card Detected")

		if pairs_dict[ARRec] == "Right":
			pairs_dict[ARRec] = "Switch"
			print "SwitchSwitchSwitchSwitchSwitchSwitchSwitchSwitchSwitchSwitchSwitchSwitchSwitchSwitchSwitch"


		if pairs_dict[tag] == False:
			pairs_dict[tag] = "Repeat Wrong Mode"

		elif pairs_dict[tag] == "Waiting for Red-card":
			pairs_dict[tag] = "Repeat Wrong Mode"

		elif pairs_dict[tag] == "Waiting for Green-card":
			pairs_dict[tag] = "Repeat Correct Mode"

		elif pairs_dict[tag] == "Corrected Mode":
			pairs_dict[tag] = "Repeat Correct Mode"

		elif pairs_dict[tag] == True:
			pairs_dict[tag] = "Repeat Correct Mode"	


	elif card_id == green_card:
		detected_tags_pub.publish("GREEN Card Detected")

		if pairs_dict[tag] == "Waiting for Green-card" :
			#red_counter = 0
			if correctPermission == True:
				pairs_dict[tag] = "Correct Happy Mode"

			elif correctPermission == "Partial Correction":
				pairs_dict[tag] = "Wrong Happy Mode"

		elif pairs_dict[tag] == "Waiting for Red-card":
			pairs_dict[tag] = "Wrong Happy Mode"
			REDCardAllert = False
			#reac = 2
	cardPosition = 0

	#rospy.Subscriber('card_pose', Pose, cardPointedAt)

	
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
	global robot_state_pub
	#idleMovementMode("OFF")
	robot_state_pub.publish("Reaction Mode Started")
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
	robot_state_pub.publish("Reaction Mode Ended")
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

global skip
skip = 0 

def idleMovementModeON(mode):
	global skip
	global idleThread
	idleThread = threading.Timer(35, idleMovementModeON, [mode])
	animationSelection = MOTION_ANIMATION_SELECTION()
	idleThread.start()
	skip += 1
	#print k
	if mode == "ON":		
		#emotionReaction.blink_eyes()
		if skip >= 2:
			global robot_state_pub
			robot_state_pub.publish("Idle Mode Reaction Started") 
			motionProxy.setBreathEnabled('Arms', False)
			animationSelection.reactionIdleMovement()
			motionProxy.setBreathEnabled('Arms', True)
			robot_state_pub.publish("Idle Mode Reaction Ended") 
		


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
		


def twoPageModeOFF():
	global idleThread
	global k
	#emotionReaction.turn_off_eye()
	idleThread.cancel()
	k = 0 
	


def faceTrackingStarted(faceSize):
	""" Robot starts to track the users face

	"""

	# First, wake up
	#motionProxy.wakeUp()
	#motionProxy.rest()

	# Add target to track
	targetName = "Face"
	faceWidth = faceSize
	tracker.registerTarget(targetName, faceWidth)

	# Then, start tracker
	tracker.track(targetName)


def faceTrackingEnded():
	""" Robot stops to track the users face and go into resting mode after certain keys are pressed

	"""

	tracker.stopTracker()
	tracker.unregisterAllTargets()
	#motionProxy.rest()


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
	
	global robot_state_pub
	robot_state_pub.publish("Looking at The Book")

	time.sleep(0.5)
	motionProxy.setStiffnesses("Head", 0.0)


def IntroduceNao():
	"""
	Nao starts introducing itself when the book cover is in front of him 
	"""
	global robot_state_pub
	global blinkThread 

	robot_state_pub.publish("Introduce Nao Started")
	# First, wake up
	#motionProxy.wakeUp()
	postureProxy.goToPosture("Crouch", 0.5)
	turn_on_eye()
	motionProxy.setBreathEnabled('Body', False)
	motionProxy.setBreathEnabled('Arms', True)
	blinkingModeON("ON")
	#idleMovementModeON("ON")
	mode = "ON"

	#motionProxy.setBreathEnabled('Head', True)
	#motionProxy.rest()


	story.setLanguage('English')
	#story.say("\\rspd=90\\ Hello \\pau=500\\ My name is nao \\pau=500\\ I really like reading short stories")
	#story.say("\\rspd=90\\ Do you want to listen to them?")
	#story.say("\\rspd=90\\ sometimes I make mistakes, can you help me to correct them?")
	#time.sleep(0.1)
	#story.say("\\rspd=90\\ If you want to read with me, please bring the book")
	#story.say("\\rspd=90\\ and don't forget the red, green and yellow card")
	#story.say("\\rspd=90\\ \\pau=50\\ you can show me the red card if I make a mistake")
	#story.say("\\rspd=90\\ \\pau=50\\ the green card when I'm correct")
	#story.say("\\rspd=90\\ \\pau=50\\ and the yellow one, when you want me to repeat")
	#story.say("\\rspd=90\\ Hello")
	pitch_angle = 0.2
	#LookAtTheBook(pitch_angle)
	time.sleep(0.5)
	idleMovementModeON("ON")
	robot_state_pub.publish("Introduce Nao Ended")



def taskSelection():
	global NumOfGreenCard
	#print "NumOfGreenCard"
	#print NumOfGreenCard

	taskLevel_dict['PrevLevel'] = taskLevel_dict['CurrLevel'] 
	#print "NumOfGreenCard"
	#print NumOfGreenCard
	#print "CurrLevel "
	#print taskLevel_dict['CurrLevel'] 
	if NumOfGreenCard == 0:
		if taskLevel_dict['PrevLevel'] == "TaskNONE":
			taskLevel_dict['CurrLevel'] = "TaskONE"

	if NumOfGreenCard == 3:
		NumOfGreenCard = 0
		if taskLevel_dict['PrevLevel'] == "TaskNONE":
			taskLevel_dict['CurrLevel'] = "TaskONE"

		elif taskLevel_dict['PrevLevel'] == "TaskONE":
			taskLevel_dict['CurrLevel'] = "TaskTWO"

		elif taskLevel_dict['PrevLevel'] == "TaskTWO":
			taskLevel_dict['CurrLevel'] = "TaskTHREE"

		elif taskLevel_dict['PrevLevel'] == "TaskTHREE":
			taskLevel_dict['CurrLevel'] = "TaskFINISH"

	if taskLevel_dict['PrevLevel'] != taskLevel_dict['CurrLevel']:
		global robot_state_pub
		robot_state_pub.publish( taskLevel_dict['CurrLevel'] + " Selected")

#def restartCoReader():


def TurnOffCoReader():

	motionProxy.setBreathEnabled('Body', False)
	motionProxy.setBreathEnabled('Arms', False)
	motionProxy.killAll()


def main():

	rospy.init_node('read_lines_location_event')
	global robot_state_pub
	robot_state_pub = rospy.Publisher("robot_state", String, queue_size=5)
	robot_state_time_pub = rospy.Publisher("robot_state_time", Pose, queue_size=5)
	global current_word_pub
	current_word_pub = rospy.Publisher("current_word", String, queue_size=5)
	current_word_time_pub = rospy.Publisher("current_word_time", Pose, queue_size=5)
	global detected_tags_pub
	detected_tags_pub = rospy.Publisher("detected_tags", String, queue_size=5)
	global robot_hand_pose_pub
	robot_hand_pose_pub = rospy.Publisher("robot_hand_pose", Pose, queue_size=10)
	robot_hand_pose_time_pub = rospy.Publisher("robot_hand_pose_time", Pose, queue_size=5)

	#nao_IP = rospy.get_param('~nao_ip')
	nao_IP = 'nao.local'
	
	global story
	story = ALProxy("ALTextToSpeech", nao_IP, 9559)
	
	global conversration
	conversation = ALProxy("ALTextToSpeech", nao_IP, 9559)

	global motionProxy
	motionProxy = ALProxy("ALMotion", nao_IP, 9559)

	global postureProxy
	postureProxy = ALProxy("ALRobotPosture", nao_IP, 9559)
	
	global audiomotionProxy
	audiomotionProxy = ALProxy("ALAudioPlayer", nao_IP, 9559)
	
	global tracker
	tracker = ALProxy("ALTracker", nao_IP, 9559)

	global proxy
	proxy = ALProxy("ALLeds", nao_IP, 9559)

	global audioReactionProxy
	audioReactionProxy = ALProxy("ALAudioPlayer", nao_IP, 9559)

	# Initialize the node


	logFile = logger()

	#storyNum = 1
	#pub = rospy.Publisher('story_number', String, queue_size=1)
	#pub.publish(str(storyNum))
	#storySelection()

	# Face tracking activated
	faceTrackingEnded()
	facesize = 0.1
	global restingEnabled 
	restingEnabled = False
	faceTrackingStarted(facesize)
	
	global space
	space      = motion.FRAME_ROBOT

	# Select a story and activity level
	global selectedStory
	#selectedStory = storySelection()
	correctFlag = False
	activityLevel = rospy.get_param('actlevel', 'medium')
	#mistakeNum = numberOfMistakesSelection(activityLevel)

	time.sleep(1)
	IntroduceNao()

	#rospy.Subscriber('tag_id_state', String, IntroduceNao)
	faceTrackingEnded()
	#time.sleep(1)
	pitch_angle = 0.3
	yaw_angle = 0.8
	LookAtTheBook(pitch_angle)
	


	"""waitAndRecord = RECORDANDTRANSCRIBE()
	waitAndRecord.recordTheOutput('new.wav')
	response = waitAndRecord.transcribeTheOutput()
	#print recordedWord
	reWord = response["results"][0]["alternatives"][0]["transcript"]
	reConf = response["results"][0]["alternatives"][0]["confidence"]

	repeatWord = '\\rspd=80\\' + str(reWord)
	story.say(repeatWord)

	print reConf
	print reWord"""

	rospy.Subscriber('tag_id_state', String, tagDetection)

	
	
	try:
		while True:
			time.sleep(1)
    	except restingEnabled == True:
        	print
        	print "Interrupted by user, shutting down"
        	myBroker.shutdown()
        	sys.exit(0)



if __name__ == "__main__":


	main()