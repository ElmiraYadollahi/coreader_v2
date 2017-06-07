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

from RTClass import RECORDANDTRANSCRIBE

instruction_tags_dict = {	'=LineNum',
							'=wordNum',
							'=MistakeNum',
							'=NextLine',
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

toCorrect_DICT = {}
trash_DICT = {}

class WORDPROCESSING:
	""" Processing the text and read """

	def __init__(self, stor, ARTag, taskLevel, proxy):

		self.tag = "=WordNum"
		self.selectedStory = self.storySelection(ARTag, taskLevel)
		print "self.selectedStory"
		print self.selectedStory
		self.wordMatrix= []
		self.lineMatrix = self.getTheLineMatrix()
		print "self.lineMatrix"
		print self.lineMatrix
		self.LineWordCount = []
		self.story = stor
		self.waitAndRecord = RECORDANDTRANSCRIBE(proxy)
		self.corrected_DICT = {}
		#self.current_word_pub = current_word_pub
		#self. = {}

	def storySelection(self, tag, taskLevel):
		""" Select a story from the text files of each story and return it

		"""

		
		""" creates a array containing the line related to detected tags """
		#with open('chick_story_en.txt') as f:
			#lines_array = f.read().splitlines()

		print taskLevel
		with open('all_stories_en.txt') as f:
			lines_array = f.read().splitlines()

		if taskLevel['CurrLevel'] == "TaskONE":
			with open('all_stories_one_en.txt') as f:
				lines_array = f.read().splitlines()

		if taskLevel['CurrLevel'] == "TaskTWO":
			with open('all_stories_two_en.txt') as f:
				lines_array = f.read().splitlines()

		if taskLevel['CurrLevel'] == "TaskTHREE":
			with open('all_stories_three_en.txt') as f:
				lines_array = f.read().splitlines()


		#for line in lines_array:
			#print(re.split('\W+', line))
			#print(word)
		#print "lines_array"
		#print lines_array
		#print "words_array"
		#print words_array

		#story_loaded = rospy.get_param('~story_text_en')
		#lines_array = story_loaded.splitlines()
		#print lines_array

		for line in lines_array:
			found = re.search(tag, line)
			if found != None:
				self.selectedStory = line[:found.start()] + line[found.end():]
				self.selectedStory = self.selectedStory.replace('[', '').replace(']', '')
				break

		#for line in lines_array:
		#print(re.split('\W+,', self.selectedStory))
		#wordCount = self.getTheWordCount()
		#print "story selection"
		#print self.selectedStory

		return self.selectedStory	


	def getTheLineMatrix(self):
		""" Reads the number of words given in the text and then remove the tag and number and returns wordCount

		"""		
		
		tag = "=NextLine"
		tagWithWord = "\w+(?=" + tag + ")"
		self.lineMatrix = re.split( tagWithWord + tag, self.selectedStory)
		#for line in self.lineMatrix:
			#self.wordMatrix.append(re.split('\W+,', line))

		#print "words_array"
		#print self.wordMatrix
		#print "line matrix"
		#print self.lineMatrix
		#wordCountString = foundTag.group(0)
		#wordCountString = self.removeTheTag(tag, wordCountString)

		#wordCount = int(wordCountString)
		#self.selectedStory = self.removeTheWordWithTag(tag, self.selectedStory)

		#print "get the word count"
		#print self.selectedStory
		return self.lineMatrix
		#return wordCount

	def getNumberOfWords(self, lineCount):

		dummyWordMatrix = []
		dummyWordCount = []
		for line in self.lineMatrix:
			dummyWordMatrix.append(re.split('\W+,', line))

		for i in range(lineCount):
			dummyWordCount.append(len(dummyWordMatrix[i+1]))

		return dummyWordCount




	def getTheInstructionTagData(self, INTag):
		"""

		"""

		#print "in tag"
		#print self.selectedStory
		tag = INTag
		tagWithWord = "\w+(?=" + tag + ")"
		foundTag = re.search(tagWithWord + tag, self.selectedStory)

		instructionString = foundTag.group(0)
		instructionString = self.removeTheTag(tag, instructionString)
		if INTag == "=S1" or INTag == "=S2" or INTag == "=S3" or INTag == "=S4" or INTag == "=S5":
			foundNP = re.search("N", instructionString)
			if foundNP == None:
				foundNP = re.search("P", instructionString)
				NPString = foundNP.group(0)
				NPString = self.removeTheTag("P", instructionString)
				instruct = int(NPString)
			else:
				NPString = foundNP.group(0)
				print NPString
				NPString = self.removeTheTag("N", instructionString)
				instruct = -int(NPString)
				print instruct
			
		else:
			instruct = int(instructionString)
		self.selectedStory = self.removeTheWordWithTag(tag, self.selectedStory)



		return instruct


	def clearAllTheInstructionTags(self):
		"""

		"""

		for inTag in instruction_tags_dict:
			if re.search(inTag, self.selectedStory) != None:
				self.selectedStory = self.removeTheWordWithTag(inTag, self.selectedStory)

		for inTag in instruction_tags_dict:
			for i in range(len(self.lineMatrix)):
				if re.search(inTag, self.lineMatrix[i]) != None:
					self.lineMatrix[i] = self.removeTheWordWithTag(inTag, self.lineMatrix[i])

		#print "line matrix clear"
		#print self.lineMatrix


	def removeTheTag(self, tag, storyContent):
		""" Find and remove the tag given to the function and leave the word connected to them intact

		"""

		while True:
			foundTag = re.search(tag, storyContent)
			if foundTag == None:
				break
			storyContent = storyContent[:foundTag.start(0)] + storyContent[foundTag.end(0):]

		return storyContent


	def removeTheWordWithTag(self, tag, storyContent):
		""" Find and remove the tag given to the function and remove the tag and the word connected to it as well

		"""

		while True:
			tagWithWord = "\w+(?=" + tag + ")"
			foundTag = re.search(tagWithWord + tag, storyContent)
			if foundTag == None:
				break
			storyContent = storyContent[:foundTag.start(0)] + storyContent[foundTag.end(0):]

		return storyContent

	def readFromMatrixLine(self, correctFlag, line, REDCardAllert):
		"""

		"""
		#print "Flag"
		#print correctFlag

		#print "line matrix before"
		#print self.lineMatrix

		eachLine = self.lineMatrix[line]


		if correctFlag == True:
			tag = "=RTag"
			eachLine = self.removeTheTag(tag, eachLine)
			tag = "=WTag"
			eachLine = self.removeTheWordWithTag(tag, eachLine)
			print "I'm in True"


		elif correctFlag == False:
			tag = "=WTag"
			eachLine = self.removeTheTag(tag, eachLine)
			tag = "=RTag"
			eachLine = self.removeTheWordWithTag(tag, eachLine)
			print "I'm in False"


		elif correctFlag == "Partial Correction":
			
			if lineWithMistake == line:
				tag = "=WTag"
				eachLine = self.removeTheTag(tag, eachLine)
				tag = "=RTag"
				eachLine = self.removeTheWordWithTag(tag, eachLine)

			else:
				tag = "=RTag"
				eachLine = self.removeTheTag(tag, eachLine)
				tag = "=WTag"
				eachLine = self.removeTheWordWithTag(tag, eachLine)
			
			print "I'm in Partial correction"

		#print "read line"
		#print line
		#print "line matrix after"
		#print eachLine
		if REDCardAllert == False:
			self.sayFromFile(self.story, eachLine, 'ascii')



	def toCorrectOrNotTo(self, ARTag):

		wrongLocation = []
		#level = 3
		
		for line in self.lineMatrix:
			self.wordMatrix.append(re.split('\W+,', line))

		for i in range(len(self.lineMatrix)):

			for j in range(len(self.wordMatrix[i])):

				wrongWord = self.wordMatrix[i][j]
				tag = "=RTag"
				foundTag = re.search(tag, wrongWord)
				if foundTag:
					XYLocation = []
					XYLocation.append(i)
					XYLocation.append(j)
					wrongLocation.append(XYLocation)

		print "WWWExistExistExistExistExistExistExistExistExistExistExistExistExistExist"

		print "string"
		print str(ARTag)
		print "toCorrect_DICT"
		print toCorrect_DICT

		if str(ARTag) in toCorrect_DICT:

			print "ExistExistExistExistExistExistExistExistExistExistExistExistExistExist"
		else:

			toCorrect_DICT[str(ARTag)] = wrongLocation
			print "NotExistNotExistNotExistNotExistNotExistNotExistNotExistNotExistNotExist"
		
		print "wrongLocation"
		print wrongLocation






	def readFromWordMatrix(self, correctFlag, wordN, lineN, breakPoint, REDCardAllert, level, ARTag, current_word_pub):
		"""

		"""
		#print "Flag"
		#print correctFlag
		#print "line matrix before"
		#print self.lineMatrix
		correctedWord =[]
		#level = 3

		wrongLocation = toCorrect_DICT[str(ARTag)]

		print "wrongLocation"
		print wrongLocation
		print "toCorrect_DICT"
		print toCorrect_DICT

		for line in self.lineMatrix:
			self.wordMatrix.append(re.split('\W+,', line))
		
		toCorrectWord =[]
		print "breakPoint"
		print breakPoint
		print level

		if str(ARTag) in trash_DICT:
			trashWord = trash_DICT[str(ARTag)]
		else:
			trashWord = []

		print "trashWord"
		print trashWord

		for i in range(len(wrongLocation)):
			if  breakPoint[0] == wrongLocation[i][0]:
				if breakPoint[1] >= wrongLocation[i][1]:
					toCorrectWord.append(wrongLocation[i])
			elif breakPoint[0] > wrongLocation[i][0]:
				toCorrectWord.append(wrongLocation[i])
			#else:
				#toCorrectWord = toCorrect_DICT[str(ARTag)]
			
			print "toCorrectWord"
			print toCorrectWord



		

		#for i in self.wordMatrix:

			#wrongWord = self.wordMatrix[lineN][wordN]
			#tag = "=RTag"
			#foundTag = re.search(tag, wrongWord)
			#if foundTag != None:
		print "wrongLocation"
		print wrongLocation
		print "wordMatrix"
		print self.wordMatrix
		print len(self.wordMatrix)
		print wordN
		print lineN
		skip = False



		eachWord = self.wordMatrix[lineN][wordN]


		if correctFlag == True:
			toCorrectWord = toCorrect_DICT[str(ARTag)]
			print "toCorrect_DICT[str(ARTag)]"
			print toCorrect_DICT[str(ARTag)]

			if level == "TaskTHREE":

				if [lineN, wordN] in toCorrectWord:
					print [lineN, wordN]
					print "[lineN, wordN]"
					tag = "=RTag"
					eachWord = self.removeTheTag(tag, eachWord)
					tag = "=WTag"
					eachWord = self.removeTheWordWithTag(tag, eachWord)
					correctedWord.append([lineN, wordN])
					print "correctedWord"
					print correctedWord
					self.stopAndAsk(eachWord)
					skip = True
					print "which level"



				elif [lineN, wordN] in trashWord:
					tag = "=RTag"
					eachWord = self.removeTheTag(tag, eachWord)
					tag = "=WTag"
					eachWord = self.removeTheWordWithTag(tag, eachWord)
					print "three level"

				else:
					tag = "=WTag"
					eachWord = self.removeTheTag(tag, eachWord)
					tag = "=RTag"
					eachWord = self.removeTheWordWithTag(tag, eachWord)
					print "three level"

			else: 

				if [lineN, wordN] in toCorrectWord:
					print [lineN, wordN]
					print "[lineN, wordN]"
					tag = "=RTag"
					eachWord = self.removeTheTag(tag, eachWord)
					tag = "=WTag"
					eachWord = self.removeTheWordWithTag(tag, eachWord)
					correctedWord.append([lineN, wordN])
					print "correctedWord"
					print correctedWord
					#self.stopAndAsk(eachWord)
					skip = True
					print "which level"



				elif [lineN, wordN] in trashWord:
					tag = "=RTag"
					eachWord = self.removeTheTag(tag, eachWord)
					tag = "=WTag"
					eachWord = self.removeTheWordWithTag(tag, eachWord)
					print "three level"

				else:
					tag = "=WTag"
					eachWord = self.removeTheTag(tag, eachWord)
					tag = "=RTag"
					eachWord = self.removeTheWordWithTag(tag, eachWord)
					print "three level"


			print "I'm in True"


		elif correctFlag == False:
			tag = "=WTag"
			eachWord = self.removeTheTag(tag, eachWord)
			tag = "=RTag"
			eachWord = self.removeTheWordWithTag(tag, eachWord)
			print "I'm in False"

		
		elif correctFlag == "Partial Correction":

			if breakPoint == [0, 0]:
				toCorrectWord = toCorrect_DICT[str(ARTag)]
				print "toCorrect_DICT[str(ARTag)]"
				print toCorrect_DICT[str(ARTag)]
			
			print "[lineN, wordN]"
			print [lineN, wordN]
			print "toCorrectWord"
			print toCorrectWord
			if level == "TaskTHREE":

				if [lineN, wordN] in toCorrectWord:
					print [lineN, wordN]
					print "[lineN, wordN]"
					tag = "=RTag"
					eachWord = self.removeTheTag(tag, eachWord)
					tag = "=WTag"
					eachWord = self.removeTheWordWithTag(tag, eachWord)
					correctedWord.append([lineN, wordN])
					print "correctedWord"
					print correctedWord
					self.stopAndAsk(eachWord)
					skip = True
					print "which level"



				elif [lineN, wordN] in trashWord:
					tag = "=RTag"
					eachWord = self.removeTheTag(tag, eachWord)
					tag = "=WTag"
					eachWord = self.removeTheWordWithTag(tag, eachWord)
					print "three level"

				else:
					tag = "=WTag"
					eachWord = self.removeTheTag(tag, eachWord)
					tag = "=RTag"
					eachWord = self.removeTheWordWithTag(tag, eachWord)
					print "three level"
			
			else:

				if [lineN, wordN] in toCorrectWord:
					print [lineN, wordN]
					print "[lineN, wordN]"
					tag = "=RTag"
					eachWord = self.removeTheTag(tag, eachWord)
					tag = "=WTag"
					eachWord = self.removeTheWordWithTag(tag, eachWord)
					print "wrong level"

					

				else:
					tag = "=WTag"
					eachWord = self.removeTheTag(tag, eachWord)
					tag = "=RTag"
					eachWord = self.removeTheWordWithTag(tag, eachWord)
					print "not three level"
			
			print "I'm in Partial correction"

		self.wordMatrix = []
		eachWord = '\\rspd=50\\' + eachWord
		#print "read line"
		#print line
		#print "line matrix after"
		#print eachLine
		if REDCardAllert == False:
			current_word_pub.publish(eachWord)
			self.sayFromFile(self.story, eachWord, 'ascii')

		if breakPoint !=[0, 0]:
			if toCorrectWord == wrongLocation:
				correctFlagReturn = True

			elif toCorrectWord != wrongLocation:
				correctFlagReturn = "Partial Correction"

		elif breakPoint == [0, 0]:
			correctFlagReturn = True

		if correctFlag == False:
			correctFlagReturn = False


		print "correctedWord"
		print correctedWord

		self.corrected_DICT[str(ARTag)] = toCorrectWord
		#if str(ARTag) in self.toCorrect_DICT:

			#print "Exist"
		#else:

			#self.toCorrect_DICT[str(ARTag)] = wrongLocation

		#for self.corrected_DICT[str(ARTag)] in toCorrectWord:
			#print "hello hello hello hello hello hello hello hello hello hello hello hello"
			#toCorrectWord.remove(self.corrected_DICT[str(ARTag)])

		print "self.corrected_DICT"
		print self.corrected_DICT
		print self.corrected_DICT[str(ARTag)] 
		print "self.toCorrect_DICT"
		print toCorrect_DICT
		print toCorrect_DICT[str(ARTag)] 





		print self.corrected_DICT
		print toCorrect_DICT



		return correctFlagReturn


	def saveAndUpdate(self, ARTag, readOrNot):

		print "inside save and update inside save and update inside save and update inside save and update inside save and update"

		if readOrNot ==1:
			check1 = self.corrected_DICT[str(ARTag)]
			if  check1 != []:
				print "hereherehereherehereherehere"
				print check1
				for i in check1: 
					if i in toCorrect_DICT[str(ARTag)]:
						print "finallyfinallyfinallyfinallyfinallyfinallyfinally"
						leftUnCorrected = toCorrect_DICT[str(ARTag)]
						leftUnCorrected.remove(i)
						print "leftUnCorrected"
						print leftUnCorrected
						toCorrect_DICT[str(ARTag)] = leftUnCorrected 
						

						if str(ARTag) in trash_DICT:

							trash_DICT[str(ARTag)].extend(self.corrected_DICT[str(ARTag)])
							print "NotrashNotrashNotrashNotrashNotrashNotrashNotrash"
						else:

							trash_DICT[str(ARTag)] = self.corrected_DICT[str(ARTag)]
							print "trashtrashtrashtrashtrashtrashtrash"


	def stopAndAsk(self, eachWord):
		time.sleep(1)
		self.story.say("\\rspd=90\\ Can you read this word for me? ")

		readCorrectly = False

		while ( readCorrectly != True):

			self.waitAndRecord.recordTheOutput('new.wav')
			response = self.waitAndRecord.transcribeTheOutput()
			print response

			if response != {}:
				reWord = response["results"][0]["alternatives"][0]["transcript"]
				reConf = response["results"][0]["alternatives"][0]["confidence"]


				if reConf > 0.4 and ''.join(sorted(reWord.lower())).strip() == ''.join(sorted(eachWord.lower())).strip():
					repeatWord = '\\rspd=80\\ Oh that is how you read'
					time.sleep(1)
					self.story.say(repeatWord)
					readCorrectly = True
					print reConf, reWord, eachWord

				else:
					self.story.say("\\rspd=90\\ Sorry I didn't understand, can you read it again? ")
					print reConf, reWord, eachWord

			elif response == {}:
					self.story.say("\\rspd=90\\ Sorry I didn't understand, can you read it again? ")
					#print reConf, reWord, eachWord



	def readTheTaggedStory(self, correctFlag):
		""" Read a story containing the tags and based on correctFlag change the tags approprietly

		"""
		
		if correctFlag == True:
			tag = "=RTag"
			self.selectedStory = self.removeTheTag(tag, self.selectedStory)
			tag = "=WTag"
			self.selectedStory = self.removeTheWordWithTag(tag, self.selectedStory)

		elif correctFlag == False:
			tag = "=WTag"
			self.selectedStory = self.removeTheTag(tag, self.selectedStory)
			tag = "=RTag"
			self.selectedStory = self.removeTheWordWithTag(tag, self.selectedStory)
		
		#print "line story"
		#print self.selectedStory

		self.sayFromFile(self.story, self.selectedStory, 'ascii')


	def sayFromFile(self, story, filename, encoding):
		"""

		"""
		
		toSay = filename.encode("utf-8")
		story.post.say(toSay)

	def saveLineWordCount(self, LiWoCount, lineDistance):

		self.__LineWordCount = LiWoCount
		print "self.LineWordCount"
		print self.__LineWordCount
		self.lineDistanceCoef = lineDistance
		print "self.lineDistanceCoef"
		print self.lineDistanceCoef


	def getLineWordCount(self):
		print " get self.LineWordCount"
		print self.__LineWordCount
		return self.__LineWordCount

	def getLineDistanceCount(self):

		return self.lineDistanceCoef

	def cleanStory(self):
		self.selectedStory = []