#!/usr/bin/env python
#coding: utf-8
"""
Created on Fri Jun 2 14:08:00 2017

@author: Elmira
"""

import time
import rospy
import datetime
from std_msgs.msg import Int16, Int32, String, Empty, Float32
from geometry_msgs.msg import Point, PoseStamped, PoseArray, Pose
from letter_learning_interaction.msg import Shape as ShapeMsg
import os
import csv

word = ""
timeWriting = ""
timeResponse = ""
path = ""
repetition = ""
learn = ""
score = ""
child_counter = 1

class logger():
    
    def __init__(self):
        # Cues to evaluate:
        rospy.Subscriber("robot_state", String, self.robot_state_callback)            # Where the child is looking at
        rospy.Subscriber("current_word", String, self.current_word_callback)             # The child is smiling
        rospy.Subscriber("detected_tags", String, self.detected_tags_callback)             # The child is moving while sitting
        rospy.Subscriber("robot_hand_pose", Pose, self.hand_pose_callback)            # The child is getting closer
        rospy.Subscriber("novelty", Float32, self.novelty_callback)             # Something new happen in the scenario
        #rospy.Subscriber("activity_time", Int32, self.time_callback)            # For how long the activity was done
        #rospy.Subscriber("robot_hand_pose", Pose, self.repetitions_callback)    # The number of word repetitions
        rospy.Subscriber("time_response", Float32, self.response_callback)      # Response time till the child writes
        rospy.Subscriber("time_writing", Float32, self.writing_callback)        # Writing time during demostration
        
        rospy.Subscriber("activity", String, self.activity_callback)            # Current activity
        rospy.Subscriber("words_to_write", String, self.word_callback)          # Word to be written
        rospy.Subscriber("new_child", String, self.child_callback)              # New child in da house
        
        rospy.Subscriber("current_demo", ShapeMsg, self.demo_callback)     # Path of the demo
        rospy.Subscriber("current_learn", ShapeMsg, self.learn_word_callback)     # Path of the learn
        rospy.Subscriber("current_score", Float32, self.score_callback)     # Score
            
        rospy.Subscriber("clear_screen", Empty, self.clearscreen_callback)     # clearscreen
        rospy.Subscriber("user_feedback", String, self.userfeedback_callback)     # user_feedback
        rospy.Subscriber("stop_learning", Empty, self.close)     # user_feedback
        
        self.clear_counter = 0
        self.user_feedback_counter =""
        self.prev = "empty"
	
        date_time = str(datetime.datetime.now().strftime("%Y-%m-%d_%H:%M:%S"))
        path = '~/.ros/visionLog/'
        if not os.path.exists(path):
    		os.makedirs(path)
        self.filePath = path + date_time + '.csv'
	
        with open(self.filePath, 'wb+') as csvfile:
            intro = '********************************NEW SESSION********************************'
            wr = csv.writer(csvfile, delimiter='-', quoting=csv.QUOTE_NONE, quotechar='')
            wr.writerow(intro)
        
        # Initialize the node and name it.       
        #rospy.init_node('logger')
                            
        # Simply keeps python from exiting until this node is stopped
        #rospy.spin()

    #################################################################################################
    ### SUBSCRIBERS CALLBACKS
    ######
  
    def child_callback(self, data):
        """ Is called when a new child starts a session"""
        global child_counter
        timestamp = self.getTime()
        
        child_counter = child_counter + 1
        child = "child:" + str(child_counter)

        date_time = str(datetime.datetime.now().strftime("%Y-%m-%d_%H:%M:%S"))
        self.filePath = '~/.ros/visionLog/' + date_time + '.csv'
        
        with open(self.filePath, 'wb') as csvfile:
            intro = "********************************NEW SESSION********************************"
            wr = csv.writer(csvfile, delimiter='-', quoting=csv.QUOTE_NONE, quotechar='')
            wr.writerow(intro)
            wr.writerow([timestamp, child])


    def robot_state_callback(self, data):
        timestamp = self.getTime()
        lookAt = "Robot State: " + ',' + data.data        
        with open(self.filePath, 'a') as csvfile:
            wr = csv.writer(csvfile, delimiter=' ', quoting=csv.QUOTE_MINIMAL)
            wr.writerow([timestamp, lookAt])
        
    def current_word_callback(self, data):
        timestamp = self.getTime()
        smile = "Current Word: "  + ',' + data.data         
        with open(self.filePath, 'a') as csvfile:
            wr = csv.writer(csvfile, delimiter=' ', quoting=csv.QUOTE_MINIMAL)
            wr.writerow([timestamp, smile])
        
    def detected_tags_callback(self, data):
        if self.prev != data.data:
            self.prev = data.data
            timestamp = self.getTime()
            movement = "Detected Cards: " + str(data.data)        
            with open(self.filePath, 'a') as csvfile:
                wr = csv.writer(csvfile, delimiter=' ', quoting=csv.QUOTE_MINIMAL)
                wr.writerow([timestamp, movement])

        
    def hand_pose_callback(self, data):
        timestamp = self.getTime()
        proximity = "Position: " + str(data.position) + '\n' + str(data.orientation)    
        with open(self.filePath, 'a') as csvfile:
            wr = csv.writer(csvfile, delimiter=' ', quoting=csv.QUOTE_MINIMAL)
            wr.writerow([timestamp, proximity])
    
    def novelty_callback(self, data):
        timestamp = self.getTime()
        novelty = "novelty: " + str(data.data)       
        with open(self.filePath, 'a') as csvfile:
            wr = csv.writer(csvfile, delimiter=' ', quoting=csv.QUOTE_MINIMAL)
            wr.writerow([timestamp, novelty])
    
    def repetitions_callback(self, data):
        global repetition
        repetition = "repetition:" + str(data.data)
	timestamp = self.getTime()
	with open(self.filePath, 'a') as csvfile:
            wr = csv.writer(csvfile, delimiter=' ', quoting=csv.QUOTE_MINIMAL)
            wr.writerow([timestamp, repetition])
    
    def response_callback(self, data):
        global timeResponse
        timeResponse = "timeResponse:" + str(data.data)
	timestamp = self.getTime()
	with open(self.filePath, 'a') as csvfile:
            wr = csv.writer(csvfile, delimiter=' ', quoting=csv.QUOTE_MINIMAL)
            wr.writerow([timestamp, timeResponse])

        
    def writing_callback(self, data):
        global timeWriting
        timeWriting = "writeTime:" + str(data.data)
	timestamp = self.getTime()
	with open(self.filePath, 'a') as csvfile:
            wr = csv.writer(csvfile, delimiter=' ', quoting=csv.QUOTE_MINIMAL)
            wr.writerow([timestamp, timeWriting])

    def clearscreen_callback(self, data):
        clear = "clearscreen"
        timestamp = self.getTime()
	self.clear_counter += 1
        with open(self.filePath, 'a') as csvfile:
            wr = csv.writer(csvfile, delimiter=' ', quoting=csv.QUOTE_MINIMAL)
            wr.writerow([timestamp, clear])
    
    def userfeedback_callback(self, data):
	global feedback
	self.user_feedback_counter = self.user_feedback_counter + data.data
        feedback = "user_feedback:"+data.data
        timestamp = self.getTime()
        with open(self.filePath, 'a') as csvfile:
            wr = csv.writer(csvfile, delimiter=' ', quoting=csv.QUOTE_MINIMAL)
            wr.writerow([timestamp, feedback])
    

    def activity_callback(self, data):
        activity = "activity:" + data.data
        timestamp = self.getTime()
        with open(self.filePath, 'a') as csvfile:
            wr = csv.writer(csvfile, delimiter=' ', quoting=csv.QUOTE_MINIMAL)
            wr.writerow([timestamp, activity])
        
    def word_callback(self, data):
        global word
        word = "word:" + data.data
	timestamp = self.getTime()
	with open(self.filePath, 'a') as csvfile:
            wr = csv.writer(csvfile, delimiter=' ', quoting=csv.QUOTE_MINIMAL)
            wr.writerow([timestamp, word])
    
    def score_callback(self, data):
        global score	
        score = "score: " + str(data.data)
	timestamp = self.getTime()
	with open(self.filePath, 'a') as csvfile:
            wr = csv.writer(csvfile, delimiter=' ', quoting=csv.QUOTE_MINIMAL)
            wr.writerow([timestamp, "new_letter", score])
    
    def learn_word_callback(self, data):
	global learn_path
	global learn_shapeType       
	learn_path = data.path
	learn_shapeType = data.shapeType
	timestamp = self.getTime()	
	with open(self.filePath, 'a') as csvfile:
            wr = csv.writer(csvfile, delimiter=' ', quoting=csv.QUOTE_MINIMAL)
            wr.writerow([timestamp, "learn:"+learn_shapeType, learn_path])
        
    def demo_callback(self, data):
	global demo_path
	global demo_shapeType
        demo_path = data.path
	demo_shapeType = data.shapeType
	timestamp = self.getTime()
	with open(self.filePath, 'a') as csvfile:
            wr = csv.writer(csvfile, delimiter=' ', quoting=csv.QUOTE_MINIMAL)
            wr.writerow([timestamp, "demo:"+demo_shapeType, demo_path])
        

    def getTime(self):
        ts = time.time()
        return datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S:%f')

    def close(self,data): 
	timestamp = self.getTime()
	with open(self.filePath, 'a') as csvfile:
            wr = csv.writer(csvfile, delimiter=' ', quoting=csv.QUOTE_MINIMAL)
            wr.writerow([timestamp, "SESSION_OVER"])

def main(): 
    logger()
    
# Main function.   
if __name__ == "__main__":
    main()  
