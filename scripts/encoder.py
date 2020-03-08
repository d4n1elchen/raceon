#!/usr/bin/python3

import rospy
from raceon.msg import Encoder

import RPi.GPIO as GPIO
import time

class EncoderNode():

    def __init__(self):
        self.topic_name_encoder = rospy.get_param("topic_name_encoder", "encoder")

        self.encoder_left = 0
        self.encoder_right = 0
        self.encoder_left_cnt = 0
        self.encoder_right_cnt = 0

        #GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(27,  GPIO.IN)
        GPIO.setup(25, GPIO.IN)
        GPIO.add_event_detect(27,  GPIO.BOTH, callback=self)
        GPIO.add_event_detect(25, GPIO.BOTH, callback=self)

    def start(self):
        self.pub_encoder = rospy.Publisher(self.topic_name_encoder, Encoder, queue_size=10)
        while True:
            self.countEncoder()

    def __call__(self, channel):
        
        if channel == 27:
            self.encoder_left_cnt  += 1
        if channel == 25:
            self.encoder_right_cnt += 1

    def countEncoder(self):
        time.sleep(0.03)
        encoder_left, encoder_right = self.encoder_left_cnt, self.encoder_right_cnt
        self.encoder_left_cnt = 0
        self.encoder_right_cnt = 0
        
        encoder_msg = Encoder()
        encoder_msg.left = encoder_left
        encoder_msg.right = encoder_right
        self.pub_encoder.publish(encoder_msg)

if __name__ == "__main__":
    rospy.init_node("Encoder")
    encoder = EncoderNode()
    try:
        encoder.start()
    except rospy.ROSInterruptException:
        pass





