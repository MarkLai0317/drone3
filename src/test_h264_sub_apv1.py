#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64MultiArray
import av
import apriltag
import cv2
import math
import numpy as np
import threading
import traceback
import time

class StandaloneVideoStream(object):
    def __init__(self):
        self.cond = threading.Condition()
        self.queue = []
        self.closed = False

    def read(self, size):
        self.cond.acquire()
        try:
            if len(self.queue) == 0 and not self.closed:
                self.cond.wait(2.0)
            data = bytes()
            while 0 < len(self.queue) and len(data) + len(self.queue[0]) < size:
                data = data + self.queue[0]
                del self.queue[0]
        finally:
            self.cond.release()
        return data

    def seek(self, offset, whence):
        return -1

    def close(self):
        self.cond.acquire()
        self.queue = []
        self.closed = True
        self.cond.notifyAll()
        self.cond.release()

    def add_frame(self, buf):
        self.cond.acquire()
        self.queue.append(buf)
        self.cond.notifyAll()
        self.cond.release()


stream = StandaloneVideoStream()


def callback(msg):
    #rospy.loginfo('frame: %d bytes' % len(msg.data))
    stream.add_frame(msg.data)

def find_Mask(img):
  lr0 = np.array([0, 70, 0])
  ur0 = np.array([5, 255, 255])
  lr1 = np.array([175, 70, 0])
  ur1 = np.array([180, 255, 255])
  rm0 = cv2.inRange(img, lr0, ur0)
  rm1 = cv2.inRange(img, lr1, ur1)
  rm = cv2.bitwise_or(rm0, rm1)
  return rm

def main():

    fourcc = cv2.VideoWriter_fourcc('X', 'V', 'I', 'D')
    out = cv2.VideoWriter('follow_demo.avi', fourcc, 20.0, (960, 720))
    detector = apriltag.Detector()
    
    rospy.init_node('h264_listener')
    rospy.Subscriber("/tello/image_raw/h264", CompressedImage, callback)

    point_pub = rospy.Publisher("/target_point", Float64MultiArray, queue_size = 10)
    point_ap_pub = rospy.Publisher("/target_ap", Float64MultiArray, queue_size = 10)
    container = av.open(stream)
    rospy.loginfo('main: opened')
    
    passed = False

    start_detect = True
    frame_skip = 300
    for frame in container.decode(video=0):
        if 0 < frame_skip:
            frame_skip -= 1
            continue
        start_time = time.time()  
        image = cv2.cvtColor(np.array(
            frame.to_image()), cv2.COLOR_RGB2BGR)



        if start_detect:

            hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            red_mask = find_Mask(hsv_img)
            _, c_c, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            if(len(c_c) == 0):
                continue
            max_c = max(c_c, key = cv2.contourArea)
            rect = cv2.minAreaRect(max_c)
            r_w, r_h = rect[1]
            rec_x = rect[0][0]
            rec_y = rect[0][1]
            
            show_image = cv2.cvtColor(np.zeros(image.shape[:2], dtype=np.uint8), cv2.COLOR_GRAY2BGR)
            #cv2.drawContours(show_image, [max_c], -1, (0, 0, 255), -1)
            cv2.rectangle(show_image, (int(rec_x-0.5*r_w), int(rec_y-0.5*r_h)), (int(rec_x+0.5*r_w), int(rec_y+0.5*r_h)), (0,0,255), -1)
            print("minAreaRect x: ", rec_x)
            print("minAreaRect y: ", rec_y)
            cv2.circle(show_image, (int(rec_x), int(rec_y)), 10, (0,50,175), -1)
            cv2.putText(show_image, str((r_w * r_h) / (960*720.)), (10,40), 5, 2, (255,255,0))
            
            if start_detect:
                if r_w * r_h >= 960*720*0.35:
                    point_pub.publish(Float64MultiArray(data = [rec_x, rec_y, 1]))
                    start_detect = False
                else:
                    point_pub.publish(Float64MultiArray(data = [rec_x, rec_y, 0]))            
            
            out.write(np.concatenate((image, show_image), axis = 1))    

            cv2.imshow('result', np.concatenate((image, show_image), axis = 1))
            cv2.waitKey(1)
            if frame.time_base < 1.0/60:
                time_base = 1.0/60
            else:
                time_base = frame.time_base
            frame_skip = int((time.time() - start_time)/time_base)

        else:
        
            gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            result = detector.detect(gray_image)
            show_image = image.copy()
        
            if len(result) == 0:
                print("not found")
            else:
                center = result[0].center
                corner = result[0].corners
                tag_id = result[0].tag_id
                w = math.sqrt( (corner[0][0] - corner[1][0])**2 + (corner[0][1] - corner[1][1])**2 )
                h = math.sqrt( (corner[2][0] - corner[1][0])**2 + (corner[2][1] - corner[1][1])**2 )
                print(w, h, w*h)
                point_ap_pub.publish(Float64MultiArray(data = [center[0], center[1], w*h, tag_id ]))
                cv2.circle(show_image, (int(center[0]),int(center[1])) , 5, (0,0,255), -1)
                cv2.polylines(show_image,[np.int32(corner)], True, (0,255,0), 2, cv2.LINE_AA)
            
            out.write(show_image)    
            cv2.imshow('result', show_image)
            cv2.waitKey(1)
            if frame.time_base < 1.0/60:
                time_base = 1.0/60
            else:
                time_base = frame.time_base
            frame_skip = int((time.time() - start_time)/time_base)

if __name__ == '__main__':
    try:
        main()
    except BaseException:
        traceback.print_exc()
    finally:
        stream.close()
        cv2.destroyAllWindows()
