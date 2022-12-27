#!/usr/bin/env python
import rospy
import simple_tello

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from behave import *
from time import sleep

AP_AREA_MAX = 100
AP_AREA_MIN = 75

AP_CENTER = 480


t1 = simple_tello.Tello_drone()

class bt_mission:

    isContinue = True
    center = (480, 200)

    passed1 = False
    passed2 = False

    dx = -1
    dy = -1

    def __init__(self):
        self.tree = (

            (self.hasPassed1 >> ((self.isNotApReceived >> self.doHover) | ((self.xNotOk >> self.doLeftOrRight) | (self.yNotOk >> self.doFowardOrBack) | self.down)))
            | (self.isNotDataReceived >> self.doHover)
            | (self.isReceivePass >> self.doAddSp >> self.pass1Complete)
            | (self.doComputeData >> (self.isForward >> self.doForward) | (self.doCorrection)) 
        )

    @condition
    def hasPassed1(self):
        print("condition: has Passed1")
        return bt_mission.passed1 == True 
    
    @condition
    def isNotApReceived(self):
        print("condition: isNotApReceived")
        return t1.state.ap_center_x == -1 and t1.state.ap_center_y == -1

    @condition
    def xNotOk(self):
        print("condition: xNotOk")
        return not (abs(t1.state.ap_center_x - AP_CENTER) <= 5)
    
    @action
    def doLeftOrRight(self):
        print("action: doLeftOrRight")
        msg = Twist()

        correct = t1.state.ap_center_x - AP_CENTER
        msg.linear.x = correct / abs(correct) * 0.1
        t1.controler.move(msg, 0.5)


    @condition
    def yNotOk(self):
        print("condition: yNotOk")
        return not (t1.state.ap_area >= AP_AREA_MIN and t1.state.ap_area <= AP_AREA_MAX)

    @action    
    def doFowardOrBack(self):

        print("action: doForwardOrBack")
        msg = Twist()
       
        if t1.state.ap_area < AP_AREA_MIN:      
            msg.linear.y = 0.1
        elif t1.state.ap_area > AP_AREA_MAX: 
            msg.linear.y = - 0.1

        t1.controler.move(msg, 0.5)

    
    @action
    def down(self):
        print("action: down")
        msg = Twist()
        msg.linear.z = 1
        t1.controler.move(msg, 1.5)

        msg2 = Twist()
        msg2.linear.y = -1
        t1.controler.move(msg2, 1)
        
        t1.controler.land()
        bt_mission.isContinue = False
        


    @condition
    def isNotDataReceived(self):
        print("condition: isNotDataReceived")
        return t1.state.target_x == -1 and t1.state.target_y == -1

    @action
    def doHover(self):
        print("action: doHover")
        msg = Twist()
        t1.controler.move(msg, 0.5)

    @condition
    def isReceivePass(self):
        print("condition: isReceivePass")
        return t1.state.canPass == 1

    @action
    def doAddSp(self):
        print("action: doAddSp")
        msg = Twist()
        msg.linear.y = 0.4
        #msg.linear.z = 0.1
        t1.controler.move(msg, 3)
      
        msg = Twist()
        msg.linear.y = 0.5
        t1.controler.move(msg, 3)

    @action
    def pass1Complete(self):
        print("action: pass1Complete")
        bt_mission.passed1 = True

    @action
    def doBtStop(self):
        print("action: doBtStop")
        bt_mission.isContinue = False

    @action
    def doComputeData(self):
        print("action: doComputeData")
        bt_mission.dx = t1.state.target_x - bt_mission.center[0]
        bt_mission.dy = t1.state.target_y - bt_mission.center[1]
        print("doComputeData: ", bt_mission.dx, bt_mission.dy)
        
    @condition
    def isForward(self):
        print("condition: isForward")
        return abs(bt_mission.dx) < 30 and abs(bt_mission.dy) < 30

    @action
    def doForward(self):
        print("action: doForward")
        msg = Twist()
        msg.linear.y = 0.2
        t1.controler.move(msg, 0.5)
    
    @action
    def doCorrection(self):
        print("action: doCorrection")
        msg = Twist()
       
        if bt_mission.dx != 0:
            msg.linear.x = bt_mission.dx / abs(bt_mission.dx) * 0.1
        if bt_mission.dy != 0:
            msg.linear.z = -bt_mission.dy / abs(bt_mission.dy) * 0.2
        t1.controler.move(msg, 0.5)

    def run(self):
        while True:
            if bt_mission.isContinue == False:
                break
            bb = self.tree.blackboard(1)
            state = bb.tick()
            print("state = %s\n" % state)
            #if bt_mission.drone.isStop == True:
            #  exec("f = open(\"123.txt\",\'rb\')")           
            while state == RUNNING:
                state = bb.tick()
                print("state = %s\n" % state)
                #if bt_mission.drone.isStop == True:
                #  exec("f = open(\"123.txt\",\'rb\')")
            assert state == SUCCESS or state == FAILURE

def main():
    
    while t1.state.is_flying == False: 
        t1.controler.takeoff()
  
    while t1.state.fly_mode != 6:
        print("wait...")
 
    btCm_n = bt_mission()
    sleep(2)
    print("bt start...")
    
    btCm_n.run()
    
    while t1.state.fly_mode != 6:
        print("wait...") 
    
    while t1.state.is_flying == True:
        t1.controler.land()        

if __name__ == "__main__":
    rospy.init_node('h264_pub', anonymous=True)
    main()
