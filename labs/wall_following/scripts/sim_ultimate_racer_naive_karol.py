#!/usr/bin/env python

import numpy as np
import time
import math
import rospy
import sys
from std_msgs.msg import UInt16
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from race.msg import drive_param

class move_auto:
    def __init__(self,slow_esc,medium_esc,fast_esc,drive_medium_threshold,drive_fast_threshold):

        self.lasteStopMsgTs=time.time()

        self.speed_record=0
        self.eStop=False
        self.eStart=False
        self.esc_brake=1000
        self.slow_esc=int(slow_esc)
        self.medium_esc=int(medium_esc)
        self.fast_esc=int(fast_esc)
        self.drive_medium_threshold=float(drive_medium_threshold)
        self.drive_fast_threshold=float(drive_medium_threshold)
        self.esc_min = 1550
        self.throttle = 1500


        self.neutral_yaw=1585
        self.yaw = self.neutral_yaw
        self.yaw_range=1200
        self.max_yaw=self.neutral_yaw+self.yaw_range/2
        self.min_yaw=self.neutral_yaw-self.yaw_range/2

        self.safe_obstacle_distance1=0.6
        self.safe_obstacle_distance2=0.05
        self.safe_obstacle_distance3=0.5
        self.safe_obstacle_distance4=2.5
        self.non_cont_dist=0.2

        self.fast_drive_distance=7.0
        self.margin_drive_fast=0.5
        self.margin_drive_slow=0.3

        self.slow_speed_chk_points=100

        self.curr_speed=0

        self.angle_increment=math.radians(0.25) #0.004363323096185923

        self.sin_alfa=np.zeros(360*4)

        for i in range(360*4):
            self.sin_alfa[i]=math.sin(math.radians((i+1)*0.25))


        self.pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)
        self.pub_esc = rospy.Publisher('/esc', UInt16,
                                   queue_size=1)

        self.pub_servo = rospy.Publisher('/servo', UInt16,
                                   queue_size=1)



        self.sub_spd = rospy.Subscriber("/spd", Float32, self.spdCallback,
                                    queue_size=1)

        self.sub = rospy.Subscriber("/scan", LaserScan, self.laserCallback,
                                    queue_size=1)

        self.sub_eStop = rospy.Subscriber("/eStop", UInt16, self.eStopCallback,
                                    queue_size=1)

    def spdCallback(self, data):
        self.curr_speed=data.data


    def checkIfReachable(self,r1,r2,alfa,margin):
        if (r1-self.safe_obstacle_distance1<r2 and r1>self.safe_obstacle_distance1):

            return True
        else:

            return (r2*self.sin_alfa[alfa]>margin)


    def steerMAX(self,scan,width_margin):
        segs=[]
        segs.append(180)
        segs.append(len(scan)-180)


        for i in range(1,len(scan)):
            if (abs(scan[i]-scan[i-1])> self.non_cont_dist):
                segs.append(i)
                segs.append(i-1)


        minRange=min(scan)

        scan2=np.copy(scan)

        idx=0

        isReachable = False

        scan2[:100]=-1
        scan2[-100:]=-1

        while (not isReachable and len(scan2[scan2>0])>0 and minRange>self.safe_obstacle_distance2):
            idx=np.argmax(scan2)

            for s in segs:
                if (s != idx):

                    if not (self.checkIfReachable(scan[idx],scan[s],abs(s-idx),width_margin)):
                        #if (idx==780):
                        #    print(idx,s)
                        #scan2[idx]=-1
                        scan2[max(0,idx-5):min(idx+5,1080)]=-1
                        break
            if (scan2[idx] != -1):
                isReachable=True

        if (isReachable==False):
            yaw=-1
        else:
            yaw=idx

        return yaw


    def eStopCallback(self, data):
        self.lasteStopMsgTs=time.time()
        if (data.data==0):
            self.exec_eStop()
            print("Emergency Stop!")
        elif (data.data==2309):
            self.eStart=True
            #self.throttle=self.starting_esc
            print("GO!")

    def exec_eStop(self):

        self.eStop=True
        self.yaw=self.neutral_yaw
        self.throttle=self.esc_brake
        self.pub_esc.publish(self.esc_brake)
        self.pub_servo.publish(self.yaw)




    def laserCallback(self, data):
        ts=time.time()

        debug_str = ''

        data.ranges = [30 if np.isinf(x) else x for x in data.ranges]
        data.ranges = [30 if np.isnan(x) else x for x in data.ranges]

        if(ts-self.lasteStopMsgTs>0.5 and self.eStart):
            debug_str += "No heartbeat in 0.5s; "
            # print("Haven't received hearbeat in 0.5s ==> ESTOP")
            self.exec_eStop()

        mid_point=int(len(data.ranges)/2)

        # Check obstacle in the front +/-20 scans
        if (min(data.ranges[mid_point-20:mid_point+20]) < self.safe_obstacle_distance3):
            debug_str += "Front dist3 %f < %f; "%(min(data.ranges[mid_point-20:mid_point+20]), self.safe_obstacle_distance3)
            # print("Front collision! ==> BRAKE")
            self.throttle=self.esc_brake
            self.pub_esc.publish(self.throttle)
            return

        scan = np.array(data.ranges)
        scan[scan>10]=10
        scan[scan<0.06]=0.06
        scan[:90]=10.0
        scan[-90:]=10.0

        idx=self.steerMAX(scan,self.margin_drive_fast)
        debug_str += "idx_fast=%d; "%idx

        if (idx==-1 or scan[idx]<self.drive_fast_threshold or idx > 600 or idx < 480):
            # print("steerMAX -> Can't go fast")
            idx=self.steerMAX(scan,self.margin_drive_slow)
            debug_str += "idx_slow=%d; "%idx

            if (idx>=0 and scan[idx] < self.safe_obstacle_distance4):
                debug_str += "dist4 %f < %f; "%(scan[idx],
                      self.safe_obstacle_distance4)
                # print("safe_obstacle_distance4: %f < %f"%(scan[idx],
                      # self.safe_obstacle_distance4))
                idx=-1
            if (idx==-1):
                debug_str += "idx=-1 => BRAKE; "
                self.throttle=self.esc_brake
            else:
                if (scan[idx]>self.drive_medium_threshold):
                    debug_str += "med_th %f > %f MEDIUM; "%(
                          scan[idx], self.drive_medium_threshold)
                    # print("drive_medium_threshold: %f > %f ==> medium_esc"%(
                          # scan[idx], self.drive_medium_threshold))
                    self.throttle=self.medium_esc
                else:
                    debug_str += "med_th %f < %f SLOW; "%(
                          scan[idx], self.drive_medium_threshold)
                    # print("drive_medium_threshold: %f < %f ==> slow_esc"%(
                          # scan[idx], self.drive_medium_threshold))
                    self.throttle=self.slow_esc
        else:
            debug_str += "Go FAST; "
            # print("steerMAX -> Go FAST ==> fast_esc")
            self.throttle=self.fast_esc



        L =len(scan)

        if (idx>=0):
            debug_str += "idx=%d; "%idx
            idx2yaw=idx/float(L)-0.5
            idx2yaw*=1.4
            if (idx2yaw>0.5):
                idx2yaw=0.5
            elif (idx2yaw<-0.5):
                idx2yaw=-0.5
            self.yaw=int(idx2yaw*self.yaw_range+self.neutral_yaw)

        if (not self.eStop and self.eStart and idx>=0):
            # alpha = 0.8
            # beta = -3.0
            # gamma = -2.0
            # idx2yaw=idx/L-0.5
            # th = alpha * scan[idx]
            # th += max(th, 2) * beta * abs(idx2yaw)
            # th += th * gamma / float(scan[idx])
            #
            # debug_str = "%6.2f %6.2f %6.2f  "%(alpha * scan[idx], beta * abs(idx2yaw), gamma / float(scan[idx])) + debug_str
            #
            # # th = alpha * scan[idx] * scan[idx] + beta * abs(idx2yaw)
            # self.throttle = int(1570 + th * 50)
            self.pub_esc.publish(self.throttle)
            self.pub_servo.publish(self.yaw)
        else:
            # print("ESTOP or !start or idx<0 ==> BRAKE")
            self.throttle=self.esc_brake
            self.pub_esc.publish(self.throttle)

        if (self.curr_speed>self.speed_record):
            self.speed_record=self.curr_speed

        min_r = min(scan[:len(scan)/2])
        min_l = min(scan[len(scan)/2:])

        if min(min_l,min_r) < 1.0:
            _t = int(100.0/float(min(min_l,min_r)))
            if min_l > min_r:
                self.yaw += _t
            else:
                self.yaw -= _t


        debug_str += "yaw=%d; "%self.yaw
        debug_str += "throttle=%d; "%self.throttle
        debug_str += "eStop=%d; "%self.eStop
        debug_str += "eStart=%d; "%self.eStart
        tf=time.time()

        print(debug_str)

        msg = drive_param()
        msg.velocity = (self.throttle - 1000.0)*4.0/1680.0
        msg.angle = (self.yaw - self.neutral_yaw)/800.0
        self.pub.publish(msg)




        # print('eStp:',self.eStop,'eStart', self.eStart,'yaw:',self.yaw,'thr:',self.throttle,'spd:',str(self.curr_speed)[0:4],'rec:', str(self.speed_record)[0:3] ,'tdiff:',str(tf-ts)[0:5])



def main(args):
    rospy.init_node('tryrover_node', anonymous=True)

    if (len(args)==6):
        slow_esc=args[1]
        medium_esc=args[2]
        fast_esc=args[3]
        drive_medium_threshold=args[4]
        drive_fast_threshold=args[5]

    else:
        slow_esc=1200
        medium_esc=2000
        fast_esc=2600
        drive_medium_threshold=5.0
        drive_fast_threshold=10.0

    print("slow_esc:",slow_esc)
    print("medium_esc:",medium_esc)
    print("fast_esc:",fast_esc)
    print("drive_medium_threshold:",drive_medium_threshold)
    print("drive_fast_threshold:",drive_fast_threshold)
    ma = move_auto(slow_esc,medium_esc,fast_esc,drive_medium_threshold,drive_fast_threshold)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
