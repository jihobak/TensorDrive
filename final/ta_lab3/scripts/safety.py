#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Header
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from threading import Thread #imsosorry

import pdb
import numpy as np

"""
Author: Ariel Anders & Corey Walsh
This program implements a simple safety node based on laser
scan data.

# Single scan from a planar laser range-finder

Header header
# stamp: The acquisition time of the first ray in the scan.
# frame_id: The laser is assumed to spin around the positive Z axis
# (counterclockwise, if Z is up) with the zero angle forward along the x axis

float32 angle_min # start angle of the scan [rad]
float32 angle_max # end angle of the scan [rad]
float32 angle_increment # angular distance between measurements [rad]

float32 time_increment # time between measurements [seconds] - if your scanner
# is moving, this will be used in interpolating position of 3d points
float32 scan_time # time between scans [seconds]

float32 range_min # minimum range value [m]
float32 range_max # maximum range value [m]

float32[] ranges # range data [m] (Note: values < range_min or > range_max should be discarded)
float32[] intensities # intensity data [device-specific units]. If your
# device does not provide intensities, please leave the array empty.

"""
HISTORY_SIZE = 5
MIN_FRONT_DIST = 1.3 # meters
CHANGE_FRON_DIST = 3.3#1.5
FAN_ANGLE = 7.0 #15.0 # angle that is considered the front
N_BINS = 19

DEFAULT_SPEED = 359
BACKWARD_SPEED = 412

GEAR = 1

FRONT_STEER = 0.16
LEFT_STEER = 0.32#0.65
RIGHT_STEER = -0.01 #-0.41

KP = 1.4
KD = 0.4

class CircularArray(object):
    """docstring for CircularArray"""
    def __init__(self, size):
        self.arr = np.zeros(size)
        self.ind = 0
        self.num_els = 0

    def append(self, value):
        if self.num_els < self.arr.shape[0]:
            self.num_els += 1
        self.arr[self.ind] = value
        self.ind = (self.ind + 1) % self.arr.shape[0]

    def mean(self):
        if self.num_els == 0:
            return 0
        return np.mean(self.arr[:self.num_els])

    def median(self):
        return np.median(self.arr[:self.num_els])


class Safety():
    def __init__(self):
        self.received_data = None
        self.parsed_data = None

        self.angles = None
        self.bins = None
        self.averages = None

        self.sub = rospy.Subscriber("/scan", LaserScan, self.lidarCB, queue_size=5)
        self.pub = rospy.Publisher("drive_cmd",\
                AckermannDriveStamped, queue_size =5)
        
        self.status = True
        self.error_last = 0
        self.steer = 0.0
        
        self.right_is_long = False
        self.left_is_long = False
        
        self.steering_hist = CircularArray(HISTORY_SIZE)
        
        self.range_hist_front = CircularArray(HISTORY_SIZE+5)
        self.range_hist_left = CircularArray(HISTORY_SIZE+5)
        self.range_hist_right = CircularArray(HISTORY_SIZE+5)        
        
        self.thread = Thread(target=self.drive)
        self.thread.start()
        rospy.loginfo("safety node initialized")

    def drive(self):
        while not rospy.is_shutdown():
            if self.received_data is None or self.parsed_data is None:
                rospy.sleep(0.5)
                continue
            
            print "front", self.range_hist_front.mean()#, self.parsed_data["front"][:,0] 
            print "left", self.range_hist_left.mean()#, self.parsed_data["left"][:,0]
            print "right", self.range_hist_right.mean()#, self.parsed_data["right"][:,0]
            
            
            if (np.any(self.parsed_data['front'][:,0] < MIN_FRONT_DIST) or (not self.status)):
                #print "stop!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                rospy.loginfo("stoping!")
                # this is overkill to specify the message, but it doesn't hurt
                # to be overly explicit
                drive_msg_stamped = AckermannDriveStamped()
                drive_msg = AckermannDrive()
                
                self.steer = 0.0
                
                if(self.right_is_long):
                    self.steer = LEFT_STEER
                else:
                    self.steer = RIGHT_STEER
                
                drive_msg.steering_angle = self.steer
                drive_msg.acceleration = 0
                drive_msg.jerk = 0
                drive_msg.steering_angle_velocity = 0
                drive_msg_stamped.drive = drive_msg
                
                #if GEAR:
                #    print "GEAR>>>>>"
                #    for i in range(390, 500):
                #        print "BACK>>>>>>>>>"
                #        drive_msg.speed = 403
                #        self.pub.publish(drive_msg_stamped)
                
                GEAR = 0
                
                drive_msg.speed = BACKWARD_SPEED
                self.pub.publish(drive_msg_stamped)
                rospy.sleep(.4)
                
            
            elif np.any(self.parsed_data['front'][:,0] < CHANGE_FRON_DIST):
                rospy.loginfo("change!")
                GEAR = 1
                
                if np.sum(self.range_hist_left.mean() > self.range_hist_right.mean()):
                    self.right_is_long = False
                    self.left_is_long = True
                    #print "turn left"
                    #print np.sum(self.parsed_data['left'][:,0]), np.sum(self.parsed_data['right'][:,0])
                    #print self.parsed_data['right'][:,0]
                    drive_msg_stamped = AckermannDriveStamped()
                    drive_msg = AckermannDrive()
                    drive_msg.speed = DEFAULT_SPEED
                    
                    #PID CONTROL
                    ERROR_CURRENT = LEFT_STEER - self.steer # 0 - 260
                    E_DIFF = ERROR_CURRENT - self.error_last
                    self.error_last = ERROR_CURRENT
                    
                    result = KP*ERROR_CURRENT - KD*E_DIFF
                    self.steer = result
                    #print "steer : ", self.steer
                    #print "error_last : ", self.error_last
                    #print "error_current : ", ERROR_CURRENT
                    #print "e_diff : ", E_DIFF
                    #print "error_last : ", self.error_last
                    print "turn left PID result>>>> ", result
                    
                    #self.steering_hist.append(result)
                    self.steer = result
                    drive_msg.steering_angle = self.steer
                    
                    drive_msg.acceleration = 0
                    drive_msg.jerk = 0
                    drive_msg.steering_angle_velocity = 0
                    drive_msg_stamped.drive = drive_msg
                    self.pub.publish(drive_msg_stamped)
                else:
                    self.right_is_long = True
                    self.left_is_long = False
                    #print "turn right"
                    #print np.sum(self.parsed_data['left'][:,0]), np.sum(self.parsed_data['right'][:,0])
                    #print self.parsed_data['right'][:,0]
                    drive_msg_stamped = AckermannDriveStamped()
                    drive_msg = AckermannDrive()
                    drive_msg.speed = DEFAULT_SPEED
                    
                    #PID CONTROL
                    ERROR_CURRENT = RIGHT_STEER - self.steer
                    E_DIFF = ERROR_CURRENT - self.error_last
                    self.error_last = ERROR_CURRENT
                    
                    result = KP*ERROR_CURRENT - KD*E_DIFF
                    #print "steer : ", self.steer
                    #print "error_current : ", ERROR_CURRENT
                    #print "e_diff : ", E_DIFF
                    #print "error_last : ", self.error_last
                    print "turn right PID result>>>> ", result
                    
                    self.steer = result
                    drive_msg.steering_angle = self.steer
                    
                    drive_msg.acceleration = 0
                    drive_msg.jerk = 0
                    drive_msg.steering_angle_velocity = 0
                    drive_msg_stamped.drive = drive_msg
                    self.pub.publish(drive_msg_stamped)
                    
            else:
                
                #print "drive~!!>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
                self.right_is_long = False
                self.left_is_long = False
                GEAR = 1
                drive_msg_stamped = AckermannDriveStamped()
                drive_msg = AckermannDrive()
                drive_msg.speed = DEFAULT_SPEED
                #PID CONTROL
                ERROR_CURRENT = FRONT_STEER - self.steer
                E_DIFF = ERROR_CURRENT - self.error_last
                self.error_last = ERROR_CURRENT
                
                result = KP*ERROR_CURRENT - KD*E_DIFF
                #print "steer : ", self.steer
                #print "error_current : ", ERROR_CURRENT
                #print "e_diff : ", E_DIFF
                #print "error_last : ", self.error_last
                print "go straight PID result>>>> ", result
                
                self.steer = result
                drive_msg.steering_angle = self.steer
                
                drive_msg.acceleration = 0
                drive_msg.jerk = 0
                drive_msg.steering_angle_velocity = 0
                drive_msg_stamped.drive = drive_msg
                self.pub.publish(drive_msg_stamped)
                
            # don't spin too fast
            rospy.sleep(.1)

    def lidarCB(self, msg):
        # for performance, cache data that does not need to be recomputed on each iteration
        #print "angle_increment: {0}".format(msg.angle_increment) # 0.00233268318698
        #print "angle_min: {0}".format(msg.angle_min) # -0.767340004444 43
        #print "angle_max: {0}".format(msg.angle_max) # 0.797890424728  45
        #print "max len: {0}".format(len(msg.ranges))
        #print "range_max: {0}".format(msg.range_max)
        #print "range_min: {0}".format(msg.range_min)
        
        if not self.received_data:
            rospy.loginfo("success! first message received")
            # cache laser scanner angles
            self.angles = (np.arange(len(msg.ranges)) * msg.angle_increment) + msg.angle_min
            self.angles *= 180.0 / np.pi # convert to degrees
            
            # bins for chunking data
            self.bins = np.linspace(self.angles[0], self.angles[-1], num=N_BINS+1)
            
            # find center angle for each bin and set to second value of each row
            self.averages = np.zeros((N_BINS,3))
            self.averages[:,1] = (self.bins[:-1] + self.bins[1:]) / 2.0

            # bins for left, center, right data
            self.large_bins = np.array([-float('inf'), -35, -28, -21, -14, -1*FAN_ANGLE, FAN_ANGLE, 14, 21, 28, 35, float('inf')])
            #self.large_bins = np.array([-float('inf'), -1*FAN_ANGLE, FAN_ANGLE, float('inf')])
            
        values = np.array(msg.ranges)
        #print 
        #print ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>{}".format(np.sum(np.isinf(values))/float(values.size))
        if np.sum(np.isnan(values)+np.isinf(values))/float(values.size) > 0.55:
            print ">>>>>>>>>>>>> Somthing wrong <<<<<<<<<<"
            self.status = False
        
        else:
            self.status = True
        #print (np.sum(np.isnan(values)+np.isinf(values))),len(values)
        #print values
        # filter out range values that are outside the bounds of the laser scanner
        
        #values[np.isnan(values)] = 5#msg.range_max
        #print np.isnan(values)
        #print values
        ranges = values[(values >= msg.range_min) & (values <= msg.range_max)]
        angles = self.angles[(values > msg.range_min) & (values < msg.range_max)]
        #print "ranges", ">>>>>>>>>>>>>>>>>>>>"
        #print ranges
        #print "angles", "***********************"
        #print angles
        # Compute average range for each bin 
        #   - this is a linear time algorithm which scans through all scan ranges 
        #     assigning each range to the correct bin for the corresponding angle
        #     works because the angles array is sorted
        self.averages[:,0] = 0
        self.averages[:,2] = 0
        bin_num = 0
        #print angles.size
        for i in xrange(angles.size):
            if angles[i] > self.bins[bin_num+1]:
                bin_num += 1
            self.averages[bin_num,0] += ranges[i] # add range to bin
            self.averages[bin_num,2] += 1 # number of elements per bin

        # compute the average value for each bin
        # first remove bins with no elements to avoid divide by zero errors
        averages = self.averages[self.averages[:,2]!= 0]
        averages[:,0] = averages[:,0] / averages[:,2]
        
        # get left, center, right data
        digits = np.digitize(averages[:,1], self.large_bins)
        
        result = { 
                'right': averages [((digits==5)+(digits==4)+(digits==3)), :2], # +(digits==3) ((digits==5)+(digits==4)+(digits==3))
                'front': averages [((digits==6)+(digits==5)), :2], # [((digits==6)+(digits==5)), :2]
                'left': averages [((digits==7)+(digits==8)+(digits==9)), :2], #(digits==9)+  ((digits==7)+(digits==8)+(digits==9))
            }
        
        #print result
        #print "front", result["front"], np.sum(result['front'][:,0])
        #print "left", result["left"], np.sum(result['left'][:,0])
        #print "right", result["right"], np.sum(result['right'][:,0])
        
        self.range_hist_front.append(np.mean(result['front'][:,0]))
        self.range_hist_left.append(np.mean(result['left'][:,0]))
        self.range_hist_right.append(np.mean(result['right'][:,0]))
        
        self.received_data = True
        self.parsed_data = result

if __name__=="__main__":
    rospy.init_node("lidar_safety")
    Safety()
    rospy.spin()

