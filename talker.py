#!/usr/bin/env python

import rospy
import serial
import re
import time
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped

setAnchorsManual = False

#configure serial port
ser = serial.Serial(
    port='/dev/ttyACM0',
    baudrate=115200,
    parity=serial.PARITY_ODD,
    stopbits=serial.STOPBITS_TWO,
    bytesize=serial.SEVENBITS
)

def errorOutput(errorStr):
    print errorStr
    rospy.logwarn(errorStr)
    return

def talker():
    #pubStr = rospy.Publisher('chatter', String, queue_size=10)
    pose = rospy.Publisher('PozyxPose', PoseWithCovarianceStamped, queue_size=10)

    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    #serial port is already open by ser setup
    '''try:
        ser.open()
    except Exception, e:
        errorOutput("unable to open serial port: " + str(e))
        exit()'''
    baseCoord = [[1,2,3],[4,5,6],[7,8,9],[10,11,12]]

    microStartup = True;
    print "Starting"
    while microStartup:
        incomingStr = ser.readline()  
        print incomingStr
        if("ERROR: Unable to connect to POZYX shield" in incomingStr):
            errorOutput("arduino unable to connect to sheild")
            exit()
        if("ERROR: calibration" in incomingStr):
            errorOutput("unable to calibrate Pozyx")
            exit()
        if("Waiting for manual anchor positions" in incomingStr ):
            print "Setting anchor posistiosns"
            for i in range(0,(2 if setAnchorsManual else 0)):
                for e in range(0,3):
                    ser.write(bytes(bin(baseCoord[i][e])[2:].zfill(32)))
            ser.write('\n')
        if("Starting positioning:" in incomingStr or incomingStr[0] == '$'):
            microStartup = False
    print 'Done'

    coord = [0,0,0]
    covariance = [[0,0,0,0,0,0], #[[0]*6 for i in range(6)]
                  [0,0,0,0,0,0],
                  [0,0,0,0,0,0],
                  [0,0,0,0,0,0],
                  [0,0,0,0,0,0],
                  [0,0,0,0,0,0]]
    time.sleep(5)
    while not rospy.is_shutdown():
        #locationStr = ser.readline()
        locationStr = "$X:100\tY:200\tZ:300\tErrors\t10\t20\t30\t40\t50\t60" #dummy value
        if(locationStr[0] == '$'):
            coordStr = re.split(r'\t+', locationStr)
            #convert string of mm to float of m
            coord[0] = float(coordStr[0][3:])/1000 #X
            coord[1] = float(coordStr[1][2:])/1000 #Y
            coord[2] = float(coordStr[2][2:])/1000 #Z
            #obtain errors 
            covariance[0][0] = float(coordStr[4])/1000 #X
            covariance[0][1] = float(coordStr[7])/1000 #XY
            covariance[0][2] = float(coordStr[8])/1000 #XZ
            covariance[1][0] = covariance[0][1]      #YX
            covariance[1][1] = float(coordStr[5])/1000 #Y
            covariance[1][2] = float(coordStr[9])/1000 #YZ
            covariance[2][0] = covariance[0][2]      #ZX
            covariance[2][1] = covariance[1][2]      #ZY
            covariance[2][2] = float(coordStr[6])/1000 #Z

            newPose = PoseWithCovarianceStamped()
            newPose.header.stamp=rospy.Time.now()
            newPose.header.frame_id="map"
            newPose.pose.pose.position.x = coord[0]
            newPose.pose.pose.position.y = coord[1]
            newPose.pose.pose.position.z = coord[2]
            newPose.pose.covariance = covariance
            rospy.loginfo(newPose)
            pose.publish(newPose)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass