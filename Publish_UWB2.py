import rospy
#from tork_rpc_util.msg import StringList
from std_msgs.msg import String
from custom_msgs.msg import Uwb, Linear_acceleration, Angular_acceleration, Euler_angles
import serial
import threading
from datetime import datetime
import time
import queue
import logging  # for logging all data in a file


# initialisation of the node and the publishers
rospy.init_node('UWB_publisher')
pub_imu_linear_acc = rospy.Publisher("/imu_uwb_linear_acc", Linear_acceleration, queue_size=10)
pub_imu_angular_acc = rospy.Publisher("/imu_uwb_angular_acc", Angular_acceleration, queue_size=10)
pub_imu_euler_angles = rospy.Publisher("/imu_uwb_euler_angles", Euler_angles, queue_size=10)
pub_distance = rospy.Publisher("/Distance", Uwb, queue_size=10)
#msg=String()
msg = Uwb()
msg1 = Linear_acceleration()
msg2 = Angular_acceleration()
msg3 = Angular_acceleration()

"""
def readSerialLines(serialPort, myQueue, evt, queueLogEvent=["+MPOS", "+DIST"]):
    serialPort.reset_input_buffer()

    while (True):
        if (serialPort.inWaiting() > 0):
            serialBytes = serialPort.readline()  ##check stoppgin criteria for readline (specific character ? Probably a blocking method check characters end.

            # to do replace internal timestamp with tag timestamp (probably more accurate if delay in transmission from the tag, but correspondance between local and tag tiemstamp has to be assessed
            # need to implement a fiunction localtimestampe_From_TagTimestamp

            tstp = datetime.now().timestamp()
            # timestamp(datetime.now()) # could be replaced by datetime.now().timestamp() ?

            ## interpret string, and queue only "interesting" data depending on "queueLogEvent" condition:
            strg = serialBytes.decode("Ascii")
            # log into file
            logging.info("timestamp: " + str(tstp) + ": " + strg)

            #for cdt in queueLogEvent:  ## one of					if cdt == "+MPOS":  # the obly implemented rigth now
                #if strg[
                   #0:5] == "+MPOS":  ## in that case, we interpret the string according to uwb device spec to queue an objet [timestamp, posX, posY]
                    #s = strg.split(",")
                    # for the moment string are logged
                    #position = str(tstp)+",+MPOS ,"+ s[1] +" ,"+ s[2]
                    #myQueue.put(position)
                    #msg.data = position
                    #pub_position.publish(msg)
                    #print('position : ', msg.data)
                    #print(type(msg.data))
          
            if strg[0:5] == "+DIST":  ## in that case, we interpret the string according to uwb device spec to queue an objet [timestamp, posX, posY]
                s = strg.split(",")
                # for the moment string are logged
                distance = str(tstp)+" ,+DIST ,"+s[1]+" ,"+s[2]
                myQueue.put(distance)
                #msg.anchorId = strg[14:22]
                msg.anchorId = s[1]
                msg.range = int(s[2])
                #print(strg[24:27])
                #print(strg[23:28])
                #print(strg[22:29])
                #print(strg)
                #msg.range = int(strg[23:26])
                #print (anchorId)
                #print(range)
                print(strg)
                pub_distance.publish(msg)

            if strg[0:5] == "+MACC":  ## in that case, we interpret the string according to uwb device spec to queue an objet [timestamp, posX, posY]
                s = strg.split(",")
                # for the moment string are logged
                acceleration = str(tstp) + " ,+MACC ," + s[1] + " ," + s[2] + " ," + s[3]
            myQueue.put(acceleration)
            # msg.anchorId = strg[14:22]
            msg.X_acceleration = s[1]
	    msg.Y_acceleration = s[2]
	    msg.Z_acceleration = s[3]
            msg.range = int(s[2])
            # print(strg[24:27])
            # print(strg[23:28])
            # print(strg[22:29])
            # print(strg)
            # msg.range = int(strg[23:26])
            # print (anchorId)
            # print(range)
            print(strg)
            pub_imu.publish(msg)
        if evt.is_set():
            print("stopping reading loop")
            return
"""

def readSerialLines2(serialPort, myQueue, evt, queueLogEvent=["+MPOS", "+DIST", "+MACC", "+MGYRO", "+MGVT", "+MQUAT", "+MHRP"]):
    if (serialPort.inWaiting() > 0):
        serialBytes = serialPort.readline()  ##check stoppgin criteria for readline (specific character ? Probably a blocking method check characters end.
        strg = serialBytes.decode("Ascii")
        #print(strg)
        if strg[0:5] == "+DIST":
            s = strg.split(",")
            tstp = datetime.now().timestamp()
            distance = str(tstp)+" ,+DIST ,"+s[1]+" ,"+s[2]
            msg.anchorId = s[1]
            msg.range = int(s[2])
            print(strg)
            pub_distance.publish(msg)

        if strg[0:5] == "+MACC":
            s = strg.split(",")
            tstp = datetime.now().timestamp()
            Linear_acceleration = str(tstp)+" ,+MACC ,"+s[1] + " ," +s[2]+ " ," +s[3][:-2]
            msg1.X_acceleration = s[1]
            msg1.Y_acceleration = s[2]
	        msg1.Z_acceleration = s[3]
            print(strg)
            pub_imu_linear_acc.publish(msg1)

        if strg[0:6] == "+MGYRO":
            s = strg.split(",")
            tstp = datetime.now().timestamp()
            angular_velocity = str(tstp) + " ,+MGYRO ," + s[1] + " ," + s[2] + " ," + s[3][:-2]
            print(strg)
	        msg2.X_acceleration = s[1]
	        msg2.Y_acceleration = s[2]
	        msg2.Z_acceleration = s[3]
            pub_imu_angular_acc.publish(msg2)

        if strg[0:6] == "+MGVT":
            s = strg.split(",")
            tstp = datetime.now().timestamp()
            tag_gravity = str(tstp) + " ,+MGYRO ," + s[1] + " ," + s[2] + " ," + s[3][:-2]
            print(strg)
            pub_imu.publish(tag_gravity)

        if strg[0:6] == "+MQUAT":
            s = strg.split(",")
            tstp = datetime.now().timestamp()
            tag_orientation = str(tstp) + " ,+MQUAT ," + s[1] + " ," + s[2] + " ," + s[3] + " ," + s[4][:-2]
            print(strg)
            pub_imu.publish(tag_orientation)

        if strg[0:5] == "+MHRP":
            s = strg.split(",")
            tstp = datetime.now().timestamp()
            tag_orientation = str(tstp) + " ,+MHRP ," + s[1] + " ," + s[2] + " ," + s[3][:-2]
            print(strg)
	        msg3.yaw = s[1]
	        msg3.pitch = s[2]
	        msg3.roll = s[3]
            pub_imu_euler_angles.publish(msg3)

            # to do replace internal timestamp with tag timestamp (probably more accurate if delay in transmission from the tag, but correspondance between local and tag tiemstamp has to be assessed
            # need to implement a fiunction localtimestampe_From_TagTimestamp

            #tstp = datetime.now().timestamp()
            # timestamp(datetime.now()) # could be replaced by datetime.now().timestamp() ?

            ## interpret string, and queue only "interesting" data depending on "queueLogEvent" condition
            # log into file
            #logging.info("timestamp: " + str(tstp) + ": " + strg)

            #for cdt in queueLogEvent:  ## one of                                       if cdt == "+MPOS":  # the obly implemented rigth now
                #if strg[
                   #0:5] == "+MPOS":  ## in that case, we interpret the string according to uwb device spec to queue an objet [timestamp, posX, posY]
                    #s = strg.split(",")
                    # for the moment string are logged
                    #position = str(tstp)+",+MPOS ,"+ s[1] +" ,"+ s[2]
                    #myQueue.put(position)
                    #msg.data = position
                    #pub_position.publish(msg)
                    #print('position : ', msg.data)
                    #print(type(msg.data))

             #if strg[0:5] == "+DIST":  ## in that case, we interpret the string according to uwb device spec to queue an objet [timestamp, posX, posY]
                #s = strg.split(",")
                # for the moment string are logged
                #distance = str(tstp)+" ,+DIST ,"+s[1]+" ,"+s[2]
                #myQueue.put(distance)
                #msg.anchorId = strg[14:22]
                #msg.anchorId = s[1]
                #msg.range = int(s[2])
                #print(strg[24:27])
                #print(strg[23:28])
                #print(strg[22:29])
                #print(strg)
                #msg.range = int(strg[23:26])
                #print (anchorId)
                #print(range)
                #print(strg)
                #pub_distance.publish(msg)
        if evt.is_set():
            print("stopping reading loop")
            return




def readSerialLinesCommand(serialPort, myQueue, evt):
    while (True):
        if (serialPort.inWaiting() > 0):
            serialBytes = serialPort.readline()  ##check stoppgin criteria for readline (specific character ? Probably a blocking method check characters end.
            ## interpret string, and queue only "interesting" data depending on "queueLogEvent" condition:
            strg = serialBytes.decode("Ascii")
            myQueue.put(strg)
            logging.info(strg)
            print(strg)

        if evt.is_set():
            print("stopping reading loop")
            return


class UWBconnect:
    def __init__(self,
                 port):  # , baudrate=9600, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE, serialParity = serial.PARITY_NONE):
        self.serialPort = serial.Serial(port)  # , baudrate, bytesize, timeout, stopbits, serialParity)
        self.serialPort.reset_input_buffer()
        self.serialPort.reset_output_buffer()
        self.queue = queue.Queue()
        self.stpReading = threading.Event()

    def setReadingThread(self):
        self.stpReading.clear()
        self.inputThread = threading.Thread(target=readSerialLines, args=(self.serialPort, self.queue, self.stpReading),
                                            daemon=True)
        self.inputThread.start()

    # stop reading thread, probably required if some commands have to be send to the tag
    def stopReadingThread(self):
        self.stpReading.set()

    # Don't rely on putty anymore, but establish interactive communication with the device
    def WriteCommand(self, strCommand):
        self.stopReadingThread()
        self.serialPort.reset_input_buffer()
        self.serialPort.reset_output_buffer()
        if strCommand[-1] != '\r\n':
            strCommand += '\r\n'
            print(strCommand)
        bstr = strCommand.encode('ascii')

        localQueue = queue.Queue()
        localstp = threading.Event()
        th = threading.Thread(target=readSerialLinesCommand, args=(self.serialPort, localQueue, localstp), daemon=True)
        th.start()
        self.serialPort.write(bstr)

        time.sleep(1)  # decent time for response ?

        localstp.set()

        while not localQueue.empty():
            st = localQueue.get()

            ## to be implemented like a destructor ?

    def closeSerial(self):
        pass


# Connexion to the UWB Tag
print("test")
UWB = UWBconnect('/dev/ttyACM0')
rate = rospy.Rate(20)
while not rospy.is_shutdown():
    readSerialLines2(UWB.serialPort, UWB.queue, UWB.stpReading)
    rate.sleep()
rospy.loginfo("Node was stopped")

#UWB.setReadingThread()
    # read queue event and run a Kalman filter
	# def locateRobot:
    # pass







