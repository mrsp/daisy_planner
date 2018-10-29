#!/usr/bin/env python
import rospy
import socket,os

class TelephoneDataSoc(object):
    def __init__(self,soc_name):
        self.soc_name = soc_name
        self.buff_size = 127
	self.connected = False
        self.MSG_INTRO = "telephone_data"

    def close(self):
        self.soc.close()
        self.connected = False
        
    def connect(self):
        try:
            self.soc = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            self.soc.connect(self.soc_name)
            self.connected = True
        except Exception as e:
            rospy.logerr("Error connecting to socket: %s" % (str(e) ))
            self.connected = False
            return False
        return True

    def isConnected(self):
        return self.connected

    def readData(self):
        data = []
        bytes_recd = 0
        while bytes_recd < self.buff_size:
            try:
                chunk = self.soc.recv(self.buff_size - bytes_recd)
            except socket.error as e:
                self.connected = False
                rospy.logerr('Connection closed')
                return None
            if chunk == '':
                self.connected = False
                return None
            data.append(chunk)
            bytes_recd = bytes_recd + len(chunk)
            #print(chunk)
        #print('red')
        line = ''.join(data)        
        return line.rstrip()

    def readEvent(self):
        intro = self.readData()
        if intro is None:
            rospy.logerr("Couldn't read from socket.")
            return None
        if intro != self.MSG_INTRO:
            rospy.logerr("Wrong input.")
            return None
        #event=[event_type, phoneNumber, contactName] 
        event=[] 
        for i in range(4):
            s = self.readData()
            if s is None:
                rospy.logerr("Couldn't read from socket.")
                return None
            event.append(s)
        return event