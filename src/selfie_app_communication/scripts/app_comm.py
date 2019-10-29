#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32, Empty
from std_srvs.srv import Empty
import dynamic_reconfigure.client
import bluetooth as bt
import select
import threading
from binascii import hexlify, unhexlify
from struct import pack, unpack
from lxml import etree
class AppComm:
    def __init__(self):

        rospy.init_node('app_comm')

        self.activeConnections = []
        self.transmitQueues = {}
        self.port = 1
        self.serverSocket = bt.BluetoothSocket(bt.RFCOMM)
        self.serverSocket.bind(('',self.port))
        self.serverSocket.listen(7)
        self.serverSocket.setblocking(False)
        self.btThread = threading.Thread(target=self.btFun,name='btThread')

        self.msgTypes = {}
        self.typeLengths = self.createTypeLengths()

        self.subs = []
        self.dyns = {}
        self.srvs = {}
        self.targets = {}
        self.varNames = {}
        self.root = self.parse()
        self.create()

        self.btThread.start()

    def subCallbackFloat32(self,msg,code):
        for key in self.transmitQueues.keys():
            self.transmitQueues[key] += (str(code) + str(pack(self.msgTypes[code],msg.data)))

    def parse(self):
        parser = etree.XMLParser(remove_blank_text=True)
        settings = rospy.get_param("app_settings")
        root = etree.XML(settings,parser)
        return root

    def create(self):

        for tab in self.root:
            for row in tab:
                for element in row:
                    tag = element.tag
                    if tag == 'dyn':
                        try:
                            dyn = dynamic_reconfigure.client.Client(element.get('dynname'),timeout = 0.1)
                            for var in element:
                                code = unhexlify(var.get('code'))
                                self.dyns[code] = dyn
                                self.varNames[code] = var.get('varname')
                                self.msgTypes[code] = var.get('type')
                        except:
                            print 'cant open client'
                    elif tag == 'sens':
                        if element.get('type') == 'f':
                            code = unhexlify(element.get('code'))
                            sub = rospy.Subscriber(element.get('topic'),Float32,lambda value,cd=code: self.subCallbackFloat32(value,cd))

                            self.subs.append(sub)
                    elif tag == 'srv':
                        if element.get('type') == 'e':
                            srv = rospy.ServiceProxy(element.get('srvname'),Empty)
                            code = unhexlify(element.get('code'))
                            self.srvs[code] = srv
                            self.msgTypes[code] = element.get('type')

    def btFun(self):
        while not rospy.is_shutdown():
            readable, writable, xd = select.select(self.activeConnections + [self.serverSocket], self.activeConnections, [], 0.01)
            if self.serverSocket in readable:
                try:
                    remote_socket, (address,_) = self.serverSocket.accept()
                except bt.BluetoothError:
                    print "couldnt accept the connection"
                else:
                    print "connected to " + bt.lookup_name(address)
                    self.activeConnections.append(remote_socket)
                    self.transmitQueues[remote_socket] = ''
                readable.remove(self.serverSocket)
            for read in readable:
                msg = ''
                try:
                    msg = read.recv(5000)
                except bt.BluetoothError:
                    self.activeConnections.remove(read)
                    print 'disconnection'
                msgs = self.seperate_msgs(msg)
                self.handleMsgs(msgs)



            for written in writable:
                if self.transmitQueues[written]:
                    written.send(self.transmitQueues[written])
                    self.transmitQueues[written] = ''

    def seperate_msgs(self,msg):
        i = 0
        sepMsgs = []
        while i < len(msg):
            code = msg[i:i+3]
            i+=3
            value = None
            type = self.msgTypes[code]
            length = self.typeLengths[type]
            if length:
                value = unpack(self.msgTypes[code],msg[i:i+length])[0]
                i+=length
            sepMsgs.append((code,value))
        return sepMsgs

    def createTypeLengths(self):
        typeLengths = {}
        typeLengths['e'] = 0
        typeLengths['?'] = 1   #bool
        typeLengths['f'] = 4
        return typeLengths

    def handleMsgs(self,msgs):
        for msg in msgs:
            code = msg[0]
            val = msg[1]
            print code, val
            if code in self.dyns.keys():
                try:
                    self.dyns[code].update_configuration({self.varNames[code]: val})
                except:
                    print 'server unavailable'
                else: print 'calling dyn'
            elif code in self.srvs.keys():
                print val
                try:
                    self.srvs[code]()
                except:
                    print 'server unavailable'
                else: print 'calling srv'


if __name__ == '__main__':
    appComm = AppComm()
    rospy.spin()
