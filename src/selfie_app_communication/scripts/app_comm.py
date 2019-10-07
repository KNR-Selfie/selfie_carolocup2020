#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32, Empty
from std_srvs.srv import Empty
import dynamic_reconfigure.client
import bluetooth as bt
import select
import threading
from binascii import hexlify, unhexlify
import struct #######
from struct import pack, unpack
from lxml import etree
class AppComm:
    def __init__(self):

        rospy.init_node('app_comm')

        self.active_connections = []
        self.transmit_queues = {}
        self.port = 1
        self.server_socket = bt.BluetoothSocket(bt.RFCOMM)
        self.server_socket.bind(('',self.port))
        self.server_socket.listen(7)
        self.server_socket.setblocking(False)
        self.btThread = threading.Thread(target=self.btFun,name='btThread')

        self.msgTypes = {}
        self.typeLengths = self.createTypeLengths()

        self.subs=[]
        self.dyns = {}
        self.srvs = {}
        self.targets = {}
        self.varNames = {}
        self.root = self.parse()
        self.create()

        self.btThread.start()

    def subCallbackFloat32(self,msg,code):
        print hexlify(code)
        for key in self.transmit_queues.keys():
            self.transmit_queues[key] += (str(code) + str(pack('f',msg.data)))
        #print self.transmit_queues.values()

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
                                print 'var'
                        except:
                            print 'cant open client'
                    elif tag == 'sens':
                        if element.get('type') == 'f':
                            print element.get('code')
                            code = unhexlify(element.get('code'))
                            print hexlify(code) + 'xd'
                            sub = rospy.Subscriber(element.get('topic'),Float32,lambda value,cd=code: self.subCallbackFloat32(value,cd))

                            self.subs.append(sub)
                    elif tag == 'srv':
                        if element.get('type') == 'e':
                            srv = rospy.ServiceProxy(element.get('srvname'),Empty)
                            code = unhexlify(element.get('code'))
                            self.srvs[code] = srv
                            self.msgTypes[code] = element.get('type')
                            print 'srv'

    def btFun(self):
        while not rospy.is_shutdown():
            readable, writable, xd = select.select(self.active_connections + [self.server_socket], self.active_connections, [], 0.01)
            if self.server_socket in readable:
                try:
                    remote_socket, (address,_) = self.server_socket.accept()
                except bt.BluetoothError:
                    print "couldnt accept the connection"
                except:
                    print "xd"
                else:
                    print "connected to " + bt.lookup_name(address)
                    self.active_connections.append(remote_socket)
                    self.transmit_queues[remote_socket] = ''
                readable.remove(self.server_socket)
            for read in readable:
                msg = ''
                try:
                    msg = read.recv(5000)
                except bt.BluetoothError:
                    self.active_connections.remove(read)
                    print 'disconnection'
                msgs = self.seperate_msgs(msg)
                self.handleMsgs(msgs)



            for written in writable:
                if self.transmit_queues[written]:
                    print 'written'
                    #print hexlify(self.transmit_queues[written])
                    written.send(self.transmit_queues[written])
                    self.transmit_queues[written] = ''

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