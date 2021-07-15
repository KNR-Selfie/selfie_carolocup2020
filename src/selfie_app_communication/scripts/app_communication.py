#!/usr/bin/env python3
import rospy
import bluetooth as bt
import select
import threading
from std_srvs.srv import Empty
import dynamic_reconfigure.client
from struct import pack, unpack
class Communicator:
    def __init__(self):
        self.connections = []
        self.port = 1
        self.server_socket = bt.BluetoothSocket(bt.RFCOMM)
        self.server_socket.bind(("", self.port))
        self.server_socket.listen(7)
        self.pidClient = dynamic_reconfigure.client.Client('controller')
        self.server_socket.setblocking(False)
        BTThread = threading.Thread(name="BTthread", target=self.BTfun)
        self.resetVisionService = rospy.ServiceProxy('resetVision', Empty)
        BTThread.start()
        #a = rospy.get_param("~operations")
        #print a
    def BTfun(self):
        while not rospy.is_shutdown():
            readable, writable, xd = select.select(self.connections + [self.server_socket], self.connections, [], 0.01)
            if self.server_socket in readable:
                try:
                    remote_socket, (address,_) = self.server_socket.accept()
                except bt.BluetoothError:
                    print "couldnt accept the connection"
                except:
                    print "xd"
                else:
                    print "connected to " + bt.lookup_name(address)
                    self.connections.append(remote_socket)
                readable.remove(self.server_socket)
            #endif
            for read in readable:
                try:
                    data = read.recv(5000)
                    print data[:3] == '\x01\x01\x01'
                    if data == "resetVision":
                        print data
                        self.resetVisionService()
                    elif data[:3]=='\x01\x01\x01':
                        self.pidClient.update_configuration({'Kp':float(unpack('f',data[3:])[0])})
                    elif data[:3] == '\x01\x01\x02':
                        self.pidClient.update_configuration({'Ki':float(unpack('f',data[3:])[0])})
                    elif data[:3] == '\x01\x01\x03':
                        self.pidClient.update_configuration({'Kd':float(unpack('f',data[3:])[0])})

                except bt.BluetoothError:
                    print "disconnected"
                    self.connections.remove(read)
                except:
                    print "xd"
                else:
                    print "received " + data






rospy.init_node("omega")
communicator = Communicator()

rospy.spin()


