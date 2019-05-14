import rospy

class Operation:
    def __init__(self, opname):
        self.opname = opname
        self.opcode = rospy.get_param("~"+opname+"/opcode")
        self.args = rospy.get_param("~"+opname+"/args")

class Operations:
    def __init__(self):
        self.operations = rospy.get_param("~operations")
