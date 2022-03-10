#!/usr/bin/env python

import rospy
import snap7
import time
from geometry_msgs.msg import Twist


# Class to communicate with plc
class Plc(object):
    def __init__(self, ip, rack, slot, db_nr, db_size, sub_topic, name):
        self.name = name
        self.sub_topic = sub_topic
        self.cl = snap7.client.Client()
        self.ip = ip
        self.rack = rack
        self.slot = slot
        self.db_nr = db_nr
        self.db_size = db_size
		
        if name == "bw":
	    print("calling bw from main")
            self.sub = rospy.Subscriber(self.sub_topic, Twist, self.get_vel_bw)
            self.data_bw = {'Operating_Input_X': 0, 'Operating_Input_Y': 0,'Operating_Input_OMG': 0}
        elif name == "mc":
	    print("calling mc from main")
	    self.sub = rospy.Subscriber(self.sub_topic, Twist, self.get_vel_mc)
	    self.data_mc = {'n_1A_soll': 0.0, 'n_1B_soll': 0.0,
	                    'n_2A_soll': 0.0, 'n_2B_soll': 0.0,
	                    'n_3A_soll': 0.0, 'n_3B_soll': 0.0,
	                    'n_4A_soll': 0.0, 'n_4B_soll': 0.0,
	                    'alpha_1_soll': 0.0, 'alpha_2_soll': 0.0,
	                    'alpha_3_soll': 0.0, 'alpha_4_soll': 0.0,
	                    'alpha_soll': 0.0, 'n_soll': 0.0, 'w_soll': 0.0, 'cmd': 0.0,
	                    'sel': 0.0, 'slew_angle_setpoint': 0.0,
	                    'slew_speed_setpoint': 0.0, 'luff_angle_setpoint': 0.0,
	                    'luff_setpoint': 0.0, 'spare_1': 0.0, 'spare_2': 0.0,
	                    'spare_1': 0.0, 'spare_2': 0.0, 'spare_3': 0.0,
	                    'spare_4': 0.0, 'spare_5': 0.0, 'spare_6': 0.0,
	                    'spare_7': 0.0, 'spare_8': 0.0, 'spare_9': 0.0,
	                    'spare_10': 0.0, 'spare_11': 0.0, 'spare_12': 0.0,
	                    'spare_13': 0.0, 'spare_14': 0.0, 'spare_15': 0.0,
	                    'Mode_A': False, 'Mode_B': False, 'Mode_C': False,
	                    'Mode_1': False, 'Mode_2': False, 'Mode_3': False,
	                    'Operating_Input_Y': 0, 'Operating_Input_X': 0,
	                    'Operating_Input_OMG': 0, 'Operating_Input_OMG2': 0,
	                    'ES': False, 'ST': False}
        else:
		print "Error: Init Class PLC"


    def connect(self):
        try:
            self.cl.connect(self.ip, self.rack, self.slot)
            print "Connected to PLC %s" % self.ip
        except:
            print "Connecting to PLC %s not possible" % self.ip

    def disconnect(self):
        self.cl.disconnect()
        print "Disconnected to PLC %s" % self.ip

    # Get communication state to plc
    def state(self):
	print("inside the states")
        return self.cl.get_connected()

    # Send data ROS -> PLC
    def send_mc(self):
        try:
            nr = self.db_nr
            size = self.db_size
            data = bytearray(size)
            param = self.data_mc
            s7 = snap7.util
            s7.set_real(data, 0, float(param['n_1A_soll']))
            s7.set_real(data, 4, float(param['n_1B_soll']))
            s7.set_real(data, 8, float(param['n_2A_soll']))
            s7.set_real(data, 12, float(param['n_2B_soll']))
            s7.set_real(data, 16, float(param['n_3A_soll']))
            s7.set_real(data, 20, float(param['n_3B_soll']))
            s7.set_real(data, 24, float(param['n_4A_soll']))
            s7.set_real(data, 28, float(param['n_4B_soll']))
            s7.set_real(data, 32, float(param['alpha_1_soll']))
            s7.set_real(data, 36, float(param['alpha_2_soll']))
            s7.set_real(data, 40, float(param['alpha_3_soll']))
            s7.set_real(data, 44, float(param['alpha_4_soll']))
            s7.set_real(data, 48, float(param['alpha_soll']))
            s7.set_real(data, 52, float(param['n_soll']))
            s7.set_real(data, 56, float(param['w_soll']))
            s7.set_real(data, 60, float(param['cmd']))
            s7.set_real(data, 64, float(param['sel']))
            s7.set_real(data, 68, float(param['slew_angle_setpoint']))
            s7.set_real(data, 72, float(param['slew_speed_setpoint']))
            s7.set_real(data, 76, float(param['luff_angle_setpoint']))
            s7.set_real(data, 80, float(param['luff_setpoint']))
            s7.set_real(data, 84, float(param['spare_1']))
            s7.set_real(data, 88, float(param['spare_2']))
            s7.set_real(data, 92, float(param['spare_3']))
            s7.set_real(data, 96, float(param['spare_4']))
            s7.set_real(data, 100, float(param['spare_5']))
            s7.set_real(data, 104, float(param['spare_6']))
            s7.set_real(data, 108, float(param['spare_7']))
            s7.set_real(data, 112, float(param['spare_8']))
            s7.set_real(data, 116, float(param['spare_9']))
            s7.set_real(data, 120, float(param['spare_10']))
            s7.set_real(data, 124, float(param['spare_11']))
            s7.set_real(data, 128, float(param['spare_12']))
            s7.set_real(data, 132, float(param['spare_13']))
            s7.set_real(data, 136, float(param['spare_14']))
            s7.set_real(data, 140, float(param['spare_15']))
            s7.set_bool(data, 144, 0, param['ES'])
            s7.set_bool(data, 144, 1, param['ST'])
            s7.set_bool(data, 144, 2, param['Mode_A'])
            s7.set_bool(data, 144, 3, param['Mode_B'])
            s7.set_bool(data, 144, 4, param['Mode_C'])
            s7.set_bool(data, 144, 5, param['Mode_1'])
            s7.set_bool(data, 144, 6, param['Mode_2'])
            s7.set_bool(data, 144, 7, param['Mode_3'])
            s7.set_int(data, 146, int(param['Operating_Input_Y']))
            s7.set_int(data, 148, int(param['Operating_Input_X']))
            s7.set_int(data, 150, int(param['Operating_Input_OMG']))
            s7.set_int(data, 152, int(param['Operating_Input_OMG2']))
            self.cl.db_write(nr, 0, data)
        except:
            print "Error: Send data to PLC %s" % self.ip

    def send_bw(self):
        try:
	    nr = self.db_nr
	    size = self.db_size
	    data = bytearray(size)
	    param = self.data_bw
	    s7 = snap7.util
	    s7.set_int(data, 0, int(param['Operating_Input_X']))
	    print("sent x velocity")
	    s7.set_int(data, 2, int(param['Operating_Input_Y']))
	    s7.set_int(data, 4, int(param['Operating_Input_OMG']))
	    self.cl.db_write(nr, 0, data)
        except:
            print "Error: Send data to PLC %s" % self.ip

    def get_vel_bw(self, msg):
	self.data_bw['Operating_Input_X'] = msg.linear.x
	self.data_bw['Operating_Input_Y'] = msg.linear.y
	self.data_bw['Operating_Input_OMG'] = msg.angular.z
	self.send_bw()
	print("PLC_COM X: ", msg.linear.x)
	print("PLC_COM Y: ", msg.linear.y)

    def get_vel_mc(self, msg):
	self.data_mc['Operating_Input_X'] = msg.linear.x
	self.data_mc['Operating_Input_Y'] = msg.linear.y
	self.data_mc['Operating_Input_OMG'] = msg.angular.z
	print("PLC_COM X: ", msg.linear.x)
	print("PLC_COM Y: ", msg.linear.y)	
	self.send_mc()
	#self.send_bw()


def main():
    print("initial")
    rospy.init_node('plc_com')
    plc_bw = Plc('192.168.1.100', 0, 1, 19, 6, "/bw_cmd_vel", "bw")
    #plc_mc = Plc('192.168.1.1', 0, 1, 24, 154, "/mc_cmd_vel", "mc")
    #plc_mc = Plc('192.168.0.100', 0, 1, 19, 6, "/mc_cmd_vel", "mc")
    while not rospy.is_shutdown():
		if not plc_bw.state():
			plc_bw.disconnect()
			time.sleep(2)
			plc_bw.connect()
		print("===== Start =====")
		#if not plc_mc.state():
		#	plc_mc.disconnect()
		#	time.sleep(2)
		#	plc_mc.connect()
		time.sleep(10)

    plc_bw.disconnect()
    #plc_mc.disconnect()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print "Error: Shutdown"
