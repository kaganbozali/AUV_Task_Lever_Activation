#IMPORTS
import math
import rospy
import numpy as np

from dronekit import connect, VehicleMode
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Int32
from bluerov_gii.srv import MoveCameraSrv
from bluerov_gii.msg import Blob
from bluerov_gii.srv import MoveGripperSrv

'''
m_1 = 9.091 # First Position Mean Value is equal to 8
m_2 = 58 # Last Position Mean Value equal to 63.66
A = np.array([[m_1, 1], [m_2, 1]])
B = np.array([2000, 1500])
X = np.linalg.solve(A, B)
print(X)
'''

connection_string="127.0.0.1:14550"
print("Connecting to vehicle on: %s" % (connection_string,))
autopilot = connect(connection_string, wait_ready=False)

###################################################### DESIRED INPUT SCENARIOS #################################################

colors = ["green","blue","red"]
actions = ["pull","push"]
d_col = colors[1]
d_act = actions[0]

# Initial Lateral Movement for Green Color
if d_col == colors[0]:
	autopilot.channels.overrides['6'] = 1600
	print('Green Task')

# Initial Lateral Movement for Red Color
if d_col == colors[2]:
	autopilot.channels.overrides['6'] = 1400
	print('Red Task')

# Initial Lateral Movement for Blue Color
else:
	autopilot.channels.overrides['6'] = 1500	

def error(D_obtained,D_reel):
	return (D_obtained-D_reel)
def p_func(K,e_f):
	return K*e_f

##################################################### LATERAL POSITION ADJUSTMENT #############################################

def lateral(data):
	global l_pos
	global l_signal
	# Green Lateral Position Arrengement
	if d_col == colors[0]:
		l_pos = data.pose[1].position.y
		error_l = error(l_pos,-0.42) # Error Function
		P_l = []
		time_l = []
		if error_l <= 0:
			autopilot.channels.overrides['6'] = 1498
		    	P_l.append(p_func(1498,error_l))
		    	time_l.append(rospy.get_rostime())
		    	if error_l > -0.02:
				l_signal = True
		    	else:
			    	l_signal = False
	    	if error_l > 0:
		    	autopilot.channels.overrides['6'] = 1502
		    	P_l.append(p_func(1502,error_l))
		    	time_l.append(rospy.get_rostime())
		    	if error_l <= 0.05:
			    	l_signal = True
		    	else:
			    	l_signal = False
	    	else:
			return
		if l_signal == True:
			autopilot.channels.overrides['3'] = 1450
		# I added initial depth speed here because otherwise I could not figure it out to make depth controller after the lateral controller.
	# RED Lateral Postion Arrangement
	if d_col == colors[2]:
		l_pos = data.pose[1].position.y
		error_l = error(l_pos,0.42) # Error Function
		P_l = []
		time_l = []
		if error_l <= 0:
			autopilot.channels.overrides['6'] = 1498
		    	P_l.append(p_func(1498,error_l))
		    	time_l.append(rospy.get_rostime())
		    	if error_l > -0.02:
				l_signal = True
		    	else:
			    	l_signal = False
	    	if error_l > 0:
		    	autopilot.channels.overrides['6'] = 1502
		    	P_l.append(p_func(1502,error_l))
		    	time_l.append(rospy.get_rostime())
		    	if error_l <= 0.05:
			    	l_signal = True
		    	else:
			    	l_signal = False
	    	else:
			return
		if l_signal == True:
			autopilot.channels.overrides['3'] = 1450
	if d_col == colors[1]:
		l_signal = True	
	else:
		return	
	    
	#print(time)
	#print(P,'P')
	#print(l_pos,'Lateral Position')
	#print(l_signal)
	#print(error_l, 'Long Controller Error')

######################################################## DEPTH ADJUSTMENT ##################################################

def model_states_callback(data):
	global depth
	global depth_signal
    	if l_signal == True:
	    	depth= data.pose[1].position.z
	    	error_func = error(depth,-1) # Error Function
	    	P = []
	    	time = []
	    	if error_func <= 0:
			autopilot.channels.overrides['3'] = 1550
		    	P.append(p_func(1550,error_func))
		    	time.append(rospy.get_rostime())
		    	if error_func > -0.02:
				depth_signal = True
		    	else:
			    	depth_signal = False
	    	if error_func > 0:
		    	autopilot.channels.overrides['3'] = 1490
		    	P.append(p_func(1490,error_func))
		    	time.append(rospy.get_rostime())
		    	if error_func <= 0.05:
			    	depth_signal = True
		    	else:
			    	depth_signal = False
    	else:
        	return
			    
	#print(time)
	#print(P,'P')
	#print(depth,'DEPTH')
	#print(depth_signal)
	#print(error_func, 'ERROR')

### CAMERA POSITION ###

def camera_callback(data):
	print ("Camera Position: %s" %data)

###################################################### COLOR DETECTION AND LONGITUDINAL MOTION #############################################
# GREEN
all_rad = []
def greencolor_callback(data):
	global all_rad
	global long_signal
	if d_col == colors[0]:
		if depth_signal == True:
			size = 10
			all_rad.append(data.radius)
			#print(data.radius)
			if len(all_rad) == size:
				all_rad.pop(0)
				str_values = (str(all_rad[0:size-1]).replace("d","").replace("a","").replace("t","").replace(":","").replace(" ",""))
				int_list = [int(num) for num in eval(str_values)]
				mean = np.mean(int_list)
				print(int_list)
				print(mean)
				#m_1 = 8 # First Position Mean Value is equal to 8
				#m_2 = 63.66 # Last Position Mean Value equal to 63.66
				#A = np.array([[m_1, 1], [m_2, 1]])
				#B = np.array([2000, 1500])
				#X = np.linalg.solve(A, B)
				#print(X)
				y = 2071.864 - 8.98 * mean
				if y > 2000:
					y = 2000
					autopilot.channels.overrides['5'] = y
					long_signal = False
		    		if y < 2000 and y > 1500:
					autopilot.channels.overrides['5'] = y
					long_signal = False
		    		if y < 1500:
		        		long_signal = True
					if d_act == actions[0]:
						move_gripper_srv(Int32(0)) # Close the Gripper
						print("Green and Pull")
						autopilot.channels.overrides['5'] = 1450
					if d_act == actions[1]:
						move_gripper_srv(Int32(0)) # Close the Gripper
						print("Green and Push")
						autopilot.channels.overrides['5'] = 1550
				print(y)
				print(long_signal)
		if depth_signal == False:
			autopilot.channels.overrides['5'] = 1500		
# BLUE
def bluecolor_callback(data):
	global all_rad
	global long_signal
	if d_col == colors[1]:
		if depth_signal == True:
			size = 10
			all_rad.append(data.radius)
			#print(data.radius)
			if len(all_rad) == size:
				all_rad.pop(0)
				str_values = (str(all_rad[0:size-1]).replace("d","").replace("a","").replace("t","").replace(":","").replace(" ",""))
				int_list = [int(num) for num in eval(str_values)]
				mean = np.mean(int_list)
				print(int_list)
				print(mean)
				y = 2071.864 - 8.98 * mean
				if y > 2000:
					y = 2000
					autopilot.channels.overrides['5'] = y
				if y < 2000 and y >= 1500:
					autopilot.channels.overrides['5'] = y
		    		if y < 1500:
		        		long_signal = True
					if d_act == actions[0]:
						move_gripper_srv(Int32(0)) # Close the Gripper
						print("Blue and Pull")
						autopilot.channels.overrides['5'] = 1450
					if d_act == actions[1]:
						move_gripper_srv(Int32(0)) # Close the Gripper
						print("Blue and Push")
						autopilot.channels.overrides['5'] = 1550
				print(y)
		if depth_signal == False:
			autopilot.channels.overrides['5'] = 1500
# RED
def redcolor_callback(data):
	global all_rad
	global long_signal
	if d_col == colors[2]:
		if depth_signal == True:
			size = 10
			all_rad.append(data.radius)
			#print(data.radius)
			if len(all_rad) == size:
				all_rad.pop(0)
				str_values = (str(all_rad[0:size-1]).replace("d","").replace("a","").replace("t","").replace(":","").replace(" ",""))
				int_list = [int(num) for num in eval(str_values)]
				mean = np.mean(int_list)
				print(int_list)
				print(mean)
				y = 2071.864 - 8.98 * mean
				if y > 2000:
					y = 2000
					autopilot.channels.overrides['5'] = y
					long_signal = False
		    		if y < 2000 and y > 1500:
					autopilot.channels.overrides['5'] = y
					long_signal = False
		    		if y < 1500:
		        		long_signal = True
					if d_act == actions[0]:
						move_gripper_srv(Int32(0)) # Close the Gripper
						print("Red and Pull")
						autopilot.channels.overrides['5'] = 1450
					if d_act == actions[1]:
						move_gripper_srv(Int32(0)) # Close the Gripper
						print("Red and Push")
						autopilot.channels.overrides['5'] = 1550					
				print(long_signal)
				print(y)
		if depth_signal == False:
			autopilot.channels.overrides['5'] = 1500

#CODE TO EXECUTE

if __name__ == "__main__":

	l_signal = False

	depth_signal = False

	long_signal = False

	rospy.init_node("mixture")

	rospy.Subscriber("/gazebo/model_states", ModelStates, lateral)

	rospy.Subscriber("/gazebo/model_states", ModelStates, model_states_callback)

	rospy.Subscriber("/cameraPosition", Int32,camera_callback)

	rospy.Subscriber("/color/green/blob", Blob, greencolor_callback)

	rospy.Subscriber("/color/blue/blob", Blob, bluecolor_callback)

	rospy.Subscriber("/color/red/blob", Blob, redcolor_callback)

	move_camera_srv=rospy.ServiceProxy("/moveCamera",MoveCameraSrv)

	move_gripper_srv=rospy.ServiceProxy("/moveGripper",MoveGripperSrv)

	move_gripper_srv(Int32(35))

    # Add Grabber Callbacks here # Get some vehicle attributes (state)
	print ("Get some vehicle attribute values:")
	print (" GPS: %s" % autopilot.gps_0)
	print (" Battery: %s" % autopilot.battery)
	print (" System status: %s" % autopilot.system_status.state)
	print (" Mode: %s" % autopilot.mode.name )

	# Arm to move the robot
	autopilot.armed=True
	print ("%s" % autopilot.armed)

	rospy.spin()


