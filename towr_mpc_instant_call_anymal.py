import pybullet as p
import os
import time
import numpy as np
import pybullet_data

from ctypes import *

lib = CDLL("./build/libtowr_anymal_dll.so")

file_path = "./rsc/anymal/"


class Trajectory_data2(Structure):
    _fields_ = [
            ('base_linear', c_float*3),
            ('base_angular',c_float*3),
            ('ee_linear', c_float*12),
            ('ee_force', c_float*12)
            
            ]
def print_traj2(a,no_of_samples):
	time = 0.00
	for i in range(no_of_samples+1):
		print("\n\ni :",i," time :",i*2.0/no_of_samples)
		print("base_linear:",a[i].base_linear[0],"\t",a[i].base_linear[1],"\t",a[i].base_linear[2])
		print("base_angular:",a[i].base_angular[0],"\t",a[i].base_angular[1],"\t",a[i].base_angular[2])
		for j in range(4):
			print("leg_no:",j)
			print("\tee_linear_leg:",a[i].ee_linear[0+3*j],"\t",a[i].ee_linear[1+3*j],"\t",a[i].ee_linear[2+3*j])
			#print("\tJoint_angles:",towr_joint_angles[i][0+3*j:3+3*j])
			print("\tee_force_leg:", a[i].ee_force[0+3*j],"\t",a[i].ee_force[1+3*j],"\t",a[i].ee_force[2+3*j])
			#print("\tJoint_Torques:",towr_joint_torques[i][0+3*j:3+3*j])
def cal_towr_code(init_pos,init_ang,target_pos,target_angle,ee1,ee2,ee3,ee4,terrain_id,gait_pattern):
	target = float_array_3()
	base_i_p= float_array_3()
	base_i_a = float_array_3()
	target_a= float_array_3()
	ee_1 = float_array_3()
	ee_2 = float_array_3()
	ee_3 = float_array_3()
	ee_4 = float_array_3()
	for i in range(3):
		target[i] = target_pos[i]
		base_i_p[i] = init_pos[i]
		base_i_a[i] = init_ang[i]
		target_a[i] = target_angle[i]
		ee_1[i] = ee1[i]
		ee_2[i] = ee2[i]
		ee_3[i] = ee3[i]
		ee_4[i] = ee4[i]
	arr_size = no_of_samples+1
	lib.Trajectory.restype = None
	lib.Trajectory.argtypes = [ Trajectory_data2*(arr_size) ,c_float*3,c_float*3,
	                                                         c_float*3,c_float*3,
	                                                         c_float*3,c_float*3,
	                                                         c_float*3,c_float*3,c_int,c_int,c_int]
	print("\nTarget:")
	for i in range(3):
		print(target[i])
    
	traj_array = Trajectory_data2*(arr_size)
	Result_traj = traj_array()

	#for i in range(no_of_samples):
	#	print("i+1 :",i+1," 2/(i+i) :",2.0/(i+1)," (i)*2.0/(i+1) :",(i)*2.0/(i+1))
	lib.Trajectory(Result_traj,base_i_p,base_i_a,
		                     ee_1,ee_2,ee_3,ee_4,
		                     target,target_a, no_of_samples,terrain_id,gait_pattern)
	#print_traj2(Result_traj,arr_size,no_of_samples)
	return(Result_traj)
def set_anymal(anymal,base_pos,base_quat,ee1,ee2,ee3,ee4):
	angles = []
	for i in range(30):
		p.resetBasePositionAndOrientation(anymal,base_pos,base_quat)

		leg_LF = p.calculateInverseKinematics(anymal,5 ,ee1)#eight_vertx_LF[j])
		leg_RF = p.calculateInverseKinematics(anymal,10,ee2)#eight_vertx_RF[j])
		leg_LH = p.calculateInverseKinematics(anymal,15,ee3)#eight_vertx_LH[j])
		leg_RH = p.calculateInverseKinematics(anymal,20,ee4)#eight_vertx_RH[j])
		p.setJointMotorControlArray(bodyUniqueId=anymal,
	                                jointIndices = [1,2,3],
	                                controlMode=p.POSITION_CONTROL,
	                                targetPositions=leg_LF[0:3],
	                                 )
		p.setJointMotorControlArray(bodyUniqueId=anymal,
	                                jointIndices = [6,7,8],
	                                controlMode=p.POSITION_CONTROL,
	                                targetPositions=leg_RF[3:6],
	                                 )
		p.setJointMotorControlArray(bodyUniqueId=anymal,
	                                jointIndices = [11,12,13],
	                                controlMode=p.POSITION_CONTROL,
	                                targetPositions=leg_LH[6:9],
	                                 )
		p.setJointMotorControlArray(bodyUniqueId=anymal,
	                                jointIndices = [16,17,18],
	                                controlMode=p.POSITION_CONTROL,
	                                targetPositions=leg_RH[9:12],
	                                 )
		p.stepSimulation()
		angles = leg_LF[0:3]+leg_RF[3:6]+leg_LH[6:9]+leg_RH[9:12]
	return angles
def draw_frame(point,draw_target=False):
	if draw_target:
		p.addUserDebugLine(point,[point[0]+0.2,point[1],point[2]],[0,1,0],5)
		p.addUserDebugLine(point,[point[0],point[1]+0.2,point[2]],[0,1,0],5)
		p.addUserDebugLine(point,[point[0],point[1],point[2]+0.2],[0,1,0],5)
	else:
		p.addUserDebugLine(point,[point[0]+0.1,point[1],point[2]],[1,0,0],5)
		p.addUserDebugLine(point,[point[0],point[1]+0.1,point[2]],[0,1,0],5)
		p.addUserDebugLine(point,[point[0],point[1],point[2]+0.1],[0,0,1],5)
def ik_leg(anymal,leg_index,ee):
    angles=[]
    for i in range(20):
        angles = p.calculateInverseKinematics(anymal,(leg_index+1)*5 ,ee)
        p.stepSimulation()
    if(leg_index == 0):
    	return(angles[0:3])
    elif(leg_index == 1):
    	return(angles[3:6])
    elif(leg_index == 2):
    	return(angles[6:9])
    elif(leg_index == 3):
    	return(angles[9:12])


def find_force(anymal,leg_index,f_tip,angles):
	linear_jacob = p.calculateJacobian(anymal,(leg_index+1)*5,
		                               [0,0,0],angles,
		                               [0,0,0,0,0,0,0,0,0,0,0,0],
		                               [0,0,0,0,0,0,0,0,0,0,0,0])[0]
	
	# 3 x 3
	Jacob_without_base = np.array([linear_jacob[0][6+3*leg_index:9+3*leg_index],
		                           linear_jacob[1][6+3*leg_index:9+3*leg_index],
		                           linear_jacob[2][6+3*leg_index:9+3*leg_index]])
				#print("Sliced:\n",Jacob_without_base)
				#print("Transpose:\n",np.transpose(Jacob_without_base))
	f_tip = np.array(f_tip)
	# tau = J^T.f_tip
	torque_calculated = np.dot(np.transpose(Jacob_without_base),f_tip)
	return(torque_calculated)
	
				
		#towr_traj[i].ee_force[0+3*j:3+3*j])
		
no_of_samples = 250

horizon = 251
float_array_3 = c_float*3
init_base_pos = [0.45,0,0.54] #0.53 for stairs
init_base_quat = [0,0,0,1]

target_pos = [1.9,0,0.54]
target_angle = [0,0,0]
kp = 100
kd = 5
log = 0
coeff_fric = 0.85
gait = 0
log_name = "./videos/towr_stairs.mp4"
save_log = False
'''
Working Combos, for target +- 1.5,0,0
gait -1 , coeff = 0.8,no_of_samples = 30, without angle feed bak
gait -1 , coeff = 0.7,no_of_samples = 20, with angle feed bak
gait -0 , coeff = 0.7,no_of_samples = 20, with angle feed b50
terrain: stairs, target[1.9,0,0.54],initial=[0.45]
gait -0 , coeff = 0.85,no_of_sample = 250,w a fb

'''
def run_towr_tracking(target_pos,target_angle,terrain_id,gait_pattern):
	current_base_pos = init_base_pos
	current_base_ang = p.getEulerFromQuaternion(init_base_quat)
	target_anymal = [target_pos[0],target_pos[1]+2,target_pos[2]]
	p.connect(p.GUI)
	p.resetSimulation()
	plane = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), 0, 0, 0) #5,10,15,20
	p.changeDynamics(plane,-1,lateralFriction=coeff_fric)
	if terrain_id==2:
		boxHalfLength = 0.2
		boxHalfWidth = 2.5
		boxHalfHeight = 0.1
		sh_colBox = p.createCollisionShape(p.GEOM_BOX,halfExtents=[boxHalfLength,boxHalfWidth,0])
		sh_final_col = p.createCollisionShape(p.GEOM_BOX,halfExtents=[0.5,boxHalfWidth,0])
		boxOrigin = 1+boxHalfLength
		step1 = p.createMultiBody(baseMass=0,baseCollisionShapeIndex = sh_colBox,
		                    basePosition = [boxOrigin,1,boxHalfHeight],
		                    baseOrientation=[0.0,0.0,0.0,1])
		# step2 = p.createMultiBody(baseMass=0,baseCollisionShapeIndex = sh_final_col,
		#           basePosition = [boxOrigin+0.5+boxHalfLength,0,boxHalfHeight + 2*boxHalfHeight],
		#                     baseOrientation=[0.0,0.0,0.0,1])
		step2 = p.createMultiBody(baseMass=0,baseCollisionShapeIndex = sh_final_col,
		          basePosition = [boxOrigin+0.5+boxHalfLength,1,0.05 + 2*boxHalfHeight],
		                    baseOrientation=[0.0,0.0,0.0,1])
		p.changeDynamics(step1,-1,lateralFriction=coeff_fric)
		p.changeDynamics(step2,-1,lateralFriction=coeff_fric)
	
	
	draw_frame(target_pos,draw_target=True)
	draw_frame(target_anymal,draw_target=True)
	p.setGravity(0,0,-10)
	anymal = p.loadURDF(file_path+"anymal_no_mass.urdf",[0,2,0.5])
	vis_anymal = p.loadURDF(file_path+"anymal_vis.urdf",[0,0,0.5])

	set_anymal(vis_anymal,init_base_pos,init_base_quat, [init_base_pos[0]+0.34,init_base_pos[1]+0.19,init_base_pos[2]-0.42],
											       [init_base_pos[0]+0.34,init_base_pos[1]-0.19,init_base_pos[2]-0.42],
    										       [init_base_pos[0]-0.34,init_base_pos[1]+0.19,init_base_pos[2]-0.42],
    										       [init_base_pos[0]-0.34,init_base_pos[1]-0.19,init_base_pos[2]-0.42])
	
	set_anymal(anymal,[init_base_pos[0],init_base_pos[1]+2,init_base_pos[2]],init_base_quat, 
		                                           [init_base_pos[0]+0.34,init_base_pos[1]+2+0.19,init_base_pos[2]-0.42],
											       [init_base_pos[0]+0.34,init_base_pos[1]+2-0.19,init_base_pos[2]-0.42],
    										       [init_base_pos[0]-0.34,init_base_pos[1]+2+0.19,init_base_pos[2]-0.42],
    										       [init_base_pos[0]-0.34,init_base_pos[1]+2-0.19,init_base_pos[2]-0.42])
	if(save_log):
		log = p.startStateLogging(loggingType=p.STATE_LOGGING_VIDEO_MP4,fileName=log_name)
	reached_or_out_or_fallen = False
	while(reached_or_out_or_fallen!=True):
		#p.removeAllUserDebugItems()

		#takes the current ee positions of anymal, to compute from that position
		ee1=list(p.getLinkState(anymal,5)[0])
		ee2=list(p.getLinkState(anymal,10)[0])
		ee3=list(p.getLinkState(anymal,15)[0])
		ee4=list(p.getLinkState(anymal,20)[0])
		ee1[1] = ee1[1] - 2
		ee2[1] = ee2[1] - 2
		ee3[1] = ee3[1] - 2
		ee4[1] = ee4[1] - 2

		##print("ee1:",ee1,"ee2:",ee2,"ee3:",ee3,"ee4:",ee4)
		draw_frame(current_base_pos)
		#start = time.time()
		Result_traj = cal_towr_code(init_pos=current_base_pos,init_ang=current_base_ang,
			                        target_pos=target_pos,target_angle=target_angle,
			                        ee1=ee1,ee2=ee2,ee3=ee3,ee4=ee4,
			                        terrain_id=terrain_id,gait_pattern=gait_pattern)

		#end = time.time()
		#print_traj2(Result_traj,no_of_samples)
		#print("Trajectory_calculation_time:",end - start)
		for towr_sample_index in range(horizon):
			towr_quat = p.getQuaternionFromEuler(Result_traj[towr_sample_index].base_angular)
			#p.resetBasePositionAndOrientation(vis_anymal,Result_traj[towr_sample_index].base_linear ,towr_quat)
			
			#a = p.getBasePositionAndOrientation(vis_anymal)[0]
			# if towr_sample_index!= 0:
			# 	p.addUserDebugLine(b,a,[1,0,0],3)
			angles = set_anymal(vis_anymal,Result_traj[towr_sample_index].base_linear ,towr_quat,
				                           Result_traj[towr_sample_index].ee_linear[0:3],
				                           Result_traj[towr_sample_index].ee_linear[3:6],
										   Result_traj[towr_sample_index].ee_linear[6:9],
				                           Result_traj[towr_sample_index].ee_linear[9:12])
			#b = a
			time.sleep(0.01)
			p.stepSimulation()
			for i in range(4):
				
				
				# leg_state = p.getJointStates(anymal,[1+5*i,2+5*i,3+5*i])

	
				# previous_angles = [leg_state[0][0],
				# 				   leg_state[1][0],
				# 				   leg_state[2][0]]
				# previous_velocities = [leg_state[0][1],
				# 				   	   leg_state[1][1],
				# 				   	   leg_state[2][1]]

				#print("angles:",previous_angles,"vel:",previous_velocities)
				if(i == 0):
					dezired_leg_angles = angles[0:3]
				elif(i == 1):
					dezired_leg_angles = angles[3:6]
				elif(i == 2):
					dezired_leg_angles = angles[6:9]
				elif(i == 3):
					dezired_leg_angles = angles[9:12]
				
				# f_tip = Result_traj[towr_sample_index].ee_force[0+3*i:3+3*i]
				# torques = find_force(vis_anymal,i,f_tip,angles)
				# motor_torques = [0,0,0]

				# for i in range(3):
				# 	motor_torques[i] = kp*(dezired_leg_angles[i] - previous_angles[i]) + kd*(0 - previous_velocities[i]) 
				#print("motor_torques:",motor_torques)
				#if(len(p.getContactPoints(plane,vis_anymal,-1,5*(i+1)))==0):
				#print("POSITION_CONTROL")
				#if (towr_sample_index != 0):
				p.setJointMotorControlArray(bodyUniqueId=anymal,
	                                		jointIndices = [1+5*i,2+5*i,3+5*i],
	                                		controlMode=p.POSITION_CONTROL,
	                                		targetPositions=dezired_leg_angles,
	                                		targetVelocities=[0,0,0])
				# else:
				#print("TORQUE_CONTROL")
				# p.setJointMotorControlArray(bodyUniqueId=anymal,
	   #                              			jointIndices = [1+5*i,2+5*i,3+5*i],
	   #                              			controlMode=p.TORQUE_CONTROL,
	   #                              			forces=motor_torques)
	 			


	    
		current_base_anymal = p.getBasePositionAndOrientation(anymal)
		'''
		set_anymal(anymal,current_base_anymal[0],current_base_anymal[1], 
		                                           [current_base_anymal[0][0]+0.34,current_base_anymal[0][1]+0.19,current_base_anymal[0][2]-0.42],
											       [current_base_anymal[0][0]+0.34,current_base_anymal[0][1]-0.19,current_base_anymal[0][2]-0.42],
    										       [current_base_anymal[0][0]-0.34,current_base_anymal[0][1]+0.19,current_base_anymal[0][2]-0.42],
    										       [current_base_anymal[0][0]-0.34,current_base_anymal[0][1]-0.19,current_base_anymal[0][2]-0.42])
		'''

		if((abs(target_anymal[0])<abs(current_base_anymal[0][0]) 
			and abs(target_anymal[1])<abs(current_base_anymal[0][1]) ) 
			or current_base_anymal[0][2]<0.3 ):
			reached_or_out_or_fallen = True
			#current_base_pos = [2,current_base_anymal[1]-2,current_base_anymal[2]]
		else:
			current_base_pos = [current_base_anymal[0][0],current_base_anymal[0][1]-2,current_base_anymal[0][2]]
			current_base_ang = p.getEulerFromQuaternion(current_base_anymal[1])

#for i in range(no_of_gaits):
#print("Gait_type:",i)
run_towr_tracking(target_pos,target_angle,terrain_id=2,gait_pattern=gait)
#p.stopStateLogging(log)
#if(save_log):
'''
p.connect(p.GUI)
plane = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), 0, 0, 0) #5,10,15,20
p.setGravity(0,0,-10)
anymal = p.loadURDF(file_path+"anymal_no_mass.urdf")
angles=set_anymal(anymal,[init_base_pos[0],init_base_pos[1],init_base_pos[2]],init_base_quat, 
		                                           [init_base_pos[0]+0.34,init_base_pos[1]+0.19,init_base_pos[2]-0.42],
											       [init_base_pos[0]+0.34,init_base_pos[1]-0.19,init_base_pos[2]-0.42],
    										       [init_base_pos[0]-0.34,init_base_pos[1]+0.19,init_base_pos[2]-0.42],
    										       [init_base_pos[0]-0.34,init_base_pos[1]-0.19,init_base_pos[2]-0.42])
while(True):
	time.sleep(0.01)

	f_tip = [0,0,-1000]
	torques = find_force(anymal,0,f_tip,angles)
	print("torques:",torques)
	p.setJointMotorControlArray(bodyUniqueId=anymal,
	                                 			jointIndices = [1,2,3],
	                                 			controlMode=p.TORQUE_CONTROL,
	                                 			targetPositions=[0,0,1.57],
	                                 			forces=[0,0,-70])
	p.stepSimulation()
	'''