import os, time
import dwl
import numpy as np
from BulletInterface import BulletInterface, pb
from ros.ROSInterface import ROSInterface
import pybullet_data
import importlib, pkgutil
from control.Controller import ws
from csuite_PDController.PDController import PDController


class ControlSuite():
    def __init__(self):
        # Parsing the dwl control suite configuration and initialization
        self.parseConfigFile()
        self.loop_sim = False
        self.t = 0.

        # Creating the bullet clien and configure it
        if self.enable_gui:
            self.client = pb.connect(pb.GUI, options="--opengl2")
        else:
            self.client = pb.connect(pb.DIRECT)

        # Creating and configurating the bullet interface
        self.biface = BulletInterface(self.client)
        self.biface.resetDebugVisualizerCamera(2.20, -2., -18.2, [-0.19,0.16,-0.17])
        self.biface.setGravity(self.gravity[dwl.X],
                               self.gravity[dwl.Y],
                               self.gravity[dwl.Z])
        self.biface.setTimeStep(self.time_step)        


        # Loading the ground model
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.plane_id = pb.loadURDF('plane.urdf')

        # Loading the robot model to dwl and simulator
        root_path = '/home/cmastalli/code/robot-description/'
        urdf_path = 'hyq_description/robots/hyq.urdf'
        self.fbs = dwl.FloatingBaseSystem()
        self.wkin = dwl.WholeBodyKinematics()
        self.wdyn = dwl.WholeBodyDynamics()
        self.fbs.resetFromURDFFile(root_path + urdf_path)
        self.wkin.reset(self.fbs)
        self.wdyn.reset(self.fbs, self.wkin)

        pb.setAdditionalSearchPath(root_path)
        self.robot_id = pb.loadURDF(urdf_path,
                                    self.robot_pos0.tolist(),
                                    pb.getQuaternionFromEuler(self.robot_rpy0),
                                    flags=pb.URDF_USE_INERTIA_FROM_FILE)

        # Creating the actual whole-body state
        self.ws_states = [dwl.WholeBodyState(self.fbs.getJointDoF())]

        # Creating the controller
        self.controller = PDController()
    
        self.parseRobot()
        self.resetRobotPosition()
        self.readControllerPlugins()

        if self.enable_ros:
            self.enableROS()


    def parseConfigFile(self):
        # Default parameters of the control suite
        yaml = dwl.YamlWrapper()
        fpath = os.path.dirname(os.path.abspath(__file__))
        yaml.setFile(fpath + '/../config/dwl_csuite.yaml')

        # Parsing the simulation parameters
        sim_ns = ['dwl_csuite', 'simulation']
        self.enable_gui = False
        read, gui = yaml.readBool('gui',  sim_ns)
        if read:
            self.enable_gui = gui

        self.time_step = 1e-3
        read, tstep = yaml.readDouble('time_step', sim_ns)
        if read:
            self.time_step = tstep
        self.gravity = np.array([0.,0.,-9.81])
        yaml.readArray3d(self.gravity, 'gravity', sim_ns)

        # Parsing ROS parameters
        ros_ns = ['dwl_csuite', 'ros']
        self.enable_ros = False
        read, ros = yaml.readBool('enable',  ros_ns)
        if read:
            self.enable_ros = ros
        self.pub_rate = 100.
        read, prate = yaml.readDouble('publish_rate', ros_ns)
        if read:
            self.pub_rate = prate

        # Parsing robot parameters
        robot_ns = ['dwl_csuite', 'robot']
        self.robot_pos0 = np.array([0.,0.,1.])
        yaml.readArray3d(self.robot_pos0, 'init_position', robot_ns)
        self.robot_rpy0 = np.array([0.,0.,0.])
        yaml.readArray3d(self.robot_rpy0, 'init_rpy', robot_ns)

    def run(self):
        self.loop_sim = True
        while self.loop_sim:
            self.update()
            time.sleep(self.time_step)

    def stop(self):
        self.loop_sim = False

    def update(self):
        # Updating the simulation time
        self.t += self.time_step

        # Update the actual whole-body state and pass it to the controller
        self.updateActualState()
    
        # Computing the torque commands
        u = self.controller.update(self.ws_states)

        # Sending the torque commands
        self.setJointCommand(np.asarray(u).reshape(-1))

        if self.enable_ros:
            self.ros_iface.update(self.ws_states)
    
        # Running a simulation step
        pb.stepSimulation()
    
    def setController(self, ctrl):
        self.controller = ctrl

    def parseRobot(self):
        # TODO Assuming the I have force/torque sensors in each foot
        contact_sensors = ['lf_foot_joint', 'lh_foot_joint', 'rf_foot_joint', 'rh_foot_joint']

        # Getting the dictionary that maps the bullet non-fixed joint names to indexes
        self.joints_bullet = dict()
        self.force_sensors = dict()
        for j in range(pb.getNumJoints(self.robot_id)):
            joint_info = pb.getJointInfo(self.robot_id, j)
            if joint_info[2] != pb.JOINT_FIXED:
                self.joints_bullet[joint_info[1]] = j
            elif joint_info[1] in contact_sensors:
                pb.enableJointForceTorqueSensor(self.robot_id,j)
                self.force_sensors[joint_info[1]] = joint_info[0]
        
        for key in self.force_sensors:
            print key, self.force_sensors[key]

    def resetRobotPosition(self):
        # Resetting the base position
        pb.resetBasePositionAndOrientation(self.robot_id,
                                           self.robot_pos0.tolist(),
                                           pb.getQuaternionFromEuler(self.robot_rpy0))

        # Resetting the joint position
        q0 = np.array([-0.2, 0.75, -1.5, -0.2, -0.75, 1.5, -0.2, 0.75, -1.5, -0.2, -0.75, 1.5])
        for j in range(self.fbs.getJointDoF()):
            name = self.fbs.getJointNames()[j]
            idx = self.joints_bullet[name]
            pb.resetJointState(self.robot_id, idx, q0[j])

    def readControllerPlugins(self):
        self.controller_plugins = {
            name: importlib.import_module(name)
            for finder, name, ispkg
            in pkgutil.iter_modules()
            if name.startswith('csuite')
        }

    def updateActualState(self):
        # Update the time
        self.ws_states[ws.actual].setTime(self.t)
    
        # Update the base states
        base_pos, base_q = pb.getBasePositionAndOrientation(self.robot_id)
        base_vel, base_omega = pb.getBaseVelocity(self.robot_id)
        self.ws_states[ws.actual].setBasePosition(np.asarray(base_pos))
        self.ws_states[ws.actual].setBaseRPY(np.asarray(pb.getEulerFromQuaternion(base_q)))
        self.ws_states[ws.actual].setBaseVelocity_W(np.asarray(base_vel))
        self.ws_states[ws.actual].setBaseAngularVelocity_W(np.asarray(base_omega))
    
        # Update the joint states
        for j in range(self.fbs.getJointDoF()):
            name = self.fbs.getJointNames()[j]
            idx = self.joints_bullet[name]
            state = pb.getJointState(self.robot_id, idx)
            self.ws_states[ws.actual].setJointPosition(state[0], j)
            self.ws_states[ws.actual].setJointVelocity(state[1], j)
            self.ws_states[ws.actual].setJointAcceleration(0., j)

        for key in self.force_sensors:
            idx = self.force_sensors[key]
            force = np.asarray(pb.getJointState(self.robot_id, idx)[2])
#            print force
#            self.ws_states[ws.actual].setContactWrench_B('lf_foot',
#                                                         np.hstack((force[3:], force[:3])))
#        print '---'
        # Updating the contact states
        base_pos = 0.*self.ws_states[ws.actual].base_pos
        joint_pos = self.ws_states[ws.actual].joint_pos
        base_vel = 0.*self.ws_states[ws.actual].base_vel
        joint_vel = self.ws_states[ws.actual].joint_vel
        base_acc = 0.*self.ws_states[ws.actual].base_acc
        joint_acc = 0.*self.ws_states[ws.actual].joint_acc
        joint_eff = self.ws_states[ws.actual].joint_eff
        contact_pos_B = self.wkin.computePosition(base_pos, joint_pos,
                                                  self.fbs.getEndEffectorNames(dwl.FOOT),
                                                  dwl.Linear)
        self.ws_states[ws.actual].setContactPositionDict_B(contact_pos_B)
        contact_vel_B = self.wkin.computeVelocity(base_pos, joint_pos,
                                                  base_vel, joint_vel,
                                                  self.fbs.getEndEffectorNames(dwl.FOOT),
                                                  dwl.Linear)
        self.ws_states[ws.actual].setContactVelocityDict_B(contact_vel_B)
        contact_acc_B = self.wkin.computeAcceleration(base_pos, joint_pos,
                                                      base_vel, joint_vel,
                                                      base_acc, joint_acc,
                                                      self.fbs.getEndEffectorNames(dwl.FOOT),
                                                      dwl.Linear)
        self.ws_states[ws.actual].setContactAccelerationDict_B(contact_acc_B)
        contact_forces = dict()
        self.wdyn.estimateContactForces(contact_forces,
                                        base_pos, joint_pos,
                                        base_vel, joint_vel,
                                        base_acc, joint_acc,
                                        joint_eff, self.fbs.getEndEffectorNames(dwl.FOOT))
        self.ws_states[ws.actual].setContactWrenchDict_B(contact_forces)
        #print contact_forces

  
    def setJointCommand(self, u):
        for j in range(self.fbs.getJointDoF()):
            name = self.fbs.getJointNames()[j]
            idx = self.joints_bullet[name]
      
            # Simulating a perfect torque tracking controller
            self.ws_states[ws.actual].setJointEffort(u[j], j)
      
            # Disabling the default velocity controller
            pb.setJointMotorControl2(self.robot_id, idx,
                                     pb.VELOCITY_CONTROL,
                                     targetVelocity=0, force=0)

            # Sending the torque commands to the bullet engine
            pb.setJointMotorControl2(self.robot_id, idx,
                                     pb.TORQUE_CONTROL, force=u[j])

    def enableROS(self):
        self.ros_iface = ROSInterface(self.fbs, self.pub_rate)
        self.enable_ros = True

    def printActualState(self):
        print self.ws_states[ws.actual]