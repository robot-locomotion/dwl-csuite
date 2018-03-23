import os, time, sys
import dwl
import numpy as np
from BulletInterface import BulletInterface, pb
import pybullet_data
import importlib, pkgutil
from control.Controller import ws


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class ControlSuite():
    def __init__(self, filename):
        # Parsing the dwl control suite configuration and initialization
        self.parseConfigFile(filename)
        self.loop_sim = False
        self.fixed_base = False
        self.t = 0.

        # Creating the bullet clien and configure it
        if self.enable_gui:
            self.client = pb.connect(pb.GUI, options='--opengl2')
        else:
            self.client = pb.connect(pb.DIRECT)

        # Creating and configurating the bullet interface
        self.biface = BulletInterface(self.client)
        self.biface.setGravity(self.gravity[dwl.X],
                               self.gravity[dwl.Y],
                               self.gravity[dwl.Z])
        self.biface.setTimeStep(self.time_step)
        self.biface.resetDebugVisualizerCamera(self.cam_dist,
                                               self.cam_yaw,
                                               self.cam_pitch,
                                               self.cam_target.tolist())

        # Loading the ground model
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.plane_id = pb.loadURDF('plane.urdf')

        # Loading the robot model to dwl and simulator
        self.fbs = dwl.FloatingBaseSystem()
        self.wkin = dwl.WholeBodyKinematics()
        self.wdyn = dwl.WholeBodyDynamics()
        self.fbs.resetFromURDFFile(self.root_path + self.urdf_path,
                                   self.root_path + self.yarf_path)
        self.wkin.reset(self.fbs)
        self.wdyn.reset(self.fbs, self.wkin)

        pb.setAdditionalSearchPath(self.root_path)
        self.robot_id = pb.loadURDF(self.urdf_path,
                                    self.robot_pos0.tolist(),
                                    pb.getQuaternionFromEuler(self.robot_rpy0),
                                    flags=pb.URDF_USE_INERTIA_FROM_FILE)

        # Creating the actual whole-body state
        self.ws_states = [dwl.WholeBodyState(self.fbs.getJointDoF()), # actual state
                          dwl.WholeBodyState(self.fbs.getJointDoF())] # desired state
    
        self.parseRobot()
        self.resetRobotPosition(self.robot_pos0)
        self.readControllerPlugins()

        # Creating the controller
        if not self.changeController(self.ctrl_name, self.ctrl_config):
            print bcolors.FAIL + 'Error: the default controller is not defined' + bcolors.ENDC
            sys.exit()

        if self.enable_ros:
            self.enableROS()

    def parseConfigFile(self, filename):
        # Default parameters of the control suite
        yaml = dwl.YamlWrapper()
        yaml.setFile(filename)

        # Parsing robot parameters
        robot_ns = ['dwl_csuite', 'robot']
        self.robot_pos0 = np.array([0.,0.,1.])
        yaml.readArray3d(self.robot_pos0, 'init_position', robot_ns)
        self.robot_rpy0 = np.array([0.,0.,0.])
        yaml.readArray3d(self.robot_rpy0, 'init_rpy', robot_ns)

        self.root_path = os.path.dirname(os.path.abspath(__file__)) + '/../data/'
        read, rpath = yaml.readString('root_path',  robot_ns)
        if read:
            self.root_path = rpath
        read, self.urdf_path = yaml.readString('urdf_path',  robot_ns)
        if not read:
            sys.exit('You need to defined the the relative path (w.r.t. root_path) of the urdf file')
        read, self.yarf_path = yaml.readString('yarf_path',  robot_ns)
        if not read:
            sys.exit('You need to defined the the relative path (w.r.t. root_path) of the yarf file')

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

        # Parsing camera position which is used in the Bullet GUI
        cam_ns = ['dwl_csuite', 'simulation', 'camera']
        self.cam_dist = 2.2
        self.cam_yaw = -2.
        self.cam_pitch = -18.2
        self.cam_target = np.array([-0.19, 0.16, -0.17])
        read, cdist = yaml.readDouble('distance', cam_ns)
        if read:
            self.cam_dist = cdist
        read, cyaw = yaml.readDouble('yaw', cam_ns)
        if read:
            self.cam_yaw = cyaw
        read, cpitch = yaml.readDouble('pitch', cam_ns)
        if read:
            self.cam_pitch = cpitch
        yaml.readArray3d(self.cam_target, 'target', cam_ns)

        # Parsing the default control parameters
        ctrl_ns = ['dwl_csuite', 'control']
        self.ctrl_name = 'csuite_PDController'
        read, cname = yaml.readString('default', ctrl_ns)
        if read:
            self.ctrl_name = cname
        else:
            print bcolors.OKGREEN \
             + 'Not defined controller, running the csuite_PDController' \
             + bcolors.ENDC
        self.ctrl_config = ''
        read, fname = yaml.readString('config_file', ctrl_ns)
        if read:
            self.ctrl_config = fname
        else:
            print bcolors.WARNING \
             + 'Warning: there is not defined a configuration file for the controller' \
             + bcolors.ENDC

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

    def run(self):
        self.loop_sim = True
        while self.loop_sim:
            self.update()
            time.sleep(self.time_step)

    def stop(self):
        self.loop_sim = False

    def update(self):
        # Update the actual whole-body state and pass it to the controller
        self.updateActualState()
    
        # Computing the torque commands
        u = self.controller.update(self.ws_states)

        # Sending the torque commands
        self.setJointCommand(np.asarray(u).reshape(-1))        

        # Updating the simulation time
        self.t += self.time_step

        # Running a simulation step
        pb.stepSimulation()

        # Publish ROS messages if it enables
        if self.enable_ros:
            self.ros_iface.update(self.ws_states)
        
        # Reset position for the fixed base state
        if self.fixed_base:
            self.resetRobotPosition(self.fixed_pos)
    
    def changeController(self, ctrl_name, ctrl_config):
        # Checking if the controller exist
        if not ctrl_name in self.controller_plugins:
            print bcolors.WARNING + 'Warning: the ' + self.ctrl_name + ' cannot be found' + bcolors.ENDC
            return False
        
        # Switching controller and starting it
        self.controller = getattr(self.controller_plugins[ctrl_name], ctrl_name)(self.fbs, ctrl_config)
        self.controller.start(self.ws_states)
        return True

    def parseRobot(self):
        # TODO Assuming the I have force/torque sensors in each foot
#        contact_sensors = ['lf_foot_joint', 'lh_foot_joint', 'rf_foot_joint', 'rh_foot_joint']

        # Getting the dictionary that maps the bullet non-fixed joint names to indexes
        self.joints_bullet = dict()
        self.force_sensors = dict()
        for j in range(pb.getNumJoints(self.robot_id)):
            joint_info = pb.getJointInfo(self.robot_id, j)
            if joint_info[2] != pb.JOINT_FIXED:
                self.joints_bullet[joint_info[1]] = j
#            elif joint_info[1] in contact_sensors:
#                pb.enableJointForceTorqueSensor(self.robot_id,j)
#                self.force_sensors[joint_info[1]] = joint_info[0]
        
#        for key in self.force_sensors:
#            print key, self.force_sensors[key]

    def resetRobot(self):
        self.resetRobotPosition(self.robot_pos0)

    def resetRobotPosition(self, robot_pos):
        # Resetting the base position
        pb.resetBasePositionAndOrientation(self.robot_id,
                                           robot_pos.tolist(),
                                           pb.getQuaternionFromEuler(self.robot_rpy0))

        # Resetting the joint position
        for key in self.joints_bullet:
            # Setting to zero unactuacted joints
            if not self.fbs.existJoint(key):
                idx = self.joints_bullet[key]
                pb.resetJointState(self.robot_id, idx, 0.)

        q0 = np.asarray(self.fbs.getDefaultPosture())
        for j in range(self.fbs.getJointDoF()):
            name = self.fbs.getJointNames()[j]
            idx = self.joints_bullet[name]
            pb.resetJointState(self.robot_id, idx, q0[j])
        
        # Setting this posture as desired one
        self.ws_states[ws.desired].setJointPosition(q0)

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

#        for key in self.force_sensors:
#            idx = self.force_sensors[key]
#            force = np.asarray(pb.getJointState(self.robot_id, idx)[2])
#            print force[:6], key, key[:-6], pb.getJointInfo(self.robot_id, idx)[1]
#            force[1] *= -1; force[2] *= -1
#            self.ws_states[ws.actual].setContactWrench_B(key[:-6],
#                                                         np.hstack((force[3:], force[:3])))
#        print '---'

        # Updating the contact states
        base_zero = np.zeros((6,1))
        joint_pos = self.ws_states[ws.actual].joint_pos
        joint_vel = self.ws_states[ws.actual].joint_vel
        joint_acc = self.ws_states[ws.actual].joint_acc
        joint_eff = self.ws_states[ws.actual].joint_eff
        contact_pos_B = self.wkin.computePosition(base_zero, joint_pos,
                                                  self.fbs.getEndEffectorNames(dwl.FOOT),
                                                  dwl.Linear)
        self.ws_states[ws.actual].setContactPositionDict_B(contact_pos_B)
        contact_vel_B = self.wkin.computeVelocity(base_zero, joint_pos,
                                                  base_zero, joint_vel,
                                                  self.fbs.getEndEffectorNames(dwl.FOOT),
                                                  dwl.Linear)
        self.ws_states[ws.actual].setContactVelocityDict_B(contact_vel_B)
        contact_acc_B = self.wkin.computeAcceleration(base_zero, joint_pos,
                                                      base_zero, joint_vel,
                                                      base_zero, joint_acc,
                                                      self.fbs.getEndEffectorNames(dwl.FOOT),
                                                      dwl.Linear)
        self.ws_states[ws.actual].setContactAccelerationDict_B(contact_acc_B)
        contact_forces = dict()
        self.wdyn.estimateContactForces(contact_forces,
                                        base_zero, joint_pos,
                                        base_zero, joint_vel,
                                        base_zero, joint_acc,
                                        joint_eff, self.fbs.getEndEffectorNames(dwl.FOOT))
        self.ws_states[ws.actual].setContactWrenchDict_B(contact_forces)
#        print self.ws_states[ws.actual].getContactWrench_B()

  
    def setJointCommand(self, u):
        # This resets the velocity controller for joints that are not actuated
        for name in self.joints_bullet:
            idx = self.joints_bullet[name]
            pb.setJointMotorControl2(self.robot_id, idx,
                                     pb.VELOCITY_CONTROL,
                                     targetVelocity=0, force=0)
        # Sending the joint torque commands to bullet engine
        for j in range(self.fbs.getJointDoF()):
            name = self.fbs.getJointName(j)
            idx = self.joints_bullet[name]
      
            # Simulating a perfect torque tracking controller
            self.ws_states[ws.actual].setJointEffort(u[j], j)

            # Sending the torque commands to the bullet engine
            pb.setJointMotorControl2(self.robot_id, idx,
                                     pb.TORQUE_CONTROL, force=u[j])

    def enableROS(self):
        from ros.ROSInterface import ROSInterface
        self.ros_iface = ROSInterface(self.fbs, self.pub_rate, pb.getBodyInfo(self.robot_id)[0])
        self.enable_ros = True

    def fixedBase(self):
        if self.fixed_base:
            self.fixed_base = False
        else:
            self.fixed_base = True
            self.fixed_pos = self.robot_pos0

    def fixedBasePosition(self, pos):
        self.fixed_pos = pos
        if self.fixed_base:
            self.fixed_base = False
        else:
            self.fixed_base = True
            self.fixed_pos = self.fixed_pos

    def printActualState(self):
        print self.ws_states[ws.actual]
    
    def printDesiredState(self):
        print self.ws_states[ws.desired]
