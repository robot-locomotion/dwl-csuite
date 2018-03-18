from control.Controller import Controller, ws
import numpy as np
import dwl, os, sys


class csuite_PDController(Controller):
    def __init__(self, fbs, filename):
        path = os.path.dirname(os.path.abspath(__file__)) + '/' + filename
        Controller.__init__(self, fbs, path)

        self.kp = np.zeros(self.fbs.getJointDoF())
        self.kd = np.zeros(self.fbs.getJointDoF())
        for j in range(self.fbs.getJointDoF()):
            name = self.fbs.getJointNames()[j]
            ctrl_ns = ['pd_controller', 'gains', name]
            read, self.kp[j] = self.yaml.readDouble('p', ctrl_ns)
            if not read:
                sys.exit('You need to specific the p gains')
            read, self.kd[j] = self.yaml.readDouble('d', ctrl_ns)
            if not read:
                sys.exit('You need to specific the d gains')

    def start(self, states):
        return None

    def update(self, states):
        q = np.asarray(states[ws.actual].getJointPosition()).reshape(-1)
        qd = np.asarray(states[ws.actual].getJointVelocity()).reshape(-1)
        des_q = np.asarray(states[ws.desired].getJointPosition()).reshape(-1)
        des_qd = np.asarray(states[ws.desired].getJointVelocity()).reshape(-1)

        return np.asmatrix(self.kp * (des_q - q) + self.kd *(des_qd - qd)).T