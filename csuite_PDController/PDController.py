from control.Controller import Controller, ws
import numpy as np


class PDController(Controller):
    def start(self, states):
        return None

    def update(self, states):
        q = np.asarray(states[ws.actual].getJointPosition()).reshape(-1)
        qd = np.asarray(states[ws.actual].getJointVelocity()).reshape(-1)
        des_q = np.asarray(states[ws.desired].getJointPosition()).reshape(-1)
        des_qd = np.asarray(states[ws.desired].getJointVelocity()).reshape(-1)

        kp = np.array([300.,300.,200., 300.,300.,200., 300.,300.,200., 300.,300.,200.])
        kd = np.array([10.,10.,6., 10.,10.,6., 10.,10.,6., 10.,10.,6.])
        return np.asmatrix(kp * (des_q - q) - kd * qd).T