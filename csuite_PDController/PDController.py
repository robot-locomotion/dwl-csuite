from control.Controller import Controller, ws
import numpy as np


class PDController(Controller):
    def start(self, states):
        return None

    def update(self, states):
        f = 0.#math.sin(0.1* 2.*3.14*self.t)
        q = states[ws.actual].getJointPosition()
        qd = states[ws.actual].getJointVelocity()

        q = np.asarray(q).reshape(-1)
        qd = np.asarray(qd).reshape(-1)
        des_q = np.array([-0.2, 0.75+f, -1.5, -0.2, -0.75, 1.5, -0.2, 0.75, -1.5, -0.2, -0.75, 1.5])
        kp = np.array([300.,300.,200., 300.,300.,200., 300.,300.,200., 300.,300.,200.])
        kd = np.array([10.,10.,6., 10.,10.,6., 10.,10.,6., 10.,10.,6.])
        return np.asmatrix(kp * (des_q - q) - kd * qd).T