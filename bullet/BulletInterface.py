import pybullet as pb


class BulletInterface():
    def __init__(self, client):
        self.client = client
    
    def setTimeStep(self, time):
        pb.setTimeStep(time, self.client)

    def setGravity(self, grav_x, grav_y, grav_z):
        pb.setGravity(grav_x, grav_y, grav_z, self.client)
    
    def resetDebugVisualizerCamera(self, distance, yaw, pitch, target):
        pb.resetDebugVisualizerCamera(distance, yaw, pitch, target, self.client)

    def setERP(self, erp):
        pb.setPhysicsEngineParameter(erp=erp)

    def setContactERP(self, erp):
        pb.setPhysicsEngineParameter(contactERP=erp)

    def setNumSolverIterations(self, n):
        pb.setPhysicsEngineParameter(numSolverIterations=n)