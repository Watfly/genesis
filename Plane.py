import numpy as np

class Plane:


    def __init__(self,configName,environment):
        self.environment = environment

        self.wings = np.empty()
        self.elevators = np.empty()
        self.payloads = np.empty()
        self.currentState = State()

        #MARK: Create custom configs here
        if configName == 'coffin':
            self.wings[0,1] = Wing()
            self.wings[1] = Wing()


class State:

    #ind is initial displacement in world coordinates (x,y,z)
    def __init__(self,ind=[0,0,0],inv=[0,0,0], ina=[0,0,0]):
        self.disp = np.zeros(ind,dtype=float)
        self.vel = np.zeros(inv,dtype=float)
        self.acc = np.zeros(ina,dtype=float)
        #TODO: Add rotation data

    def updateState(self,disp,vel,acc):
        self.disp = disp
        self.vel = vel
        self.acc = acc
        #TODO: Add rotation data
