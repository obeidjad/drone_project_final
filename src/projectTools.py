#!/usr/bin/env python
class DroneCommand:
    def __init__(self,P,I,D):
        self._P = P
        self._I = I
        self._D = D
        self.TotErr = 0
        self.oldErr = 0
        self.cmd = 0
        self.MAX_CMD = 10
    def computeCommand(self,currentVal,targetVal):
        err = targetVal - currentVal
        if(abs(self.cmd) < self.MAX_CMD):
            self.TotErr = self.TotErr + err
        if(self.oldErr != 0):
            Derr = err - self.oldErr
        else:
            Derr = 0
        self.cmd = self._P*err + self.TotErr*self._I + self._D*Derr
        return self.cmd
class GenTools:
    def __init__(self):
        pass
    @staticmethod
    def setMax(val,vmax):
        vmax = abs(vmax)
        if(val > vmax):
            return vmax
        elif(val <  -vmax):
            return -vmax
        else:
            return val