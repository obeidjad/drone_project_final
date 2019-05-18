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
class Sequence:
    def __init__(self,mode='init'):
        self._mode = mode
        self._phase = 0
        self._published=False
    def get_phase(self):
        return self._phase
    def get_mode(self):
        return self._mode
    def get_publ(self):
        return self._published
    def set_phase(self,val):
        self._phase = val
    def set_published(self,val):
        self._published = val
    def reset_seq(self):
        self._published = False
        self._phase = 0
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