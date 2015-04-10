# coding: utf-8
"""pyLinkam

Copyright 2014-2015 Mick Phillips (mick.phillips at gmail dot com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
=============================================================================

python wrapper for Linkam's T95 interfacing DLL.

Requires .NET Framework 4.5.1 Full.
"""

import clr
import sys
import ctypes
import time

DLL_PATH = r"C:\b24\pyLinkam\linkam"

sys.path.append(DLL_PATH)
clr.AddReference(r"LinkamCommsLibrary.dll")
import LinkamCommsDll


class _IntParser(object):
    """A class that parses bits in a (u)Int to attributes of itself."""
    # Mapping of bit numbers to attribute names. Override.
    _bitFields = {}
    def __init__(self):
        # Initialise empty attributes.
        for key, bit in self._bitFields.iteritems():
            setattr(self, key, None)


    def update(self, uInt):
        """Update attributes with new values."""
        self._value = uInt
        for key, bit in self._bitFields.iteritems():
            setattr(self, key, self._value & 2**bit > 0)


class _StageConfig(_IntParser):
    # stageTypes and bitFields from SDK docs, appear to be correct.
    _stageTypes = {2**0: 'standard',
                   2**1: 'high temp',
                   2**2: 'peltier',
                   2**3: 'graded',
                   2**4: 'tensile',
                   2**5: 'dsc',
                   2**6: 'warm',
                   2**7: 'ito',
                   2**8: 'css450',
                   2**9: 'correlative',}

    _bitFields = {'motorisedX': 49,
                  'motorisedY': 50}


    def update(self, u64):
        super(_StageConfig, self).update(u64)
        self.stageType = self._stageTypes[u64 & 0b1111111111]
        

class _StageStatus(_IntParser):
    # These bitFields are as per the documentation, but they are incorrect.
    _bitFields = {'errorState': 0,
                  'heater1RampAtSetPoint': 1,
                  'heater1Started': 2,
                  'heater2RampAtSetPoint': 3,
                  'heater2Started': 4,
                  'vacuumRampAtSetPoint': 5,
                  'vacuumControlStarted': 6,
                  'vacuumValveTAtMaxVac': 7,
                  'vacuumValveAtAtmos': 8,
                  'humidityRampAtSetPoint': 9, 
                  'humidityStarted': 10,
                  'coolingPumpPumping': 11,
                  'coolingPumpInAutoMode': 12,
                  'Ramp1': 13,
                  'xMotorAtMinTravel': 41,
                  'xMotorAtMaxTravel': 42,
                  'xMotorStopped': 43,
                  'yMotorAtMinTravel': 44,
                  'yMotorAtMaxTravel': 45,
                  'yMotorStopped': 46,
                  'zMotorAtMinTravel': 47,
                  'zMotorAtMaxTravel': 48,
                  'zMotorStopped': 49,
                  'heater1SampleCalibrationApplied': 50,
                  }


class LinkamStage(object):
    """An interface to Linkam's .net assembly for their stages."""
    # Enumerated value types for SetValue and GetValue.
    eVALUETYPE = LinkamCommsDll.Comms.eVALUETYPE
    XMOTOR_BIT = 2**45
    YMOTOR_BIT = 2**48

    def __init__(self):
        self.stage = LinkamCommsDll.Comms()
        self.stageConfig = _StageConfig()
        self.status = _StageStatus()
        

    def connect(self):
        result = self.stage.OpenComms(True, 0, 0)
        connected = result & 0x0001
        if connected:
            self.stageConfig.update(self.stage.GetStageConfig())
            self.status.update(self.stage.GetStatus())
        return dict(connected = connected,
                    commsNotResponding = result & 0b0010,
                    commsFailedToSendConfigData = result & 0b0100,
                    commsSerialPortError = result & 0b1000)


    def getConfig(self):
        configWord = self.stage.GetStageConfig()
        self.stageConfig.update(configWord)
        return self.stageConfig


    def getStatus(self):
        statusWord = self.stage.GetStatus()
        self.status.update(statusWord)
        return self.status


    def homeMotors(self):
        """Home the motors.
        From CMS196.exe CMS196.frmMotorControl.btnIndex_Click.
        """
        self.moveToXY(-12000., -4000.)


    def moveToXY(self, x, y):
        """Move stage motors to position (x, y)"""
        xValueID = self.eVALUETYPE.u32XMotorLimitRW.value__
        yValueID = self.eVALUETYPE.u32YMotorLimitRW.value__
        self.stage.SetValue(xValueID, x)
        self.stage.StartMotors(True, 0)
        self.stage.SetValue(yValueID, y)
        self.stage.StartMotors(True, 1)


    def isMoving(self):
        """Factored out from moveToXY.
        Blocking until move is completed will cause timeout errors when
        called remotely.
        """
        stoppedCount = 0
        maxCount = 3
        while stoppedCount < maxCount:
            status = self.stage.GetStatus()
            if status & self.XMOTOR_BIT and status & self.YMOTOR_BIT:
                stoppedCount += 1
            else:
                return True
            time.sleep(0.001)
        return False


def main():
    import time
    stage = LinkamStage()
    stage.connect()

    while True:
        stage.getStatus()
        sys.stdout.write("\033[2J")
        sys.stdout.write("\x1b[1;1H")
        for (key, value) in stage.status.__dict__.iteritems():
            sys.stdout.write(u'%s  %f\n' % (key.ljust(25), value))
        time.sleep(1)

if __name__ == '__main__':
    main()