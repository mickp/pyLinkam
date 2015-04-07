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

DLL_PATH = r"C:\b24\pyLinkam\linkam"

sys.path.append(DLL_PATH)
clr.AddReference(r"LinkamCommsLibrary.dll")
import LinkamCommsDll


class _IntParser(object):
    _bitFields = {}
    def __init__(self):
        for key, bit in self._bitFields.iteritems():
            setattr(self, key, None)


    def update(self, uInt):
        self._value = uInt
        for key, bit in self._bitFields.iteritems():
            setattr(self, key, self._value & 2**bit > 0)


class _StageConfig(_IntParser):
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
    _valueTypes = [(0, 'heater1Temp', '°C', 'r',),
                   (1, 'heater1Rate', '°C/min', 'rw'),
                   (2, 'heater1Limit', '°C', 'rw'),
                   (3, 'heater1Power', '%', 'r'),
                   (4, 'heater1LNPSpeed', '%', 'r'),
                   (5, 'heater2Temp', '°C', 'r'),
                   (10, 'coolingWaterTemp', '°C', 'r'),
                   (12, 'vacuum', 'mbar', 'r'),
                   (13, 'vacuumLimit', 'mbar', 'rw'),
                   (14, 'humidity', '%rh', 'r'),
                   (15, 'humidityLimit', '%rh', 'rw'),
                   (16, 'xMotorPosition', 'µm', 'r'),
                   (17, 'xMotorVelocity', 'µm/s', 'rw'),
                   (18, 'xMotorLimit', 'µm', 'rw'),
                   (19, 'yMotorPosition', 'µm', 'r'),
                   (20, 'yMotorVelocity', 'µm/s', 'rw'),
                   (21, 'yMotorLimit', 'µm', 'rw'),
                   (25, 'tensileMotorPosition', 'µm', 'r'),
                   (26, 'tensileMotorVelocity', 'µm/s', 'rw'),
                   (27, 'tensileMotorLimit', 'µm', 'rw'),
                   (28, 'MV196 motorised valve position', 'µm', 'r'),
                   (29, 'MV196 motorised valve velocity', 'µm/s', 'rw'),
                   (30, 'MV196 motorised valve limit', 'µm', 'rw'),
                   (31, 'Graded stage valve position', 'µm', 'r'),
                   (32, 'Graded stage motor velocity', 'µm/s', 'rw'),
                   (33, 'Graded stage motor limit', 'µm', 'r'),
                   (34, 'SampleRefNeg', '°C', 'rw'),
                   (35, 'SampleActNeg', '°C', 'rw'),
                   (36, 'SampleRefZero', '°C', 'rw'),
                   (37, 'SampleActZero', '°C', 'rw'),
                   (38, 'SampleRefPos', '°C', 'rw'),
                   (39, 'SampleActPos', '°C', 'rw'),
                   (50, 'Heater 3 temperature', '°C', 'r'), # CMS196 Correlative Dewar temperature
                   (51, 'Heater 4 temperature', '°C', 'r'),
                   (52, 'correlativeLight', '', 'rw'),
                   (53, 'correlativeWarmingHeater', '', 'rw'),
                   (54, 'correlativeRefillSolenoid', '', 'rw'),
                   (55, 'correlativeSampleDewarFilling', '', 'r'), # 1=filling
                   (56, 'correlativeStatus', '', 'r'),
                   (57, 'correlativeErrors', '', 'r'),
                   (58, 'ramp1HoldTime', 's', 'rw'),
                   (59, 'ramp1HoldTimeLeft', 's', 'r'),
                   (60, 'correlativeMainDewarFilling', '', 'r'), # 1=filling
                   (61, 'correlativeCondenserLED', '%', 'rw'), # Only 0 and 100 valid: 0=off, 100=on
                   (62, 'correlativeTestMotion', '', 'rw'), 
                   (63, 'xyMotorFeedbackMode', '', 'rw')]


    nameToIndex = {name:index for (index, name, _, _) in _valueTypes}
    nameToUnit = {name:unit for (_, name, unit, _) in _valueTypes}
    nameToAccess = {name:access for (_, name, _, access) in _valueTypes}

    def __init__(self):
        self.object = LinkamCommsDll.Comms()
        self.stageConfig = _StageConfig()
        self.status = _StageStatus()
        

    def connect(self):
        result = self.object.OpenComms(True, 0, 0)
        connected = result & 0x0001
        if connected:
            self.stageConfig.update(self.object.GetStageConfig())
            self.status.update(self.object.GetStatus())
        return dict(connected = connected,
                    commsNotResponding = result & 0b0010,
                    commsFailedToSendConfigData = result & 0b0100,
                    commsSerialPortError = result & 0b1000)


    def getConfig(self):
        configWord = self.object.GetStageConfig()
        self.stageConfig.update(configWord)
        return self.stageConfig


    def getStatus(self):
        statusWord = self.object.GetStatus()
        self.status.update(statusWord)
        return self.status


    def getValueByName(self, name):
        index = self.nameToIndex.get(name)
        if index is None:
            return None
        return self.object.GetValue(index)


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