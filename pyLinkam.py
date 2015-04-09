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
    _valueTypes = [(0,  'Heater1TempR', '°C', 'r',),
                   (1,  'Heater1RateRW', '°C/min', 'rw'),
                   (2,  'Heater1LimitRW', '°C', 'rw'),
                   (3,  'Heater1Power', '%', 'r'),
                   (4,  'Heater1LnpSpeed', '%', 'r'),
                   (5,  'Heater2TempR', '°C', 'r'),
                   (6,  'Heater2RateRW', '°C/min', 'rw'),
                   (7,  'Heater2LimitRW', '°C', 'rw'),
                   (8,  'Heater2Power', '%', 'r'),
                   (9,  'Heater2LnpSpeed', '%', 'r'),                   
                   (10, 'CoolingWaterTempR', '°C', 'r'),
                   (12, 'VacuumR', 'mbar', 'r'),
                   (13, 'VacuumLimitRW', 'mbar', 'rw'),
                   (14, 'HumidityR', '%rh', 'r'),
                   (15, 'HumidityLimitRW', '%rh', 'rw'),
                   (16, 'XMotorPosnR', 'µm', 'r'),
                   (17, 'XMotorVelRW', 'µm/s', 'rw'),
                   (18, 'XMotorLimitRW', 'µm', 'rw'),
                   (19, 'YMotorPosnR', 'µm', 'r'),
                   (20, 'YMotorVelRW', 'µm/s', 'rw'),
                   (21, 'YMotorLimitRW', 'µm', 'rw'),
                   (22, 'ZMotorPosnR', 'µm', 'r'),
                   (23, 'ZMotorVelRW', 'µm/s', 'rw'),
                   (24, 'ZMotorLimitRW', 'µm', 'rw'),
                   (25, 'TstMotorPosnR', 'µm', 'r'),
                   (26, 'TstMotorVelRW', 'µm/s', 'rw'),
                   (27, 'TstMotorLimitRW', 'µm', 'rw'),
                   (28, 'VacMotorPosnR', 'µm', 'r'),
                   (29, 'VacMotorVelRW', 'µm/s', 'rw'),
                   (30, 'VacMotorLimitRW', 'µm', 'rw'),
                   (31, 'GsMotorPosnR', 'µm', 'r'),
                   (32, 'GsMotorVelRW', 'µm/s', 'rw'),
                   (33, 'GsMotorLimitRW', 'µm', 'r'),
                   (34, 'SampleRefNegRW', '°C', 'rw'),
                   (35, 'SampleActNegRW', '°C', 'rw'),
                   (36, 'SampleRefZeroRW', '°C', 'rw'),
                   (37, 'SampleActZeroRW', '°C', 'rw'),
                   (38, 'SampleRefPosRW', '°C', 'rw'),
                   (39, 'SampleActPosRW', '°C', 'rw'),
                   (50, 'Heater3TempR', '°C', 'r'), # CMS196 Correlative Dewar temperature
                   (51, 'Heater4TempR', '°C', 'r'),
                   (52, 'CMS196Light', '', 'rw'),
                   (53, 'CMS196Heater', '', 'rw'),
                   (54, 'CMS196Solenoid', '', 'rw'),
                   (55, 'CMS196SampleDewarFillSignal', '', 'r'), # 1=filling
                   (56, 'CMS196Status', '', 'r'),
                   (57, 'CMS196Errors', '', 'r'),
                   (58, 'HoldTimeRW', 's', 'rw'),
                   (59, 'HoldTimeLeftR', 's', 'r'),
                   (60, 'CMS196MainDewarFillSignal', '', 'r'), # 1=filling
                   (61, 'CMS196CondensorLedLevel', '%', 'rw'), # Only 0 and 100 valid: 0=off, 100=on
                   (62, 'CMS196TestMotion', '', 'rw'), 
                   (63, 'XYMotorFeedbackMode', '', 'rw')]


    nameToIndex = {name:index for (index, name, _, _) in _valueTypes}
    nameToUnit = {name:unit for (_, name, unit, _) in _valueTypes}
    nameToAccess = {name:access for (_, name, _, access) in _valueTypes}

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