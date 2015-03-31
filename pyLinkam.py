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


class StageConfig(object):
    _stageTypes = {2**0: 'standard',
                   2**1: 'high temp',
                   2**2: 'peltier',
                   2**9: 'correlative',}

    def __init__(self, u64):
        self.stageType = self._stageTypes[u64 & 0b1111111111]


class LinkamStage(object):
    def __init__(self):
        self.object = LinkamCommsDll.Comms()


    def connect(self):
        result = self.object.OpenComms(True, 0, 0)
        return dict(connected = result & 0x0001,
                    commsNotResponding = result & 0b0010,
                    commsFailedToSendConfigData = result & 0b0100,
                    commsSerialPortError = result & 0b1000)


    def getConfig(self):
        config = self.object.GetStageConfig()
        return StageConfig(config)
