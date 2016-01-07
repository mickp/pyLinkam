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

Note that return values are cast as native python types to
enable remote calls over Pyro from python instances without
support for .NET.
"""

DEFAULT_ERRORTHRESHOLD = 2.5 # microns
DEFAULT_HUNTINGTHRESHOLD = 0.1 # microns
DEFAULT_KICKSTEP = 5 # microns
DEFAULT_SETTLINGTIME = 10 # ms

import clr
import ctypes
import distutils.version
from operator import sub
import os
import random
import signal
import sys
import threading
import time
import Pyro4
import System

if (distutils.version.LooseVersion(Pyro4.__version__) >=
    distutils.version.LooseVersion('4.22')):
    Pyro4.config.SERIALIZERS_ACCEPTED.discard('serpent')
    Pyro4.config.SERIALIZERS_ACCEPTED.add('pickle')
Pyro4.config.SERIALIZER = 'pickle'

# Cockpit config name
CONFIG_NAME = 'linkam'

# DLL details
DLL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'linkam')
DLL_FILE = r"LinkamCommsLibrary.dll"
DLL_VER = distutils.version.LooseVersion(
                System.Diagnostics.FileVersionInfo.GetVersionInfo(
                        os.path.join(DLL_PATH, DLL_FILE)).FileVersion)

## Add the DLL to the path and clr, then import LinkamCommsDll.
sys.path.append(DLL_PATH)
clr.AddReference(r"LinkamCommsLibrary")
import Linkam
import LinkamCommsDll

## Enumerated value types for SetValue and GetValue. Linkam moved the the enum
# def between versions 1.8.2.0 and 1.8.5.0.
if DLL_VER >= distutils.version.LooseVersion('1.8.5.0'):
    eVALUETYPE = Linkam.SharedEnums.eVALUETYPE
else:
    # eVALUETYPE = LinkamCommsDll.Comms.eVALUETYPE
    raise Exception('%s requires Linkam DLL version >= 1.8.5.0')


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
        self._value = long(uInt)
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
    XMOTOR_BIT = 2**45
    YMOTOR_BIT = 2**48


    def __del__(self):
        self._run_flag = False
        self.statusThread.join()
        self.motionThread.join()


    def __init__(self):
        # Comms link to the hardware.
        self.stage = LinkamCommsDll.Comms()
        # Reported stage configuration.
        self.stageConfig = _StageConfig()
        # Stage status.
        self.status = _StageStatus()
        # A flag to show if the stage has been connected.
        self.connected = False
        # A flag to show that the motors have been homed.
        self.motorsHomed = False
        # A handler to detect stage disconnection events.
        self.stage.ControllerDisconnected += self._disconnectEventHandler
        # A handler to detect stage connection events.
        self.stage.ControllerConnected += self._connectEventHandler
        # Motion control parameters
        self.controlParameters = dict(
            errorThreshold = DEFAULT_ERRORTHRESHOLD, # microns
            huntingThreshold = DEFAULT_HUNTINGTHRESHOLD, # microns
            kickStep = DEFAULT_KICKSTEP, # microns
            settlingTime = DEFAULT_SETTLINGTIME) # ms

        # A thread to update status.
        self.statusThread = threading.Thread(target=self._updateStatus,
                                             name='StatusThread')
        self.statusThread.Daemon = True
        # A thread to correct for motion issues.
        self.motionThread = threading.Thread(target=self._correctMotion,
                                             name='MotionThread')
        self.motionThread.Daemon = True
        # A lock to block on state and status manipulations.
        self.lock = threading.RLock()
        # Flag to indicate movement status
        self.moving = None
        # Stage position target.
        self.targetPos = [None, None]
        # Current stage position
        self.position = (None, None)
        # Flag to indicate stop motors after move
        self.stopMotorsBetweenMoves = True
        # Client to send status updates to
        self.client = None
        # Run flag.
        self._run_flag = True
        # Start the threads *after* all properties intialized.
        self.statusThread.start()
        self.motionThread.start()


    def _connectEventHandler(self, sender, eventArgs):
        """Handles stage connection events."""
        self.connected = True
        oldPosition = self.position
        if self.position == (0.0, 0.0):
            # Stage may have just been powered on.
            self.motorsHomed = False
        else:
            self.motorsHomed = self.position == oldPosition


    def _disconnectEventHandler(self, sender, eventArgs):
        """Handles stage disconnection events."""
        # Indicate that the stage is no longer connected.
        self.connected = False
        # Indicated that the motors are not homed.
        self.motorsHomed = False


    def _connect(self, reconnect=False):
        if reconnect or not self.connected:
            result = self.stage.OpenComms(True, 0, 0)
        return self.connected


    def _getConfig(self):
        configWord = self.stage.GetStageConfig()
        self.stageConfig.update(configWord)
        return self.stageConfig


    def _updatePosition(self):
        """Fetch and return the stage's current position as (x, y)."""
        ValueIDs = (eVALUETYPE.u32XMotorPosnR,
                    eVALUETYPE.u32YMotorPosnR)
        with self.lock:
            self.position = tuple((float(self.stage.GetValue(id)) for id in ValueIDs))
        return self.position


    def getPosition(self):
        """Return the stage's current position as (x, y)."""
        if None in self.position:
            # Client may not handle None
            return (-1, -1)
        else:
            return self.position


    def getStatus(self):
        statusWord = self.stage.GetStatus()
        self.status.update(statusWord)
        return self.status


    def homeMotors(self):
        """Home the motors.
        From CMS196.exe CMS196.frmMotorControl.btnIndex_Click.
        """
        self.moveToXY(-12000., -4000.)
        # Set this here, but would be better to wait until the
        # homing operation has finished by, say, watching the
        # stage position until it reaches (0, 0).
        self.motorsHomed = True


    def _moveToXY(self, x=None, y=None):
        """Move the stage motors - private version.

        Moves the motors without requiring self.lock or updating
        this instance's targetPos.
        """
        self.moving = True
        xValueID = eVALUETYPE.u32XMotorLimitRW
        yValueID = eVALUETYPE.u32YMotorLimitRW
        if x:
            self.stage.SetValue(xValueID, x)
            self.stage.StartMotors(True, 0)
        if y:
            self.stage.SetValue(yValueID, y)
            self.stage.StartMotors(True, 1)



    def moveToXY(self, x=None, y=None):
        """Move stage motors to position (x, y)

        If either x or y is None, the position on that axis is unchanged.
        Return as soon as motion is started to avoid timeouts when called
        remotely. Use isMoving() to check movement status.
        """
        # Grab self.lock and hold on to it to stop updateStatus 
        # clearing the move flag before we have started moving the 
        # stage.
        with self.lock:
            if x:
                self.targetPos[0] = x
            if y:
                self.targetPos[1] = y
            self._moveToXY(*self.targetPos)


    def isMoving(self):
        """Return whether or not the stage is moving.
    
        This now uses an instance variable instead of testing
        the hardware directly - that required multiple GetStatus
        calls, which lead to either caused the .NET assembly to
        fall over (probably due to too high a call rate) or Pyro
        timeouts (too long between calls).
        """
        with self.lock:
            return self.moving


    def _sendStatus(self, status):
        """Send status to any client."""
        try:
            self.client.receiveData(status)
        except (Pyro4.errors.PyroError):
            # Something happened to the client.
            pass
        except:
            raise


    def setClient(self, uri):
        self.client = self.client = Pyro4.Proxy(uri)


    def setCondensorLedLevel(self, level):
        enum = eVALUETYPE.u32CMS196CondensorLedLevel
        self.stage.SetValue(enum, level)


    def setControlParameters(self, parameters):
        badParams = False
        try:
            n = len(parameters)
        except TypeError:
            badParams = True
        if n != 4:
            badParams = True
        if badParams:
            raise Exception('Control parameters must be a 4-element tuple or list.')
        self.controlParameters = dict(
                errorThreshold = parameters[0],
                huntingThreshold = parameters[1],
                kickStep = parameters[2],
                settlingTime = parameters[3])


    def getControlParameters(self):
        return (self.controlParameters['errorThreshold'],
                self.controlParameters['huntingThreshold'],
                self.controlParameters['kickStep'],
                self.controlParameters['settlingTime'])


    def setMotorSpeed(self, speed):
        self.stage.SetValue(eVALUETYPE.u32XMotorVelRW, speed)
        self.stage.SetValue(eVALUETYPE.u32YMotorVelRW, speed)


    def stopMotors(self):
        for m in [0, 1]:
            self.stage.StartMotors(False, m)


    def toggleChamberLight(self):
        """Toggle the chamber light.

        GetValue(u32CMS196Light) always returns 4, regardless
        of the state of the light.
        Writing any value to u32CMS196Light toggles its state.
        So, we can toggle the state, but not be certain which
        state it is in.
        """
        enum = eVALUETYPE.u32CMS196Light
        self.stage.SetValue(enum, 0)


    def _updateStatus(self):
        """Runs in a separate thread to update status variables."""
        # Delay between iterations
        sleepBetweenIterations = 0.1

        # Map status values to eVALUETYPEs
        statusMap = {'bridgeT':eVALUETYPE.u32Heater1TempR,
                     'chamberT':eVALUETYPE.u32Heater2TempR,
                     'dewarT':eVALUETYPE.u32Heater3TempR,
                     'light':eVALUETYPE.u32CMS196Light,
                     'mainFill':eVALUETYPE.u32CMS196MainDewarFillSignal,
                     'sampleFill':eVALUETYPE.u32CMS196SampleDewarFillSignal,
                     'condensor':eVALUETYPE.u32CMS196CondensorLedLevel,
                     'caseHeater':eVALUETYPE.u32CMS196Heater,}
        # Last time status was sent
        tLastStatus = 0
        # Status update period
        tStatusUpdatePeriod = 1

        while self._run_flag:
            tNow = time.time()
            if not self.connected:
                # Try to connect to stage.
                self._connect()
                # Skip to next iteration.
                if self.client and (tNow - tLastStatus > tStatusUpdatePeriod):
                    tLastStatus = tNow
                    self._sendStatus({'connected':False})
                time.sleep(1)
                continue
            time.sleep(sleepBetweenIterations)
            if self.client and (tNow - tLastStatus > tStatusUpdatePeriod):
                tLastStatus = tNow
                # Must cast results to float for non-.NET clients.
                status = {key: float(self.stage.GetValue(enum))
                          for key, enum in statusMap.iteritems()}
                status['time'] = tNow
                status['connected'] = True
                self._sendStatus(status)


    def _correctMotion(self):
        # Sleep between interations
        sleepBetweenIterations = 0.01
        # Counts required to exit stop-detection loop.
        maxCount = 3
        # Consecutive reads on target
        onTargetCount = 0
        # Consecutive reads showing hunting
        huntingCount = 0
        maxHuntingCount = 10
        # Sliding position average
        slidingMean = None
        # Sliding variance
        slidingVar = None
        # Sliding statistics weighting factor
        alpha = 0.33
        while self._run_flag:
            time.sleep(sleepBetweenIterations)
            if not self.connected:
                # No connection. Will be fixed by StatusThread.
                # Don't hog the CPU.
                time.sleep(1)
                # Skip to next iteration.
                continue
            try:
                with self.lock:
                    pos = self._updatePosition()
                    targetPos = self.targetPos
                    moving = self.moving
            except:
                # There is an issue communicating with the stage.
                # It is not the this thread's job to fix it, so
                # just skip to the next iteration.
                continue

            if moving:
                # Update the sliding statistics.
                if slidingMean is None:
                    slidingMean = [p for p in pos]
                    slidingVar = [m for m in slidingMean]
                else:
                    diff = [p - m for (p, m) in zip(pos, slidingMean)]
                    incr = [alpha * d for d in diff]
                    slidingMean = [m + i for (m, i) in zip(slidingMean, incr)]
                    slidingVar = [(1. - alpha) * (v + d * i)
                                  for(v, d, i) in zip(slidingVar, diff, incr)]

            if moving and not None in targetPos:
                # calculate position error: target - position
                positionError = map(sub, targetPos, pos)
                if all(abs(p) < self.controlParameters['errorThreshold']
                        for p in positionError):
                    # Both axes on target.
                    onTargetCount += 1
                    time.sleep(self.controlParameters['settlingTime'] / 1000)
                else:
                    # Not on target
                    onTargetCount = 0
                    if all(v < self.controlParameters['huntingThreshold']
                            for v in slidingVar):
                        huntingCount += 1
                    else:
                        huntingCount = 0

                if onTargetCount >= maxCount:
                    # Stable at target position.
                    if self.stopMotorsBetweenMoves:
                        self.stopMotors()
                    with self.lock:
                        self.moving = False
                elif huntingCount >= maxHuntingCount:
                    huntingCount = 0
                    # The motor is stuck - give it a kick in a random direction.
                    with self.lock:
                        delta = self.controlParameters['kickStep']
                        self._moveToXY(*[p + random.choice((-delta, delta))
                                       for p in self.targetPos])
                    time.sleep(self.controlParameters['settlingTime'] / 1000)
                    with self.lock:
                        self._moveToXY(*self.targetPos)


class Server(object):
    def __init__(self):
        self.object = None
        self.daemon_thread = None
        self.config = None
        self.run_flag = True


    def __del__(self):
        self.run_flag = False
        if self.daemon_thread:
            self.daemon_thread.join()


    def run(self):
        import readconfig
        config = readconfig.config

        Pyro4.config.COMMTIMEOUT = 5.0

        host = config.get(CONFIG_NAME, 'ipAddress')
        port = config.getint(CONFIG_NAME, 'port')

        self.object = LinkamStage()

        daemon = Pyro4.Daemon(port=port, host=host)

        # Start the daemon in a new thread.
        self.daemon_thread = threading.Thread(
            target=Pyro4.Daemon.serveSimple,
            args = ({self.object: CONFIG_NAME},),
            kwargs = {'daemon': daemon, 'ns': False}
            )
        self.daemon_thread.start()

        # Wait until run_flag is set to False.
        while self.run_flag:
            time.sleep(1)

        # Do any cleanup.
        daemon.shutdown()
        self.daemon_thread.join()

        # There seems to be a refcounting problem in ironpython.
        # If I delete self.object, then there should be no more references
        # to it and it's __del__ method should be called, which should
        # terminate the statusthread. That does not happen and, since
        # ironpython doesn't implement getrefcount, I can't chase it down.
        # Instead, we must call the __del__ method explicitly.
        # self.object = None
        self.object.__del__()


    def stop(self):
        self.run_flag = False


def main():
    s = Server()
    # Make SIGINT stop the server.
    signal.signal(signal.SIGINT, lambda signal, frame: s.stop())
    s.run()


if __name__ == '__main__':
    main()
    os._exit(0)
