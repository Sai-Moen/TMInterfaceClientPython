# sd_railgun; sd script by SaiMoen

import numpy as np
from tminterface.client import Client, run_client
from tminterface.interface import TMInterface
from tminterface.structs import SimStateData

from struct import unpack
from tminterface.constants import SIMULATION_WHEELS_SIZE

USE_DECIMAL_NOTATION = False # set to True for decimal notation, False for milliseconds

class Railgun(Client):
    """Main Client Implementation."""
    def __init__(self):
        self.cfg = {
            "time_from" : -1,
            "time_to" : -1,
            "direction" : 0,
            "sdmode" : default(self),
            "use_decimal" : USE_DECIMAL_NOTATION
        }
        self.load_cfg()
        self.set_time_format()

        self.input_time = -1
        self.schedule = []
        self.inputs = []
        self.algo = SteerAlgo()
        self.state = RgState()

    def load_cfg(self):
        self.time_from: int = self.cfg["time_from"]
        self.time_to: int = self.cfg["time_to"]
        self.direction: int = self.cfg["direction"]
        self.sdmode: default = self.cfg["sdmode"]
        self.use_decimal: bool = self.cfg["use_decimal"]

    def set_time_format(self):
        self.generateCmdTime = self.generateDecimal if self.use_decimal else lambda tick: self.time_from + tick * 10

    def on_registered(self, iface: TMInterface):
        print(f"[Railgun] Registered to {iface.server_name}")
        iface.execute_command("set controller none")
        iface.register_custom_command("sd")
        iface.log("[Railgun] Use the sd command to set a time range and direction: time_from-time_to sd direction")
        iface.register_custom_command("sdmode")
        iface.log("[Railgun] Use the sdmode command to switch between: default, s4d, s4dirt, wiggle or wiggledirt.")

    def on_custom_command(self, iface: TMInterface, time_from: int, time_to: int, command: str, args: list):
        if command == "sd":
            msg = self.on_sd(time_from, time_to, args)
        elif command == "sdmode":
            msg = self.on_sdmode(args)
        else:
            return
        iface.log(*msg)

    def on_sd(self, time_from: int, time_to: int, args: list):
        if time_to == -1 or time_from == -1:
            return "[Railgun] Timerange not set, Usage: time_from-time_to sd direction", "warning"
        elif len(args) == 1 and args[0] in ("left", "right"):
            self.cfg["direction"] = int(args[0] == "right") - int(args[0] == "left")
            self.cfg["time_from"], self.cfg["time_to"] = time_from, time_to
            return "[Railgun] sd settings changed successfully!", "success"
        return "[Railgun] Usage: time_from-time_to sd direction", "warning"

    def on_sdmode(self, args: list):
        if not args:
            self.cfg["sdmode"] = default(self)
            return "[Railgun] sdmode reset to default", "success"
        elif len(args) == 1:
            mode = args[0]
            if mode == "default":
                self.cfg["sdmode"] = default(self)
            elif mode == "s4d":
                self.cfg["sdmode"] = s4d(self)
            elif mode == "s4dirt":
                self.cfg["sdmode"] = s4dirt(self)
            elif mode == "wiggle":
                self.cfg["sdmode"] = wiggle(self)
            elif mode == "wiggledirt":
                self.cfg["sdmode"] = wiggledirt(self)
            else:
                return "[Railgun] Invalid mode", "warning"
            return f"[Railgun] sdmode set to {args[0]}", "success"
        return "[Railgun] Usage: sdmode <default, s4d, s4dirt, wiggle or wiggledirt>", "warning"

    def on_simulation_begin(self, iface: TMInterface):
        iface.remove_state_validation()
        self.load_cfg()
        if not self.direction:
            iface.log("[Railgun] Usage: time_from-time_to sd direction", "error")
            iface.close()
            print("[Railgun] Closed due to exception, use the sd command to change timerange and direction.")
            return
        print(f"[Railgun] Simulating from {self.time_from} to {self.time_to} trying to sd {self.direction}")
        self.input_time = self.time_from

        self.resetSeek()
        self.schedule.clear()
        self.csteering = False
        self.algo.setupNewTick(self.direction)

        self.time_limit = self.time_to + self.seek + 10
        iface.set_simulation_time_limit(self.time_limit - 10010) # temporary manual offset

    def on_simulation_step(self, iface: TMInterface, _time: int):
        self.state.update(iface)
        if _time >= self.time_limit:
            return
        elif _time == self.input_time + self.seek:
            self.algo.addData((self.state.sdvel, self.steer))
            iface.rewind_to_state(self.step)
            return
        elif _time == self.input_time:
            self.sdmode.exec()
            if self.sdmode.stepflag:
                self.sdmode.resetStepFlag()
                self.nextStep(iface)
                return
            self.steer = self.algo.getSteer()
            if self.algo.stepflag:
                self.nextStep(iface)
                return
        elif _time == self.input_time - 10:
            self.step = self.state.data

        if _time >= self.input_time:
            iface.set_input_state(sim_clear_buffer=False, steer=self.steer)

    def on_simulation_end(self, *_):
        print("[Railgun] Saving steering inputs to sd_railgun.txt...")
        self.writeSteerToFile()

    def on_deregistered(self, *_):
        print("[Railgun] Attempting to back up most recent inputs to sd_railgun.txt...")
        self.writeSteerToFile()

    def setSeek(self, ms: int):
        self.seek = ms

    def resetSeek(self):
        self.seek = 120

    def getFirstInput(self):
        self.inputs = [self.state.data.input_steer]

    def addToSchedule(self, fn):
        self.schedule.append(fn)

    def nextStep(self, iface: TMInterface):
        best = self.algo.getBest()
        if best * self.direction < 0 and not self.csteering:
            self.changeDirection()
        else:
            iface.set_input_state(sim_clear_buffer=False, steer=best)
            self.inputs.append(best)
            print(f"{self.input_time} steer {best} -> {self.state.speed} km/h")
            if self.csteering:
                self.changeDirection()
            self.input_time += 10
        self.algo.setupNewTick(self.direction)
        iface.rewind_to_state(self.step)

    def changeDirection(self):
        self.direction *= -1
        self.csteering ^= True

    def writeSteerToFile(self):
        msg = "success!"
        try:
            with open("sd_railgun.txt", 'w') as f:
                f.writelines(
                    [
                        f"{self.generateCmdTime(t[0])} steer {t[1]}\n" for t in
                        enumerate(self.inputs[1:]) if t[1] != self.inputs[t[0]]
                    ]
                )
        except:
            msg = "failed."
        finally:
            print(f"[Railgun] Input write {msg}")

    def generateDecimal(self, tick: int):
        t = self.time_from // 10 + tick
        h, m, s, c = t//360000, t//6000%60, t//100%60, t%100

        c = f"{c / 100}".removeprefix("0")
        s = f"{s}"
        m = f"{m}:" if h or m else ""
        h = f"{h}:" if h else ""

        return h + m + s + c

    @staticmethod
    def getServerName():
        try:
            server_id = int(
                input("[Railgun] Enter the TMInterface instance ID you would like to connect to...\n")
            )
            server_id *= (server_id > 0)
        except:
            server_id = 0
        finally:
            return f"TMInterface{server_id}"

    def main(self):
        server_name = self.getServerName()
        print(f"[Railgun] Connecting to {server_name}...")
        run_client(self, server_name)
        print(f"[Railgun] Deregistered from {server_name}")

class SteerAlgo:
    """Steering algorithm implementation for finding sd steering values"""
    def __init__(self):
        self.srange: tuple[int, int] = (0, 0)
        self.steerGen = None
        self.data: list[tuple] = []
        self.best: tuple[np.floating, int] = (0, 0)
        self.stepflag: bool = False

    def setupNewTick(self, direction: int):
        self.data = [(0, 0x8000 * direction)]
        self.setSteerRange(0x8000) 
        self.steerGen = self.getSteerGen()
        self.stepflag = False

    def addData(self, data: tuple):
        self.data.append(data)

    def setSteerRange(self, step: int):
        best = self.getBest()
        self.srange = (best - step, best + step)

    def getBest(self):
        self.data.sort(reverse=True)
        self.best = self.data[0]
        return self.best[1]

    def getSteerGen(self):
        mins, maxs = min(self.srange), max(self.srange)
        step = (maxs - mins) >> 0x3
        if stepflag := step < 0x4:
            for s in range(mins, maxs + 1):
                if s != self.best[1] and abs(s) <= 0x10000:
                    yield s
        else:
            for s in (mins + step, mins + 3 * step, maxs - 3 * step, maxs - step):
                if abs(s) <= 0x10000:
                    yield s
        self.setSteerRange(step << 0x1) # this bit-shift goes hard
        self.data = [self.best]
        self.stepflag = stepflag

    def getSteer(self):
        if (steer := next(self.steerGen, None)) != None:
            return steer
        elif not self.stepflag:
            self.steerGen = self.getSteerGen()
            return next(self.steerGen) # except if stepflag fails
        return 0

class RgState:
    """Modified version of SimStateData that automatically calculates relevant values for this script."""
    def __init__(self):
        self.data = SimStateData()
        self.velocity = np.float64(0)
        self.speed = np.float64(0)

        self.wheels = [0, 0, 0, 0]
        self.setLocalVelocity()
        self.sdvel = np.float64(0)

    def update(self, iface: TMInterface):
        self.data = iface.get_simulation_state()
        self.velocity = np.linalg.norm(self.data.velocity)
        self.speed = self.velocity * 3.6

        self.wheels = self.getWheelContact()
        self.setLocalVelocity()
        self.sdvel = self.getEvalVelocity()

    def setLocalVelocity(self):
        self.lvx, self.lvy, self.lvz = [self.getLocalVelocity(i) for i in range(3)]

    def getLocalVelocity(self, idx: int):
        return np.sum(
            [self.data.velocity[i] * self.data.rotation_matrix[i][idx] for i in range(3)]
        )

    def getEvalVelocity(self):
        return np.linalg.norm(
            (
                self.lvx,
                self.lvy * (not all(self.wheels)),
                self.lvz
            )
        )

    wheels_size = tuple([(SIMULATION_WHEELS_SIZE >> 2) * i for i in range(4)])
    def getWheelContact(self):
        return [
            unpack('i', self.data.simulation_wheels[i+292:i+296])[0]
            for i in self.wheels_size
        ]

class default:
    """Base class for all sdmode implementations."""
    def __init__(self, rg: Railgun):
        self.rg = rg
        self.reset()

    def reset(self):
        self.schedule = [self.rg.getFirstInput]
        self.resetStepFlag()

    def setStepFlag(self):
        self.stepflag = True

    def resetStepFlag(self):
        self.stepflag = False

    def exec(self):
        schedule = self.schedule.copy()
        self.schedule.clear()
        for fn in schedule:
            fn()

class s4d(default):
    """s4d mode for road"""
    def reset(self):
        self.schedule = [self.rg.getFirstInput, self.checks4d]
        self.resetStepFlag()

    def isUndersteering(self):
        return self.rg.state.lvx * self.rg.direction < self.minLvx()

    def minLvx(self):
        return 4 - 0.5 * ((self.rg.state.speed > 401) * ((self.rg.state.speed - 401) / 100))

    def checks4d(self):
        if self.isUndersteering():
            self.schedule.append(self.checks4d)
            self.setStepFlag()
            return
        self.rg.setSeek(130)
        self.seek_reset_time = self.rg.input_time + 60
        self.schedule.append(self.resets4d)

    def resets4d(self):
        if self.seek_reset_time == self.rg.input_time:
            self.rg.resetSeek()
        else:
            self.schedule.append(self.resets4d)

class s4dirt(s4d):
    """s4d mode for dirt"""
    def minLvx(self):
        return 0.25 + (self.rg.state.speed > 235) * (306 - self.rg.state.speed) / 1000

class wiggle(default):
    """wiggle mode for grass"""
    def reset(self):
        self.schedule = [self.rg.getFirstInput, self.checkWiggle]
        self.resetWiggleTimer()

    def resetWiggleTimer(self):
        self.wiggle_time = self.rg.input_time + 300

    def checkWiggle(self):
        if self.rg.state.velocity < np.linalg.norm(self.rg.step.velocity):
            self.resetWiggleTimer()
        elif self.wiggle_time <= self.rg.input_time:
            self.ifWiggleTimerDepleted()
        self.schedule.append(self.checkWiggle)

    def ifWiggleTimerDepleted(self):
        self.resetWiggleTimer()
        self.rg.addToSchedule(self.rg.changeDirection)

class wiggledirt(wiggle, s4dirt):
    """wiggle mode for dirt"""
    def reset(self):
        super().reset()
        self.resetStepFlag()

    def ifWiggleTimerDepleted(self):
        super().ifWiggleTimerDepleted()
        self.schedule.append(self.checks4d)
        self.setStepFlag()

if __name__ == "__main__":
    Railgun().main()
