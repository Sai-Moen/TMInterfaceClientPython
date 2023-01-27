# sd_railgun; sd script by SaiMoen

import numpy as np
from tminterface.client import Client, run_client
from tminterface.interface import TMInterface
from tminterface.structs import SimStateData
from tminterface.constants import SIMULATION_WHEELS_SIZE

USE_DECIMAL_NOTATION = False # set to True for decimal notation, False for milliseconds
# You shouldn't need to change anything except for the previous line (if you want decimal notation of course)

FULLSTEER = 0x10000
HALF_STEER = 0x8000
TICK_MS = 10

TAG = "[Railgun] "
def getServerName():
    try:
        server_id = int(
            input(TAG + "Enter the TMInterface instance ID you would like to connect to...\n")
        )
        server_id *= (server_id > 0)
    except:
        server_id = 0
    finally:
        return f"TMInterface{server_id}"

class Railgun(Client):
    """Main Client Implementation."""
    def __init__(self):
        self.cfg = {
            "time_from": -1,
            "time_to": -1,
            "direction": 0,
            "sdmode": default(self),
            "use_decimal": USE_DECIMAL_NOTATION
        }
        self.load_cfg()

        self.input_time = -1
        self.schedule = []
        self.inputs = []
        self.state = RgState()
        self.algo = Steerer()

    def load_cfg(self):
        """
        Loads all fields from the config dict into instance variables
        to avoid live variable tampering (mostly for GUI compatibility).
        """
        self.time_from: int = self.cfg["time_from"]
        self.time_to: int = self.cfg["time_to"]
        self.direction: int = self.cfg["direction"]
        self.sdmode: default = self.cfg["sdmode"]
        self.generateCmdTime = self.generateDecimal if self.cfg["use_decimal"] else self.generateMs

    def on_registered(self, iface: TMInterface):
        print(TAG + f"Registered to {iface.server_name}")
        iface.execute_command("set controller none")
        iface.register_custom_command("sd")
        iface.log(TAG + "Use the sd command to set a time range and direction: time_from-time_to sd direction")
        iface.register_custom_command("sdmode")
        iface.log(TAG + "Use the sdmode command to switch between: default, s4d, s4dirt, wiggle or wiggledirt.")

    def on_custom_command(self, iface: TMInterface, time_from: int, time_to: int, command: str, args: list):
        if command == "sd":
            msg = self.on_sd(time_from, time_to, args)
        elif command == "sdmode":
            msg = self.on_sdmode(args)
        else:
            iface.log(TAG + "Invalid command", "error")
            return
        iface.log(TAG + msg[0], msg[1])

    def on_sd(self, time_from: int, time_to: int, args: list):
        if time_to == -1 or time_from == -1:
            return "Timerange not set, Usage: time_from-time_to sd direction", "warning"
        elif len(args) == 1 and args[0] in ("left", "right"):
            self.cfg["direction"] = int(args[0] == "right") - int(args[0] == "left")
            self.cfg["time_from"], self.cfg["time_to"] = time_from, time_to
            return "sd settings changed successfully!", "success"
        return "Usage: time_from-time_to sd direction", "warning"

    def on_sdmode(self, args: list):
        if not args:
            return "Usage: sdmode <default, s4d, s4dirt, wiggle or wiggledirt>", "log"
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
                return "Invalid mode", "warning"
            return f"sdmode set to {mode}", "success"
        return "Usage: sdmode <default, s4d, s4dirt, wiggle or wiggledirt>", "warning"

    def on_simulation_begin(self, iface: TMInterface):
        iface.remove_state_validation()
        self.load_cfg()
        if not self.direction:
            iface.log(TAG + "Usage: time_from-time_to sd direction", "error")
            iface.close()
            print(TAG + "Closed due to exception, use the sd command to change timerange and direction.")
            return
        print(TAG + "Simulation variables:")
        print(f"time_from: {self.time_from}")
        print(f"time_to: {self.time_to}")
        print(f"direction: {self.direction}")
        print(f"sdmode: {type(self.sdmode)}")
        self.input_time = self.time_from

        self.resetSeek()
        self.schedule = [self.setFirstInput]
        self.csteering = False
        self.algo.setupNewTick(self.direction)

        self.time_limit = self.time_to + self.seek + TICK_MS
        iface.set_simulation_time_limit(self.time_limit - 10010) # temporary manual offset

    def on_simulation_step(self, iface: TMInterface, _time: int):
        if _time >= self.time_limit:
            return
        if _time == self.input_time + self.seek:
            self.state.update(iface)
            self.algo.addData((self.state.sd_vel, self.steer))
            iface.rewind_to_state(self.step)
            return
        elif _time == self.input_time:
            self.state.update(iface)
            if self.executeSchedule(iface):
                return
            self.steer = self.algo.getSteer()
            if not self.algo.running:
                self.nextStep(iface)
                return
        elif _time == self.input_time - TICK_MS:
            self.state.update(iface)
            self.step = self.state.data

        if _time >= self.input_time:
            iface.set_input_state(sim_clear_buffer=False, steer=self.steer)

    def executeSchedule(self, iface: TMInterface):
        self.sdmode.executeSchedule()
        schedule = self.schedule.copy()
        self.schedule.clear()
        for fn in schedule:
            fn()
        if self.sdmode.stepflag:
            self.sdmode.resetStepFlag()
            self.nextStep(iface)
        return self.sdmode.stepflag

    def on_simulation_end(self, *_):
        print(TAG + "Saving steering inputs to sd_railgun.txt...")
        self.writeSteerToFile()

    def on_deregistered(self, *_):
        print(TAG + "Attempting to back up most recent inputs to sd_railgun.txt...")
        self.writeSteerToFile()

    def setSeek(self, ms: int):
        self.seek = ms

    def resetSeek(self):
        self.seek = 120

    def setFirstInput(self):
        self.inputs = [self.state.data.input_steer]

    def addToSchedule(self, fn):
        self.schedule.append(fn)

    def nextStep(self, iface: TMInterface):
        """Re-do the current tick while countersteering or go to the next tick."""
        best = self.algo.getBest()
        if best * self.direction < 0 and not self.csteering:
            self.changeDirection()
        else:
            iface.set_input_state(sim_clear_buffer=False, steer=best)
            self.inputs.append(best)
            print(f"{self.input_time} steer {best} -> {self.state.speed} km/h")
            if self.csteering:
                self.changeDirection()
            self.input_time += TICK_MS
        self.algo.setupNewTick(self.direction)
        iface.rewind_to_state(self.step)

    def changeDirection(self):
        self.direction *= -1
        self.csteering ^= True

    def writeSteerToFile(self):
        try:
            with open("sd_railgun.txt", 'w') as f:
                f.writelines(
                    [
                        f"{self.generateCmdTime(t)} steer {s}\n" for t, s in
                        enumerate(self.inputs[1:]) if s != self.inputs[t]
                    ]
                )
        except:
            msg = "failed."
        else:
            msg = "success!"
        finally:
            print(TAG + f"Input write {msg}")
            self.inputs.clear()

    def generateMs(self, tick: int):
        return self.time_from + TICK_MS * tick

    def generateDecimal(self, tick: int):
        t = self.time_from // TICK_MS + tick
        h, m, s, c = t//360000, t//6000%60, t//100%60, t%100

        c = f"{c / 100}".removeprefix("0")
        s = f"{s}"
        m = f"{m}:" if h or m else ""
        h = f"{h}:" if h else ""

        return h + m + s + c

    def main(self, server_name = getServerName()):
        print(TAG + f"Connecting to {server_name}...")
        run_client(self, server_name)
        print(TAG + f"Deregistered from {server_name}")

class RgState:
    """Modified version of SimStateData that automatically calculates relevant values for this script."""
    def __init__(self):
        self.data = SimStateData()
        self.velocity = np.float64(0)
        self.speed = np.float64(0)

        self.not_all_wheels = False
        self.local_vel = np.array([0, 0, 0])
        self.sd_vel = np.float64(0)

    def update(self, iface: TMInterface):
        """Run this at the start of every tick and you won't have to calculate anything in the main client."""
        self.data = iface.get_simulation_state()
        self.velocity = np.linalg.norm(self.data.velocity)
        self.speed = self.velocity * 3.6

        self.not_all_wheels = not self.isFullGroundContact()
        self.local_vel = self.data.scene_mobil.current_local_speed
        self.local_vel[1] *= self.not_all_wheels
        self.sd_vel = np.linalg.norm(self.local_vel)

    def isFullGroundContact(self):
        return all(
            [
                w.real_time_state.has_ground_contact
                for w in self.data.simulation_wheels
            ]
        )

class Steerer:
    """Steering algorithm implementation for finding sd steering values."""
    def __init__(self):
        self.data = []
        self.srange = (0, 0)
        self.best = (0.0, 0)
        self.steerGen = None
        self.running = False

    def setupNewTick(self, direction: int):
        self.data = [(0, HALF_STEER * direction)]
        self.setSteerRange(HALF_STEER) 
        self.steerGen = self.getSteerGen()
        self.running = True

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
        running = (step := (maxs - mins) >> 3) >= 4
        step = (offset := step * running) << 1
        for s in range(mins + offset, maxs - offset + 1, step + (not running)):
            if s != self.best[1] and abs(s) <= FULLSTEER:
                yield s
        self.setSteerRange(step)
        self.data = [self.best]
        self.running = running

    def getSteer(self):
        if (steer := next(self.steerGen, None)) != None:
            return steer
        elif self.running:
            self.steerGen = self.getSteerGen()
            return next(self.steerGen) # except if running flag fails
        return 0 # doesn't matter what is returned, because generator is done running

class default:
    """Base class for all sdmode implementations."""
    def __init__(self, rg: Railgun):
        self.rg = rg
        self.schedule = []
        self.resetStepFlag()

    def reset(self):
        self.schedule.clear()
        self.resetStepFlag()

    def setStepFlag(self):
        self.stepflag = True

    def resetStepFlag(self):
        self.stepflag = False

    def executeSchedule(self):
        schedule = self.schedule.copy()
        self.schedule.clear()
        for fn in schedule: # avoid side effects
            fn()

class s4d(default):
    """s4d mode for road."""
    def reset(self):
        self.schedule = [self.checks4d]
        self.resetStepFlag()

    def isUndersteering(self):
        return self.rg.state.local_vel[0] * self.rg.direction < self.minLvx()

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
    """s4d mode for dirt."""
    def minLvx(self):
        return 0.25 + (self.rg.state.speed > 235) * (306 - self.rg.state.speed) / 1000

class wiggle(default):
    """wiggle mode for grass."""
    def reset(self):
        self.schedule = [self.checkWiggle]
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
    """wiggle mode for dirt."""
    def reset(self):
        super().reset()
        self.resetStepFlag()

    def ifWiggleTimerDepleted(self):
        super().ifWiggleTimerDepleted()
        self.schedule.append(self.checks4d)
        self.setStepFlag()

if __name__ == "__main__":
    Railgun().main()
