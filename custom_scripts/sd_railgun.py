# sd_railgun; sd script by SaiMoen

from tminterface.client import Client, run_client
from tminterface.interface import TMInterface
from tminterface.structs import SimStateData

from numpy.linalg import norm

USE_DECIMAL_NOTATION = False # set to True for decimal notation, False for milliseconds
USE_INFO = True # set to True to get info (like speed), False to get easier to copy output (if writing file fails)

# You shouldn't need to change anything except for the previous line (if you want decimal notation of course)

FULLSTEER = 0x10000
HALF_STEER = 0x8000
TICK_MS = 10
DEFAULT_SEEK = 12 * TICK_MS

TAG = "[Railgun] "
def getServerName():
    msg = "Enter the TMInterface instance ID you would like to connect to... (number after server name)\n"
    try:
        server_id = int(input(TAG + msg))
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
            "seek": DEFAULT_SEEK,
            "use_decimal": USE_DECIMAL_NOTATION
        }
        self.load_cfg()

        self.input_time = -1
        self.inputs = []
        self.state = RgState()
        self.algo = Steerer()

    def load_cfg(self):
        self.time_from: int = self.cfg["time_from"]
        self.time_to: int = self.cfg["time_to"]
        self.direction: int = self.cfg["direction"]
        self.seek: int = self.cfg["seek"]
        self.generateCmdTime = self.generateDecimal if self.cfg["use_decimal"] else self.generateMs

    def on_registered(self, iface: TMInterface):
        print(TAG + f"Registered to {iface.server_name}")
        iface.execute_command("set controller none")
        iface.register_custom_command("sd")
        iface.log(TAG + "Use the sd command to set a time range and direction: time_from-time_to sd direction")
        iface.register_custom_command("seek")
        iface.log(TAG + "Use the seek command to manipulate the seeking timerange in tick amount (default: 12).")
        iface.log(TAG + "Setting it lower may slightly improve gains at high sd quality, but it's less stable.")

    def on_custom_command(self, iface: TMInterface, time_from: int, time_to: int, command: str, args: list):
        msg = ("Invalid command", "error")
        if command == "sd":
            msg = self.on_sd(time_from, time_to, args)
        elif command == "seek":
            msg = self.on_seek(args)
        iface.log(TAG + msg[0], msg[1])

    def on_sd(self, time_from: int, time_to: int, args: list):
        if time_to == -1 or time_from == -1:
            return "Timerange not set, Usage: time_from-time_to sd direction", "warning"
        elif len(args) == 1 and args[0] in ("left", "right"):
            self.cfg["direction"] = int(args[0] == "right") - int(args[0] == "left")
            self.cfg["time_from"], self.cfg["time_to"] = time_from, time_to
            return "sd settings changed successfully!", "success"
        return "Direction not set, Usage: time_from-time_to sd direction", "warning"

    def on_seek(self, args: list):
        if not args:
            self.cfg["seek"] = DEFAULT_SEEK
            return f"seek value reset to {DEFAULT_SEEK}ms successfully!", "success"
        try:
            new_seek = int(args[0])
            if new_seek >= 1:
                new_seek *= TICK_MS
                self.cfg["seek"] = new_seek
                return f"seek value changed to {new_seek}ms successfully!", "success"
            return "seek value should be at least 1 tick", "warning"
        except:
            return "Not a parseable number", "warning"

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
        print(f"seek: {self.seek}")
        self.input_time = self.time_from

        self.inputs: list = [None]
        self.csteering = False
        self.algo.setupNewTick(self.direction)

        self.time_limit = self.time_to + self.seek + TICK_MS
        iface.set_simulation_time_limit(self.time_limit - 10010) # temporary manual offset

    def on_simulation_step(self, iface: TMInterface, _time: int):
        time_min = self.input_time - TICK_MS
        is_in_range = time_min <= _time < self.time_limit
        if not is_in_range:
            return
        
        if _time == self.input_time + self.seek:
            self.onVelocityCheck(iface)
            return
        elif _time == self.input_time:
            if self.onInputTime(iface):
                return
        elif _time == time_min:
            self.step = iface.get_simulation_state()

        if _time >= self.input_time:
            iface.set_input_state(sim_clear_buffer=False, steer=self.steer)

    def on_simulation_end(self, *_):
        print(TAG + "Saving steering inputs to sd_railgun.txt...")
        self.writeSteerToFile()

    def on_deregistered(self, *_):
        print(TAG + "Attempting to back up most recent inputs to sd_railgun.txt...")
        self.writeSteerToFile()

    def onVelocityCheck(self, iface: TMInterface):
        self.state.update(iface)
        self.algo.addData((self.state.sd_vel, self.steer))
        iface.rewind_to_state(self.step)

    def onInputTime(self, iface: TMInterface):
        self.state.update(iface)
        self.steer = self.algo.getSteer()
        if self.algo.isDone:
            self.nextStep(iface)
            return True
        return False

    def nextStep(self, iface: TMInterface):
        """Re-do the current tick while countersteering or go to the next tick."""
        best = self.algo.getBest()
        if best * self.direction < 0 and not self.csteering:
            self.changeDirection()
        else:
            iface.set_input_state(sim_clear_buffer=False, steer=best)
            self.inputs.append(best)
            self.printInfo(best)
            
            if self.csteering:
                self.changeDirection()
            self.input_time += TICK_MS
        self.algo.setupNewTick(self.direction)
        iface.rewind_to_state(self.step)

    def changeDirection(self):
        self.direction *= -1
        self.csteering ^= True

    def printInfo(self, best: int):
        info = USE_INFO * f" -> {self.state.speed} km/h"
        print(f"{self.input_time} steer {best}{info}")

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
        self.speed = 0

        self.not_all_wheels = False
        self.local_vel = [0.0, 0.0, 0.0]
        self.sd_vel = 0

    def update(self, iface: TMInterface):
        self.data = iface.get_simulation_state()
        self.speed = norm(self.data.velocity) * 3.6

        self.not_all_wheels = not all(
            [
                w.real_time_state.has_ground_contact
                for w in self.data.simulation_wheels
            ]
        )
        self.local_vel = self.data.scene_mobil.current_local_speed
        self.local_vel[1] *= self.not_all_wheels
        self.sd_vel = norm(self.local_vel)

class Steerer:
    """Steering algorithm implementation for finding sd steering values."""
    def __init__(self):
        self.data = []
        self.srange = (0, 0)
        self.best = (0.0, 0)
        self.steerGen = None
        self.isDone = False

    def setupNewTick(self, direction: int):
        self.data = [(0, HALF_STEER * direction)]
        self.setSteerRange(HALF_STEER) 
        self.steerGen = self.getSteerGen()
        self.isDone = False

    def addData(self, data: tuple):
        self.data.append(data)

    def setSteerRange(self, step: int):
        best = self.getBest()
        self.srange = (best - step, best + step)

    def getBest(self):
        self.data.sort(reverse=True)
        self.best = self.data[0]
        return self.best[1]

    def getSteerGen(self): # Probably just rewriting this for new api
        mins, maxs = min(self.srange), max(self.srange)
        done = (step := (maxs - mins) >> 3) < 4
        step = (offset := step * (not done)) << 1
        for s in range(mins + offset, maxs - offset + 1, step + done):
            if s != self.best[1] and abs(s) <= FULLSTEER:
                yield s
        self.setSteerRange(step)
        self.data = [self.best]
        self.isDone = done

    def getSteer(self):
        if (steer := next(self.steerGen, None)) != None:
            return steer
        elif not self.isDone:
            self.steerGen = self.getSteerGen()
            return next(self.steerGen)
        return 0

if __name__ == "__main__":
    Railgun().main()
