# wallhugger; wh script by SaiMoen

import numpy as np
from tminterface.client import Client, run_client
from tminterface.interface import TMInterface
from tminterface.structs import SimStateData

USE_DECIMAL_NOTATION = False # set to True for decimal notation, False for milliseconds
# You shouldn't need to change anything except for the previous line (if you want decimal notation of course)

class Wallhugger(Client):
    """Main Client Implementation."""
    def __init__(self):
        self.cfg = {
            "time_from" : -1,
            "time_to" : -1,
            "direction" : 0,
            "use_decimal" : USE_DECIMAL_NOTATION
        }
        self.load_cfg()

        self.input_time = -1
        self.schedule = []
        self.inputs = []
        self.state = WhState()
        self.algo = Steerer()

    def load_cfg(self):
        """
        Loads all fields from the config dict into instance variables
        to avoid live variable tampering (mostly for GUI compatibility).
        """
        self.time_from: int = self.cfg["time_from"]
        self.time_to: int = self.cfg["time_to"]
        self.direction: int = self.cfg["direction"]
        self.generateCmdTime = self.generateDecimal if self.cfg["use_decimal"] else self.generateMs

    def on_registered(self, iface: TMInterface):
        print(f"[Wallhugger] Registered to {iface.server_name}")
        iface.execute_command("set controller none")
        iface.register_custom_command("wh")
        iface.log("[Wallhugger] Use the wh command to set a time range and direction: time_from-time_to wh direction")

    def on_custom_command(self, iface: TMInterface, time_from: int, time_to: int, command: str, args: list):
        if command == "wh":
            msg = self.on_wh(time_from, time_to, args)
        else:
            return
        iface.log(*msg)

    def on_wh(self, time_from: int, time_to: int, args: list):
        if time_to == -1 or time_from == -1:
            return "[Wallhugger] Timerange not set, Usage: time_from-time_to wh direction", "warning"
        elif len(args) == 1 and args[0] in ("left", "right"):
            self.cfg["direction"] = int(args[0] == "right") - int(args[0] == "left")
            self.cfg["time_from"], self.cfg["time_to"] = time_from, time_to
            return "[Wallhugger] wh settings changed successfully!", "success"
        return "[Wallhugger] Usage: time_from-time_to wh direction", "warning"

    def on_simulation_begin(self, iface: TMInterface):
        iface.remove_state_validation()
        self.load_cfg()
        if not self.direction:
            iface.log("[Wallhugger] Usage: time_from-time_to wh direction", "error")
            iface.close()
            print("[Wallhugger] Closed due to exception, use the wh command to change timerange and direction.")
            return
        print(f"[Wallhugger] Simulating from {self.time_from} to {self.time_to} trying to wh {self.direction}")
        self.input_time = self.time_from

        self.resetSeek()
        self.schedule = [self.setFirstInput]
        self.algo.start(self.direction)

        self.time_limit = self.time_to + self.seek + 10
        iface.set_simulation_time_limit(self.time_limit - 10010) # temporary manual offset

    def on_simulation_step(self, iface: TMInterface, _time: int):
        self.state.update(iface)
        if _time >= self.time_limit:
            return
        elif _time == self.input_time + self.seek:
            self.algo.data.append((self.state.wheval, self.steer))
            iface.rewind_to_state(self.step)
            return
        elif _time == self.input_time:
            self.executeSchedule()
            self.steer = self.algo.getSteer()
            if not self.algo.running:
                self.nextStep(iface)
                return
        elif _time == self.input_time - 10:
            self.step = self.state.data

        if _time >= self.input_time:
            iface.set_input_state(sim_clear_buffer=False, steer=self.steer)

    def executeSchedule(self):
        schedule = self.schedule.copy()
        self.schedule.clear()
        for fn in schedule:
            fn()

    def on_simulation_end(self, *_):
        print("[Wallhugger] Saving steering inputs to wallhugger.txt...")
        self.writeSteerToFile()

    def on_deregistered(self, *_):
        print("[Wallhugger] Attempting to back up most recent inputs to wallhugger.txt...")
        self.writeSteerToFile()

    def resetSeek(self):
        self.seek = 300

    def setFirstInput(self):
        self.inputs = [self.state.data.input_steer]

    def nextStep(self, iface: TMInterface):
        """Go to the next tick."""
        best = self.algo.getBest()
        iface.set_input_state(sim_clear_buffer=False, steer=best)
        self.inputs.append(best)
        print(f"{self.input_time} steer {best} -> {self.state.speed} km/h")

        self.input_time += 10
        self.algo.setupNewTick()
        iface.rewind_to_state(self.step)

    def writeSteerToFile(self):
        msg = "success!"
        try:
            with open("wallhugger.txt", 'w') as f:
                f.writelines(
                    [
                        f"{self.generateCmdTime(t[0])} steer {t[1]}\n" for t in
                        enumerate(self.inputs[1:]) if t[1] != self.inputs[t[0]]
                    ]
                )
        except:
            msg = "failed."
        finally:
            print(f"[Wallhugger] Input write {msg}")

    def generateMs(self, tick: int):
        return self.time_from + 10 * tick

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
                input("[Wallhugger] Enter the TMInterface instance ID you would like to connect to...\n")
            )
            server_id *= (server_id > 0)
        except:
            server_id = 0
        finally:
            return f"TMInterface{server_id}"

    def main(self, server_name = getServerName()):
        print(f"[Wallhugger] Connecting to {server_name}...")
        run_client(self, server_name)
        print(f"[Wallhugger] Deregistered from {server_name}")

class WhState:
    """Modified version of SimStateData that automatically calculates relevant values for this script."""
    def __init__(self):
        self.data = SimStateData()
        self.velocity = np.float64(0)
        self.speed = np.float64(0)
        self.setLocalVelocity()
        self.wheval = np.float64(0)

    def update(self, iface: TMInterface):
        """Run this at the start of every tick and you won't have to calculate anything in the main client."""
        self.data = iface.get_simulation_state()
        self.velocity = np.linalg.norm(self.data.velocity)
        self.speed = self.velocity * 3.6
        self.setLocalVelocity()
        sign = int(self.lvx > 0) - int(self.lvx < 0)
        self.wheval = self.getEvalVelocity() * sign

    def setLocalVelocity(self):
        self.lvx, self.lvy, self.lvz = [self.getLocalVelocity(i) for i in range(3)]

    def getLocalVelocity(self, idx: int):
        return np.sum(
            [self.data.velocity[i] * self.data.rotation_matrix[i][idx] for i in range(3)]
        )

    def getEvalVelocity(self):
        return np.linalg.norm((self.lvx, self.lvz ** 0.375))

class Steerer:
    """
    Steering algorithm implementation for finding wh steering values.\n
    The implementation uses a lot of hexadecimal
    because the steering range goes from -0x10000 to 0x10000
    which is easier to work with than 65536.
    """
    def __init__(self):
        self.desc: bool = False
        self.data: list[tuple] = []
        self.srange: tuple[int, int] = (0, 0)
        self.best: tuple[np.floating, int] = (0, 0)
        self.steerGen = None
        self.running: bool = False

    def start(self, direction: int):
        self.desc = direction == 1
        self.setupNewTick()

    def setupNewTick(self):
        self.data.clear()
        self.srange = (-0x10000, 0x10000)
        self.steerGen = self.getSteerGen()
        self.running = True

    def setSteerRange(self, step: int):
        best = self.getBest()
        self.srange = (best - step, best + step)

    def getBest(self):
        self.data.sort(reverse=self.desc)
        self.best = self.data[0]
        return self.best[1]

    def getSteerGen(self):
        """
        Generator that calculates a range of steering values
        based on the steering range.\n
        If the distance between potential candidates
        is small enough, reset the running flag
        and go past each int individually, in the remaining range.
        """
        mins, maxs = min(self.srange), max(self.srange)
        running = (step := (maxs - mins) >> 3) >= 4
        step = (offset := step * running) << 1 # Extremely cursed double walrus into bit-shift
        for s in range(mins + offset, maxs - offset + 1, step + (not running)):
            if s != self.best[1] and abs(s) <= 0x10000:
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

if __name__ == "__main__":
    Wallhugger().main()
