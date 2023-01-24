# wallhugger; wh script by SaiMoen

import numpy as np
from tminterface.client import Client, run_client
from tminterface.interface import TMInterface
from tminterface.structs import SimStateData
from struct import unpack

USE_DECIMAL_NOTATION = False # set to True for decimal notation, False for milliseconds
# You shouldn't need to change anything except for the previous line (if you want decimal notation of course)

FULLSTEER = 0x10000
TICK_MS = 10
SEEK = 40 * TICK_MS
HOLD = 30 * TICK_MS

TAG = "[Wallhugger] "
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

class Wallhugger(Client):
    """Main Client Implementation."""
    def __init__(self):
        self.cfg = {
            "time_from": -1,
            "time_to": -1,
            "direction": 0,
            "use_decimal": USE_DECIMAL_NOTATION
        }
        self.load_cfg()

        self.input_time = -1
        self.inputs = [None]
        self.state = WhState()

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
        print(TAG + f"Registered to {iface.server_name}")
        iface.execute_command("set controller none")
        iface.register_custom_command("wh")
        iface.log(TAG + "Use the wh command to set a time range and direction: time_from-time_to wh direction")

    def on_custom_command(self, iface: TMInterface, time_from: int, time_to: int, command: str, args: list):
        if command == "wh":
            msg = self.on_wh(time_from, time_to, args)
        else:
            iface.log(TAG + "Invalid command", "error")
            return
        iface.log(*msg)

    def on_wh(self, time_from: int, time_to: int, args: list):
        if time_to == -1 or time_from == -1:
            return TAG + "Timerange not set, Usage: time_from-time_to wh direction", "warning"
        elif len(args) == 1 and args[0] in ("left", "right"):
            self.cfg["direction"] = int(args[0] == "right") - int(args[0] == "left")
            self.cfg["time_from"], self.cfg["time_to"] = time_from, time_to
            return TAG + "wh settings changed successfully!", "success"
        return TAG + "Usage: time_from-time_to wh direction", "warning"

    def on_simulation_begin(self, iface: TMInterface):
        iface.remove_state_validation()
        self.load_cfg()
        if not self.direction:
            iface.log(TAG + "Usage: time_from-time_to wh direction", "error")
            iface.close()
            print(TAG + "Closed due to exception, use the wh command to change timerange and direction.")
            return
        print(TAG + "Simulation variables:")
        print(f"time_from: {self.time_from}")
        print(f"time_to: {self.time_to}")
        print(f"direction: {self.direction}")
        self.input_time = self.time_from

        self.fullsteer = FULLSTEER * self.direction
        self.countersteer = -FULLSTEER * self.direction

        self.inputs: list = [None]
        self.reset()
        self.time_limit = self.time_to + SEEK + TICK_MS
        iface.set_simulation_time_limit(self.time_limit - 10010) # temporary manual offset

    def on_simulation_step(self, iface: TMInterface, _time: int):
        self.state.update(iface, _time)
        if _time >= self.time_limit:
            return
        elif _time == self.input_time + SEEK:
            self.onNoCollision(iface)
        elif _time > self.input_time:
            self.onCollisionCheck(iface)
        elif _time == self.input_time:
            self.onInputTime(iface)
        elif _time == self.input_time - TICK_MS:
            self.step = self.state.data

    def on_simulation_end(self, *_):
        print(TAG + "Saving steering inputs to wallhugger.txt...")
        self.writeSteerToFile()

    def on_deregistered(self, *_):
        print(TAG + "Attempting to back up most recent inputs to wallhugger.txt...")
        self.writeSteerToFile()

    def reset(self):
        self.isDone = False
        self.isAvoiding = False
        self.avoider = self.countersteer
        self.collider = self.fullsteer
        self.steer = self.fullsteer

    def inputSteer(self, iface: TMInterface, steer: int):
        iface.set_input_state(sim_clear_buffer=False, steer=steer)

    def getSteer(self):
        if self.state.isColliding:
            self.collider = self.steer
        else:
            self.avoider = self.steer
        self.isDone = abs(self.avoider - self.collider) <= 1
        if self.isDone:
            return self.avoider
        return (self.avoider + self.collider) >> 1

    def onInputTime(self, iface: TMInterface):
        if self.isDone:
            self.inputSteer(iface, self.avoider)
            self.inputs.append(self.avoider)
            print(f"{self.input_time} steer {self.avoider} -> {self.state.speed} km/h")

            self.reset()
            self.input_time += TICK_MS
            iface.rewind_to_state(self.step)
            return
        self.inputSteer(iface, self.steer)

    def onNoCollision(self, iface: TMInterface):
        if self.isAvoiding:
            self.steer = self.getSteer()
        else:
            self.avoider = self.steer
            self.isDone = True
        iface.rewind_to_state(self.step)

    def onCollisionCheck(self, iface: TMInterface):
        if self.state.isColliding:
            self.isAvoiding = True
            self.steer = self.getSteer()
            iface.rewind_to_state(self.step)
            return
        steer = self.countersteer if self.state.race_time > self.input_time + HOLD else self.steer
        self.inputSteer(iface, steer)

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

class WhState:
    """Modified version of SimStateData that automatically calculates relevant values for this script."""
    def __init__(self):
        self.race_time = -2600
        self.data = SimStateData()
        self.velocity = np.float64(0)
        self.speed = np.float64(0)
        self.isColliding = False

    def update(self, iface: TMInterface, _time: int):
        """Run this at the start of every tick and you won't have to calculate anything in the main client."""
        self.race_time = _time
        self.data = iface.get_simulation_state()
        self.velocity = np.linalg.norm(self.data.velocity)
        self.speed = self.velocity * 3.6
        self.isColliding = any(unpack('2i', self.data.scene_mobil[1496:1504]))

if __name__ == "__main__":
    Wallhugger().main()
