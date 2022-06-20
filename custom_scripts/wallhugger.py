# wallhugger; wallhug script by SaiMoen

import numpy as np
from sys import argv
from tminterface.client import Client, run_client
from tminterface.interface import TMInterface

class MainClient(Client):
    def __init__(self):
        super().__init__()
        self.time_from: int = -1
        self.time_to: int = -1

    def on_registered(self, iface: TMInterface):
        print(f'Registered to {iface.server_name}')
        iface.execute_command('set controller none')
        iface.register_custom_command('wallhug')
        iface.log('[Wallhugger] Use the wallhug command to set a time range: time_from-time_to wallhug <direction>')

    def on_custom_command(self, iface: TMInterface, time_from: int, time_to: int, command: str, args: list):
        if command == 'wallhug':
            if time_to == -1 or time_from == -1:
                iface.log('[Wallhugger] Timerange not set, Usage: time_from-time_to wallhug <direction>', 'warning')

            elif len(args) == 1 and args[0] in ('left', 'right'):
                self.direction = int((args[0] == 'right') - (args[0] == 'left'))
                self.time_from, self.time_to = time_from, time_to
                iface.log('[Wallhugger] wallhug settings changed successfully!', 'success')

            else:
                iface.log('[Wallhugger] Usage: time_from-time_to wallhug <direction>', 'warning')

    def on_simulation_begin(self, iface: TMInterface):
        if self.time_to == -1 or self.time_from == -1:
            iface.log('[Wallhugger] Usage: time_from-time_to wallhug', 'error')
            iface.log('[Wallhugger] (closing to prevent exception): You forgot to set a time range', 'error')
            iface.close()

        iface.remove_state_validation()

        self.input_time = self.time_from
        self.seek = 600
        self.inputs = [None]
        self.huggy = steerPredictor(self.direction)

    def on_simulation_step(self, iface: TMInterface, _time: int):
        if self.input_time > self.time_to:
            return

        elif _time == self.input_time + self.seek:
            state = iface.get_simulation_state()
            if self.prev_velocity > self.getHorizontalVelocity(state) + 0.08:
                self.huggy.i += 1

            else:
                self.huggy + (self.getYawDiff(state), self.steer)
            
            iface.rewind_to_state(self.step)

        elif _time == self.input_time:
            if self.inputs[0] == None:
                self.inputs[0] = iface.get_simulation_state().input_steer

            self.steer: int = self.huggy.iter()
            if self.huggy.i == self.huggy.max_i:
                self.inputs.append(self.huggy.best)
                print(f'{self.input_time} steer {self.huggy.best} -> {self.getVelocity(iface) * 3.6} km/h')

                self.huggy.reset()
                self.input_time += 10

                iface.rewind_to_state(self.step)

        elif _time == self.input_time - 10:
            self.step = iface.get_simulation_state()
            self.prev_velocity = self.getHorizontalVelocity(self.step)
            if _time >= self.time_from:
                idx = (self.input_time - self.time_from) // 10
                iface.set_input_state(sim_clear_buffer=False, steer=self.inputs[idx])

        if self.input_time <= _time < self.input_time + self.seek:
            iface.set_input_state(sim_clear_buffer=False, steer=self.steer)

    def on_simulation_end(self, iface: TMInterface, result: int):
        print('[Wallhugger] Saving steering inputs to wallhugger.txt...')
        self.writeSteerToFile()
        print('[Wallhugger] Saved! Keep in mind that this is only the pad steering inputs found, and nothing else.')

    def on_deregistered(self, iface: TMInterface):
        print('[Wallhugger] Attempting to back up most recent inputs to wallhugger.txt...')
        self.writeSteerToFile()

    def getYawDiff(self, state):
        tau = 2 * np.pi
        yaw = state.yaw_pitch_roll[0]
        diff1 = (self.step.yaw_pitch_roll[0] - yaw) % tau
        diff2 = (yaw - self.step.yaw_pitch_roll[0]) % tau
        return self.direction * (-diff2 if diff2 < diff1 else diff1)

    @staticmethod
    def getVelocity(iface: TMInterface):
        return np.linalg.norm(iface.get_simulation_state().velocity)

    @staticmethod
    def getHorizontalVelocity(state):
        xx = state.rotation_matrix[0][0]
        yx = state.rotation_matrix[1][0]
        zx = state.rotation_matrix[2][0]

        xz = state.rotation_matrix[0][2]
        yz = state.rotation_matrix[1][2]
        zz = state.rotation_matrix[2][2]

        vx = state.velocity[0]
        vy = state.velocity[1]
        vz = state.velocity[2]

        return np.linalg.norm(
            (
                vx * xx + vy * yx + vz * zx,
                vx * xz + vy * yz + vz * zz
            )
        ) - 6 * state.yaw_pitch_roll[1]

    def writeSteerToFile(self):
        msg = 'success!'
        try:
            with open('wallhugger.txt', 'w') as f:
                f.writelines(
                    [
                        f'{self.time_from + t[0] * 10} steer {t[1]}\n' for t in
                        enumerate(self.inputs[1:]) if t[1] != self.inputs[t[0]]
                    ]
                )
        except:
            msg = 'failed.'
        finally:
            print(f'[Wallhugger] Input write {msg}')

class steerPredictor:
    def __init__(self, direction: int):
        self.direction: int = direction
        self.i: int = 0
        self.max_i: int = 23
        self.vdata: list[tuple] = [(0, 0)] * self.max_i

    def reset(self):
        self.i: int = 0
        self.vdata: list[tuple] = [(0, 0)] * self.max_i

    def __add__(self, pair: tuple):
        self.vdata[self.i] = pair
        self.i += 1

    def iter(self):
        self.vdata.sort(reverse=True)
        self.best: int = self.vdata[0][1]

        if self.i <= 8:
            steer = 16384 * (4 - self.i) * self.direction

        elif self.i <= 22:
            steer = self.best + 2 ** (22 - self.i) * self.direction

            if steer > 65536:
                steer = 65536

            elif steer < -65536:
                steer = -65536

        else: steer = 0

        if steer not in [v[1] for v in self.vdata if v != (0, 0)]:
            return steer

        elif self.i == self.max_i:
            return

        else:
            self.i += 1
            return self.iter()

if __name__ == '__main__':
    server_name = f'TMInterface{argv[1]}' if len(argv) > 1 else 'TMInterface0'
    print(f'Connecting to {server_name}...')
    run_client(MainClient(), server_name)