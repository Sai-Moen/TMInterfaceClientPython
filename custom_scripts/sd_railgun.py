# sd_railgun; sd script by SaiMoen

import numpy as np
from sys import argv
from tminterface.client import Client, run_client
from tminterface.interface import TMInterface

class MainClient(Client):
    def __init__(self):
        super().__init__()
        self.time_from: int = -1
        self.time_to: int = -1
        self.s4d: list[bool] = [False, False]
        self.do_wiggles: bool = False
        self.sd_eval: function = self.getHorizontalVelocity

    def on_registered(self, iface: TMInterface):
        print(f'Registered to {iface.server_name}')
        iface.execute_command('set controller none')
        iface.register_custom_command('sd')
        iface.log('[Railgun] Use the sd command to set a time range and direction: time_from-time_to sd <direction>')
        iface.register_custom_command('s4d')
        iface.log('[Railgun] Use the s4d command to toggle s4d assist, False by default')
        iface.register_custom_command('sdmode')
        iface.log('[Railgun] Use the sdmode command to switch between modes: horizontal (default, local), global and wiggle')

    def on_custom_command(self, iface: TMInterface, time_from: int, time_to: int, command: str, args: list):
        if command == 'sd':
            if time_to == -1 or time_from == -1:
                iface.log('[Railgun] Timerange not set, Usage: time_from-time_to sd <direction>', 'warning')

            elif len(args) == 1 and args[0] in ('left', 'right'):
                self.direction = int(args[0] == 'right') - int(args[0] == 'left')
                self.time_from, self.time_to = time_from, time_to
                iface.log('[Railgun] sd settings changed successfully!', 'success')

            else:
                iface.log('[Railgun] Usage: time_from-time_to sd <direction>', 'warning')

        elif command == 's4d':
            self.s4d[1] = not self.s4d[1]
            iface.log(f'[Railgun] s4d detection set to {self.s4d[1]}', 'success')

        elif command == 'sdmode':
            if len(args) == 1:
                if args[0] == 'horizontal':
                    self.sd_eval = self.getHorizontalVelocity
                    self.do_wiggles = False

                elif args[0] == 'global':
                    self.sd_eval = self.getVelocity
                    self.do_wiggles = False

                elif args[0] == 'wiggle':
                    self.sd_eval = self.getHorizontalVelocity
                    self.do_wiggles = True

                else:
                    iface.log('[Railgun] Invalid mode', 'error')
                    return

                iface.log(f'[Railgun] sdmode set to {args[0]}', 'success')

            else:
                iface.log('[Railgun] Usage: sdmode mode, where mode is: horizontal, global or wiggle', 'warning')

    def on_simulation_begin(self, iface: TMInterface):
        if self.time_to == -1 or self.time_from == -1:
            iface.log('[Railgun] Usage: time_from-time_to sd', 'error')
            iface.log('[Railgun] (closing to prevent exception): You forgot to set a time range', 'error')
            iface.close()

        iface.remove_state_validation()
        iface.set_simulation_time_limit(self.time_to - 9880) # tmp manual offset

        self.input_time = self.time_from
        self.inputs = [None]
        self.seek = 120
        self.seek_reset_time = -1
        self.s4d[0] = self.s4d[1]
        self.railgun = steerPredictor(self.direction)

    def on_simulation_step(self, iface: TMInterface, _time: int):
        if _time == self.input_time + self.seek:
            self.railgun + (self.sd_eval(iface), self.steer)
            iface.rewind_to_state(self.step)

        elif _time == self.input_time:
            if self.inputs[0] == None:
                self.inputs[0] = iface.get_simulation_state().input_steer

            if self.s4d[0]:
                self.s4dAssist(iface)

            elif _time == self.seek_reset_time:
                self.seek = 120

            self.steer: int = self.railgun.iter()
            if self.railgun.i == self.railgun.max_i:
                self.nextStep(iface)

        elif _time == self.input_time - 10:
            self.step = iface.get_simulation_state()

        if self.input_time <= _time < self.input_time + self.seek:
            iface.set_input_state(sim_clear_buffer=False, steer=self.steer)

    def on_simulation_end(self, iface: TMInterface, result: int):
        print('[Railgun] Saving steering inputs to sd_railgun.txt...')
        self.writeSteerToFile()
        print('[Railgun] Saved! Keep in mind that this is only the pad steering inputs found, and nothing else.')

    def on_deregistered(self, iface: TMInterface):
        print('[Railgun] Attempting to back up most recent inputs to sd_railgun.txt...')
        self.writeSteerToFile()

    def nextStep(self, iface: TMInterface):
        if self.railgun.best * self.direction < 0 and self.railgun.direction == self.direction:
            self.railgun.direction = -self.direction

        else:
            iface.set_input_state(sim_clear_buffer=False, steer=self.railgun.best)
            self.inputs.append(self.railgun.best)
            print(f'{self.input_time} steer {self.railgun.best} -> {self.getVelocity(iface) * 3.6} km/h')

            self.input_time += 10
            if self.do_wiggles and (self.input_time - self.time_from) // 290 % 2:
                self.railgun.direction = -self.direction

            else:
                self.railgun.direction = self.direction

        self.railgun.reset()
        iface.rewind_to_state(self.step)

    def s4dAssist(self, iface: TMInterface):
        state = iface.get_simulation_state()

        xx = state.rotation_matrix[0][0]
        yx = state.rotation_matrix[1][0]
        zx = state.rotation_matrix[2][0]

        vx = state.velocity[0]
        vy = state.velocity[1]
        vz = state.velocity[2]

        vx_local = vx * xx + vy * yx + vz * zx
        if vx_local * self.direction < 4:
            self.railgun.best = 65536 * self.direction
            self.nextStep(iface)

        else:
            self.seek = 130
            self.seek_reset_time = self.input_time + 60
            self.s4d[0] = False

    @staticmethod
    def getVelocity(iface: TMInterface):
        return np.linalg.norm(iface.get_simulation_state().velocity)

    @staticmethod
    def getHorizontalVelocity(iface: TMInterface):
        state = iface.get_simulation_state()

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
        )

    def writeSteerToFile(self):
        msg = 'success!'
        try:
            with open('sd_railgun.txt', 'w') as f:
                f.writelines(
                    [
                        f'{self.time_from + t[0] * 10} steer {t[1]}\n' for t in
                        enumerate(self.inputs[1:]) if t[1] != self.inputs[t[0]]
                    ]
                )
        except:
            msg = 'failed.'
        finally:
            print(f'[Railgun] Input write {msg}')

class steerPredictor:
    def __init__(self, direction: int):
        self.direction: int = direction
        self.i: int = 0
        self.max_i: int = 85
        self.vdata: list[tuple] = [(0, 0)] * self.max_i
        self.interval = (4096, 512, 64, 8, 1, 0)
        self.offset = (16, 25, 42, 59, 76, 85)

    def reset(self):
        self.i: int = 0
        self.vdata: list[tuple] = [(0, 0)] * self.max_i

    def __add__(self, pair: tuple):
        self.vdata[self.i] = pair
        self.i += 1

    def iter(self):
        self.vdata.sort(reverse=True)
        self.best: int = self.vdata[0][1]

        if self.i == (idx := self.i // 17) * 17:
            self.base = self.best

        steer: int = self.base + self.interval[idx] * (self.offset[idx] - self.i) * self.direction
        if steer > 65536:
            steer = 65536

        elif steer < -65536:
            steer = -65536

        if steer not in [v[1] for v in self.vdata if v != (0, 0)] or self.i == self.max_i:
            return steer

        self.i += 1
        return self.iter()

if __name__ == '__main__':
    server_name = f'TMInterface{argv[1]}' if len(argv) > 1 else 'TMInterface0'
    print(f'Connecting to {server_name}...')
    run_client(MainClient(), server_name)
