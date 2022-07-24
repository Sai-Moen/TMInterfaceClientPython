# sd_railgun; sd script by SaiMoen

import numpy as np
from tminterface.client import Client, run_client
from tminterface.interface import TMInterface

from struct import unpack
from tminterface.constants import SIMULATION_WHEELS_SIZE

class MainClient(Client):
    def __init__(self):
        # Default config
        super().__init__()
        self.time_from: int = -1
        self.time_to: int = -1
        self.default_seek: int = 120
        self.max_wiggle_timer: int = 300
        self.sdmode: list[bool] = [False, False]

        # Global to Local velocity vector transposing lambda
        self.stateToLocalVelocity = lambda state, idx: np.sum(
            [state.velocity[i] * state.rotation_matrix[i][idx] for i in range(3)]
        )

        # Evaluation velocity calculation lambda
        allWheelsOnGround = lambda state: all(
            [
                unpack('i', state.simulation_wheels[i+292:i+296])[0]
                for i in [(SIMULATION_WHEELS_SIZE // 4) * j for j in range(4)]
            ]
        )
        self.getEvalVelocity = lambda state: np.linalg.norm(
            (
                self.stateToLocalVelocity(state, 0),
                self.stateToLocalVelocity(state, 2)
            ) if allWheelsOnGround(state) else state.velocity
        )

    def on_registered(self, iface: TMInterface):
        print(f'[Railgun] Registered to {iface.server_name}')
        iface.execute_command('set controller none')
        iface.register_custom_command('sd')
        iface.log('[Railgun] Use the sd command to set a time range and direction: time_from-time_to sd <direction>')
        iface.register_custom_command('sdmode')
        iface.log('[Railgun] Use the sdmode command to switch between: normal, s4d, s4dirt or wiggle. normal by default')

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

        elif command == 'sdmode':
            if len(args) == 0:
                self.sdmode = [False, False]
                iface.log('[Railgun] sdmode reset to normal', 'success')

            elif len(args) == 1:
                if args[0] == 'normal':
                    self.sdmode = [False, False]
                    iface.log('[Railgun] sdmode set to normal', 'success')

                elif args[0] == 's4d':
                    self.sdmode = [True, False]
                    self.minLvx = lambda speed: 4 - 0.5 * ((401 < speed < 501) * ((speed - 401) / 100) + (speed >= 501))
                    iface.log('[Railgun] sdmode set to s4d', 'success')
                
                elif args[0] == 's4dirt':
                    self.sdmode = [True, False]
                    self.minLvx = lambda speed: 0.25 + 0.25 * (speed >= 234)
                    iface.log('[Railgun] sdmode set to s4d', 'success')

                elif args[0] == 'wiggle':
                    self.sdmode = [False, True]
                    iface.log('[Railgun] sdmode set to wiggle', 'success')

                else:
                    iface.log('[Railgun] Invalid mode', 'warning')

            else:
                iface.log('[Railgun] Usage: sdmode <normal, s4d, s4dirt or wiggle>', 'warning')

    def on_simulation_begin(self, iface: TMInterface):
        # Close because otherwise we get an exception and close anyway
        if self.time_to == -1 or self.time_from == -1:
            iface.log('[Railgun] Usage: time_from-time_to sd', 'error')
            iface.log('[Railgun] (closing to prevent exception): You forgot to set a time range', 'error')
            iface.close()

        # Required to properly change inputs
        self.time_limit = self.time_to + self.default_seek + 10
        iface.remove_state_validation()
        iface.set_simulation_time_limit(self.time_limit - 10010) # temporary manual offset

        # Variable config
        self.input_time = self.time_from
        self.inputs = [None]
        self.countersteered = False
        self.railgun = steerPredictor(self.direction)
        self.seek = self.default_seek
        self.seek_reset_time = -1
        self.s4d, self.wiggle = self.sdmode
        self.wiggle_timer = self.max_wiggle_timer

    def on_simulation_step(self, iface: TMInterface, _time: int):
        if _time >= self.time_limit:
            return

        elif _time == self.input_time + self.seek:
            self.railgun + (self.getEvalVelocity(iface.get_simulation_state()), self.steer)
            iface.rewind_to_state(self.step)
            return

        elif _time == self.input_time:
            if self.inputs[0] == None:
                self.inputs[0] = iface.get_simulation_state().input_steer

            if self.s4d:
                self.s4dAssist(iface)
                if self.s4d:
                    return

            elif _time == self.seek_reset_time:
                self.seek = self.default_seek

            self.steer: int = self.railgun.iter()
            if self.railgun.i == self.railgun.max_i:
                self.nextStep(iface)
                return

        elif _time == self.input_time - 10:
            self.step = iface.get_simulation_state()
            return

        if _time >= self.input_time:
            iface.set_input_state(sim_clear_buffer=False, steer=self.steer)

    def on_simulation_end(self, iface: TMInterface, result: int):
        print('[Railgun] Saving steering inputs to sd_railgun.txt...')
        self.writeSteerToFile()

    def on_deregistered(self, iface: TMInterface):
        print('[Railgun] Attempting to back up most recent inputs to sd_railgun.txt...')
        self.writeSteerToFile()

    def nextStep(self, iface: TMInterface):
        if self.railgun.best * self.railgun.direction < 0 and not self.countersteered:
            self.railgun.direction *= -1
            self.countersteered = True

        else:
            iface.set_input_state(sim_clear_buffer=False, steer=self.railgun.best)
            self.inputs.append(self.railgun.best)
            v = np.linalg.norm(iface.get_simulation_state().velocity)
            print(f'{self.input_time} steer {self.railgun.best} -> {v * 3.6} km/h')

            self.input_time += 10
            if self.countersteered:
                self.railgun.direction *= -1
                self.countersteered = False

            if self.wiggle:
                self.wiggle_timer -= 10
                if v < np.linalg.norm(self.step.velocity):
                    self.wiggle_timer = self.max_wiggle_timer

                elif not self.wiggle_timer:
                    self.wiggle_timer = self.max_wiggle_timer
                    self.railgun.direction *= -1

        self.railgun.reset()
        iface.rewind_to_state(self.step)

    def s4dAssist(self, iface: TMInterface):
        state = iface.get_simulation_state()

        local_vx = self.stateToLocalVelocity(state, 0) * self.railgun.direction
        if local_vx < self.minLvx(np.linalg.norm(state.velocity) * 3.6):
            self.railgun.best = 65536 * self.railgun.direction
            self.nextStep(iface)

        else:
            self.seek = 130
            self.seek_reset_time = self.input_time + 60
            self.s4d = False

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
        self.interval: tuple[int] = (4096, 512, 64, 8, 1, 0)
        self.offset: tuple[int] = (16, 25, 42, 59, 76, 85)

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
        steer: int = self.limitInputs(steer)
        if steer not in [v[1] for v in self.vdata if v != (0, 0)] or self.i == self.max_i:
            return steer

        self.i += 1
        return self.iter()

    def limitInputs(self, steer: int):
        return steer if abs(steer) <= 65536 else 65536 * self.direction

if __name__ == '__main__':
    try:
        server_id = int(
            input('[Railgun] Enter the TMInterface instance ID you would like to connect to...\n')
        )
        server_name = f'TMInterface{server_id * (0 <= server_id < 16)}'
    except:
        server_name = 'TMInterface0'
    finally:
        print(f'[Railgun] Connecting to {server_name}...')
        run_client(MainClient(), server_name)
        print(f'[Railgun] Deregistered from {server_name}')
