# sd_railgun; sd script by SaiMoen

import numpy as np
from sys import argv
from tminterface.client import Client, run_client
from tminterface.constants import ANALOG_STEER_NAME
from tminterface.interface import TMInterface

class MainClient(Client):
    def __init__(self):
        super().__init__()
        self.time_from: int = -1
        self.time_to: int = -1
        self.mode: str = 'main'

    def on_registered(self, iface: TMInterface):
        print(f'Registered to {iface.server_name}')
        iface.execute_command('set controller none')
        iface.register_custom_command('sd')
        iface.log('[Railgun] Use the sd command to set a time range: time_from-time_to sd')
        iface.register_custom_command('sdmode')
        iface.log('[Railgun] Use the sdmode command to switch between modes; main or road')

    def on_custom_command(self, iface: TMInterface, time_from: int, time_to: int, command: str, args: list):
        if command == 'sd':
            if len(args) > 0:
                iface.log('[Railgun] Usage: time_from-time_to sd', 'warning')

            else:
                self.time_from, self.f_idx = time_from, time_from // 10
                self.time_to, self.t_idx = time_to, time_to // 10 + 1
                if self.time_to == -1 or self.time_from == -1:
                    iface.log('[Railgun] Timerange not set, Usage: time_from-time_to sd', 'warning')

                else:
                    iface.log('[Railgun] Timerange changed successfully!', 'success')

        elif command == 'sdmode':
            modes = ('main', 'road') #, 'dirt', 'grass')
            if len(args) == 1:
                for m in modes:
                    if args[0] == m:
                        self.mode = m
                        iface.log('[Railgun] Mode changed successfully!', 'success')
                        return
            iface.log('[Railgun] Usage: sdmode <mode>', 'warning')
            iface.log('[Railgun] Modes: main, road', 'warning')

    def on_simulation_begin(self, iface: TMInterface):
        if self.time_to == -1 or self.time_from == -1:
            iface.log('[Railgun] Usage: time_from-time_to sd', 'error')
            iface.log('[Railgun] (closing to prevent exception): You forgot to set a time range', 'error')
            iface.close()
        iface.remove_state_validation()

        if self.mode == 'main':
            eval_descending = True
            self.seek = 120
            self.eval = self.getVelocity

        elif self.mode == 'road':
            eval_descending = False
            self.seek = 50
            self.eval = self.getQuality
            self.offset = self.roadOffset

        elif self.mode == 'dirt':
            eval_descending = False
            self.seek = 50
            self.eval = self.getQuality
            self.offset = self.dirtOffset

        elif self.mode == 'grass':
            eval_descending = False
            self.seek = 50
            self.eval = self.getQuality
            self.offset = self.grassOffset

        self.input_time = self.time_from
        self.inputs: list[int] = self.fillInputs(iface)
        self.sHelper = steerPredictor(
            int(np.sign(sum(
                self.inputs[self.f_idx:self.t_idx]
            ))),
            eval_descending
        )

    def on_simulation_step(self, iface: TMInterface, _time: int):
        if self.input_time > self.time_to:
            return

        elif _time == self.input_time + self.seek:
            self.sHelper + (self.eval(iface), self.steer)
            iface.rewind_to_state(self.step)

        elif _time == self.input_time:
            self.steer: int = self.sHelper.iter()
            if self.sHelper.i == self.sHelper.max_i:
                self.inputs[_time // 10] = self.sHelper.s1
                print(f'{_time} steer {self.sHelper.s1} -> {self.getVelocity(iface) * 3.6} km/h')

                self.sHelper.reset()
                self.input_time += 10
                iface.rewind_to_state(self.step)

        elif _time == self.input_time - 10:
            self.step = iface.get_simulation_state()
            iface.set_input_state(sim_clear_buffer=False, steer= -self.inputs[_time // 10])
            # REMOVE "-" AFTER v1.1.2 DROPS, this is a temporary fix for an input flip bug

        if self.input_time <= _time < self.input_time + self.seek:
            iface.set_input_state(sim_clear_buffer=False, steer= -self.steer)
            # REMOVE "-" AFTER v1.1.2 DROPS, this is a temporary fix for an input flip bug

    def on_simulation_end(self, iface: TMInterface, result: int):
        print('[Railgun] Saving steering inputs to sd_railgun.txt...')
        self.writeSteerToFile()
        print('[Railgun] Saved! Keep in mind that this is only the pad steering inputs found, and nothing else.')

    def on_deregistered(self, iface: TMInterface):
        print('[Railgun] Attempting to back up most recent inputs to sd_railgun.txt...')
        self.writeSteerToFile()

    def fillInputs(self, iface: TMInterface):
        inputs = [
            (st.time-100010, st.analog_value)
            for st in iface.get_event_buffer().find(event_name=ANALOG_STEER_NAME)
        ]

        for tick in range(self.t_idx):
            prev_steer = (None, inputs[tick - 1][1] * (tick != 0))
            try:
                if inputs[tick][0] != tick * 10:
                    inputs.insert(tick, prev_steer)
            except IndexError:
                inputs.append(prev_steer)

        return [inp[1] for inp in inputs]

    def getQuality(self, iface: TMInterface):
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

        v_sideways = abs(vx * xx + vy * yx + vz * zx)
        v_forwards = vx * xz + vy * yz + vz * zz
        h_speed = np.linalg.norm((v_sideways, v_forwards)) * 3.6

        return abs(v_sideways - self.offset(h_speed))

    @staticmethod
    def roadOffset(h_speed: np.floating):
        return 5.5 + (h_speed < 401) * 0.5 + (401 <= h_speed < 501) * (501 - h_speed) * 0.005

    @staticmethod
    def dirtOffset(h_speed: np.floating):
        pass

    @staticmethod
    def grassOffset(h_speed: np.floating):
        pass

    @staticmethod
    def getVelocity(iface: TMInterface):
        return np.linalg.norm(iface.get_simulation_state().velocity)

    def writeSteerToFile(self):
        msg = 'success!'
        try:
            with open('sd_railgun.txt', 'w') as f:
                f.writelines(
                    [
                        f'{t[0] * 10} steer {t[1]}\n' for t in
                        enumerate(self.inputs[self.f_idx:self.t_idx], self.f_idx)
                        if t[1] != self.inputs[t[0] - 1]
                    ]
                )
        except:
            msg = 'failed.'
        finally:
            print(f'[Railgun] Input write {msg}')

class steerPredictor:
    def __init__(self, direction: int, eval_descending: bool):
        self.direction: int = direction
        self.eval_descending: bool = eval_descending
        self.i: int = 0
        self.max_i: int = 85
        self.pairs: list[tuple] = [(6, 0)] * self.max_i

    def reset(self):
        self.i: int = 0
        self.pairs: list[tuple] = [(6, 0)] * self.max_i

    def __add__(self, pair: tuple):
        self.pairs[self.i] = pair
        self.i += 1

    def iter(self):
        self.pairs.sort(reverse=self.eval_descending)
        self.s1: int = self.pairs[0][1]
        
        idx = self.i // 17
        if self.i == idx * 17:
            self.base = self.s1
        interval = 2 ** (12 - idx * 3)
        midpoint = (16, 25, 42, 59, 76, 84)[idx]

        steer: int = self.base + interval * (midpoint - self.i) * self.direction
        if abs(steer) > 65536:
            steer = self.direction * 65536

        if steer not in [t[1] for t in self.pairs if t != (6, 0)]:
            return steer

        else:
            self.i += 1
            return self.iter()

if __name__ == '__main__':
    server_name = f'TMInterface{argv[1]}' if len(argv) > 1 else 'TMInterface0'
    print(f'Connecting to {server_name}...')
    run_client(MainClient(), server_name)
