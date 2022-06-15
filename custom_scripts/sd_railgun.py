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
        self.s4d_cfg: bool = False
        self.sdh_cfg: bool = False

    def on_registered(self, iface: TMInterface):
        print(f'Registered to {iface.server_name}')
        iface.execute_command('set controller none')
        iface.register_custom_command('sd')
        iface.log('[Railgun] Use the sd command to set a time range: time_from-time_to sd')
        iface.register_custom_command('s4d')
        iface.log('[Railgun] Use the s4d command to toggle s4d assist, False by default')
        iface.register_custom_command('sdh')
        iface.log('[Railgun] Use the sdh command to toggle local horizontal velocity evaluation, False by default')

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

        elif command == 's4d':
            self.s4d_cfg = not self.s4d_cfg
            iface.log(f'[Railgun] s4d detection; set to {self.s4d_cfg}', 'success')

        elif command == 'sdh':
            self.sdh_cfg = not self.sdh_cfg
            iface.log(f'[Railgun] Local horizontal only evaluation; set to {self.sdh_cfg}', 'success')

    def on_simulation_begin(self, iface: TMInterface):
        if self.time_to == -1 or self.time_from == -1:
            iface.log('[Railgun] Usage: time_from-time_to sd', 'error')
            iface.log('[Railgun] (closing to prevent exception): You forgot to set a time range', 'error')
            iface.close()
        iface.remove_state_validation()

        self.input_time = self.time_from
        self.seek = 120
        self.seek_reset_time = -1
        self.s4d = self.s4d_cfg
        self.sd_eval = self.getHorizontalVelocity if self.sdh_cfg else self.getVelocity
        self.inputs = self.fillInputs(iface)
        self.direction = int(np.sign(sum(
            self.inputs[self.f_idx:self.t_idx]
        )))
        self.sHelper = steerPredictor(self.direction)

    def on_simulation_step(self, iface: TMInterface, _time: int):
        if self.input_time > self.time_to:
            return

        elif _time == self.input_time + self.seek:
            self.sHelper + (self.sd_eval(iface), self.steer)
            iface.rewind_to_state(self.step)

        elif _time == self.input_time:
            if _time == self.seek_reset_time:
                self.seek = 120

            self.steer: int = self.sHelper.iter()
            if self.sHelper.i == self.sHelper.max_i:
                self.nextStep(iface, _time)

            elif self.s4d:
                if self.getSidewaysVelocity(iface) * self.direction < 4:
                    self.sHelper.best = 65536 * self.direction
                    self.nextStep(iface, _time)

                else:
                    self.seek = 130
                    self.seek_reset_time = _time + 60
                    self.s4d = False

        elif _time == self.input_time - 10:
            self.step = iface.get_simulation_state()
            iface.set_input_state(sim_clear_buffer=False, steer=self.inputs[_time // 10])

        if self.input_time <= _time < self.input_time + self.seek:
            iface.set_input_state(sim_clear_buffer=False, steer=self.steer)

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

    def nextStep(self, iface: TMInterface, time: int):
        self.inputs[time // 10] = self.sHelper.best
        print(f'{time} steer {self.sHelper.best} -> {self.getVelocity(iface) * 3.6} km/h')

        self.sHelper.reset()
        self.input_time += 10
        iface.rewind_to_state(self.step)

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

    @staticmethod
    def getSidewaysVelocity(iface: TMInterface):
        state = iface.get_simulation_state()
        return (
            state.velocity[0] * state.rotation_matrix[0][0] +
            state.velocity[1] * state.rotation_matrix[1][0] +
            state.velocity[2] * state.rotation_matrix[2][0]
        )

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
    def __init__(self, direction: int):
        self.direction: int = direction
        self.i: int = 0
        self.max_i: int = 85
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

        if self.i == (idx := self.i // 17) * 17:
            self.base = self.best
        interval = 2 ** (12 - idx * 3)
        midpoint = (16, 25, 42, 59, 76, 84)[idx]

        steer: int = self.base + interval * (midpoint - self.i) * self.direction
        if steer > 65536:
            steer = 65536

        elif steer < -65536:
            steer = -65536

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
