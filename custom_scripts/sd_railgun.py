# sd_railgun; sd script by SaiMoen

import numpy as np
from sys import argv
from tminterface.client import Client, run_client
from tminterface.constants import ANALOG_STEER_NAME
from tminterface.interface import TMInterface

class MainClient(Client):
    def __init__(self):
        super().__init__()
        self.time_from = -1
        self.time_to = -1
        self.SEEK = 120

    def on_registered(self, iface: TMInterface):
        print(f'Registered to {iface.server_name}')
        iface.execute_command('set controller none')
        iface.register_custom_command('sd')
        iface.log('[Railgun] Use the sd command to set a time range: time_from-time_to sd')

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
                    iface.log('[Railgun] Settings changed succesfully!', 'success')

    def on_simulation_begin(self, iface: TMInterface):
        if self.time_to == -1 or self.time_from == -1:
            iface.log('[Railgun] Usage: time_from-time_to sd', 'error')
            iface.log('[Railgun] (closing to prevent exception): You forgot to set a time range', 'error')
            iface.close()
        iface.remove_state_validation()

        # Fill steering inputs
        self.inputs = [
            (st.time-100010, st.analog_value)
            for st in iface.get_event_buffer().find(event_name=ANALOG_STEER_NAME)
        ]

        for tick in range(self.t_idx):
            prev_steer = (None, self.inputs[tick - 1][1] * (tick != 0))
            try:
                if self.inputs[tick][0] != tick * 10:
                    self.inputs.insert(tick, prev_steer)
            except IndexError:
                self.inputs.append(prev_steer)

        self.inputs: list[int] = [inp[1] for inp in self.inputs]
        self.direction = int(np.sign(
            sum(self.inputs[self.f_idx:self.t_idx])
        ))
        # Re-init on every simulation start
        self.input_time = self.time_from
        self.velocity: list[tuple] = []

    def on_simulation_step(self, iface: TMInterface, _time: int):
        if self.input_time > self.time_to:
            return

        # After self.SEEK milliseconds, save the velocity and go back
        elif _time == self.input_time + self.SEEK:
            self.velocity.append(
                (np.linalg.norm(iface.get_simulation_state().velocity), self.steer)
            )
            iface.rewind_to_state(self.step)

        # Steering value algorithm
        elif _time == self.input_time:
            self.velocity.sort(reverse=True) # Descending velocity
            v_len = len(self.velocity)

            if v_len <= 32: # Get some starting points
                self.steer: int = 2048 * (32 - v_len) * self.direction

            elif v_len <= 43: # Start interpolating
                s1: int = self.velocity[0][1]
                s2: int = self.velocity[1][1]
                self.steer: int = s1 + (2 ** (43 - v_len)) * ((s1 < s2) - (s1 > s2))

            else: # Interpolated down to a change of 1, pick best steer
                s1: int = self.velocity[0][1]
                self.inputs[_time // 10] = s1
                print(f'{_time} steer {s1}')

                # Reset vars, go to next tick
                self.velocity = []
                self.input_time += 10
                iface.rewind_to_state(self.step)

        # Grab new savestate and apply steering value obtained from previous iteration
        elif _time == self.input_time - 10:
            self.step = iface.get_simulation_state()
            iface.set_input_state(sim_clear_buffer=False, steer= -self.inputs[_time // 10])
            # REMOVE "-" AFTER v1.1.2 DROPS, this is a temporary fix for an input flip bug

        # Apply test steer values
        if self.input_time <= _time < self.input_time + self.SEEK:
            iface.set_input_state(sim_clear_buffer=False, steer= -self.steer)
            # REMOVE "-" AFTER v1.1.2 DROPS, this is a temporary fix for an input flip bug

    def on_simulation_end(self, iface: TMInterface, result: int):
        print('[Railgun] Saving steering inputs to sd_railgun.txt...')
        with open('sd_railgun.txt', 'w') as f:
            f.writelines(
                [
                    f'{t[0] * 10} steer {t[1]}\n' for t in
                    enumerate(self.inputs[self.f_idx:self.t_idx], self.f_idx)
                    if t[1] != self.inputs[t[0] - 1]
                ]
            )
        print('[Railgun] Saved! Keep in mind that this is only the pad steering inputs found, and nothing else.')

if __name__ == '__main__':
    server_name = f'TMInterface{argv[1]}' if len(argv) > 1 else 'TMInterface0'
    print(f'Connecting to {server_name}...')
    run_client(MainClient(), server_name)
