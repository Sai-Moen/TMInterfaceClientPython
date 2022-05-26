# sd_railgun; sd script by SaiMoen

import numpy as np
from sys import argv
from tminterface.client import Client, run_client
from tminterface.commandlist import CommandList as cmdlist
from tminterface.constants import ANALOG_STEER_NAME
from tminterface.interface import TMInterface

class MainClient(Client):
    def __init__(self) -> None:
        super().__init__()
        self.max_time = -1
        self.SEEK = 160

    def on_registered(self, iface: TMInterface):
        print(f'Registered to {iface.server_name}')
        iface.log('[Railgun] Use the sd command to set a time range: sd time_from time_to')
        iface.log('[Railgun] Make sure of the following: Bruteforce should be disabled,')
        iface.log('[Railgun] The steer input at the start time should be in the direction you would like to sd.')
        iface.register_custom_command('sd')

    def on_custom_command(self, iface: TMInterface, time_from: int, time_to: int, command: str, args: list):
        if command == 'sd':
            if len(args) == 2:
                time_from = cmdlist.parse_time(args[0])
                time_to = cmdlist.parse_time(args[1])
                if time_to < 0 or time_from < 0:
                    iface.log('[Railgun] Timerange must be in positive hundreths', 'error')
                elif time_to <= time_from:
                    iface.log('[Railgun] Start time must be lower than the end time.', 'error')
                else:
                    self.input_time = time_from
                    self.max_time = time_to + self.SEEK + 10
                    iface.log('[Railgun] Changed settings', 'success')
            else:
                iface.log('[Railgun] Usage: sd time_from time_to', 'warning')

    def on_simulation_begin(self, iface: TMInterface):
        if self.max_time == -1:
            iface.log('[Railgun] Usage: sd time_from time_to', 'warning')
            iface.log('[Railgun] (closing to prevent exception): You forgot to set a time range', 'error')
            iface.close()
        iface.remove_state_validation()

        # Re-init every simulation start
        self.velocity: list[tuple] = []
        self.final_inputs: list[tuple] = []
        self.last_steer: int = None

        # Fill original replay steering inputs
        self.inputs = [
            [val.time-100010, val.analog_value]
            for val in iface.get_event_buffer().find(event_name=ANALOG_STEER_NAME)
        ]
        for tick in range(self.max_time // 10):
            prev_steer = self.inputs[tick - 1][1] * (tick != 0)
            try:
                if self.inputs[tick][0] != tick * 10:
                    self.inputs.insert(tick, [None, prev_steer])
            except IndexError:
                self.inputs.append([None, prev_steer])
        self.inputs: list[int] = [inp[1] for inp in self.inputs]

        # Determine direction
        self.direction = int(self.inputs[self.input_time // 10])
        self.direction = (self.direction > 0) - (self.direction < 0)

    def on_simulation_step(self, iface: TMInterface, _time: int):
        # Earlier _time values are checked last and vice versa
        if _time > self.max_time:
            return

        elif _time == self.max_time:
            print('Saving steering inputs to sd_railgun.txt...')
            with open('sd_railgun.txt', 'w') as f:
                f.writelines([f'{t[0]} steer {t[1]}\n' for t in self.final_inputs])
            print('Saved! Keep in mind that this is only the pad steering inputs found, and nothing else.')
            return

        # After self.SEEK milliseconds, save the velocity and go back
        elif _time == self.input_time + self.SEEK:
            v = np.linalg.norm(iface.get_simulation_state().velocity)
            self.velocity.append((v, self.steer))
            iface.rewind_to_state(self.step)

        # Steering value algorithm
        elif _time == self.input_time:
            self.velocity.sort(reverse=True) # Descending velocity
            v_len = len(self.velocity)

            if v_len <= 32: # Get some starting points
                self.steer: int = 2048 * (32 - v_len) * self.direction

            elif 32 < v_len <= 43: # Start interpolating
                s1: int = self.velocity[0][1]
                s2: int = self.velocity[1][1]
                self.steer: int = s1 + (2 ** (43 - v_len)) * ((s1 < s2) - (s1 > s2))

            else: # Interpolated down to a change of 1, pick best steer
                s1: int = self.velocity[0][1]
                self.inputs[self.input_time // 10] = s1
                if self.last_steer != s1:
                    print(f'{self.input_time} steer {s1}')
                    self.final_inputs.append((self.input_time, s1))
                    self.last_steer = s1
                
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

if __name__ == '__main__':
    server_name = f'TMInterface{argv[1]}' if len(argv) > 1 else 'TMInterface0'
    print(f'Connecting to {server_name}...')
    run_client(MainClient(), server_name)