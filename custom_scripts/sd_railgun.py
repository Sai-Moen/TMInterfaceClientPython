# sd_railgun; sd script by SaiMoen

import numpy as np
from tminterface.client import Client, run_client
from tminterface.interface import TMInterface
from tminterface.structs import SimStateData

from struct import unpack
from tminterface.constants import SIMULATION_WHEELS_SIZE

USE_DECIMAL_NOTATION = False # set to True for decimal notation, False for milliseconds

class Railgun(Client):
    def __init__(self):
        # Default config
        self.s4d, self.wiggle = False, False

        self.minLvxRoad = lambda speed: 4 - 0.5 * ((speed > 401) * ((speed - 401) / 100))
        self.minLvxDirt = lambda speed: 0.25 + (speed > 235) * 0.001 * (306 - speed)

        self.ordered_input_ops = (self.nextStep, self.getFirstInput, self.getInputTimeState, self.s4dExec, self.s4dReset)
        self.ordered_step_ops = (self.toggleCountersteer, self.wiggleExec)

        # set time notation function pointer
        generateMS = lambda tick: self.time_from + tick * 10
        self.generateCmdTime = self.generateDecimal if USE_DECIMAL_NOTATION else generateMS

    def on_registered(self, iface: TMInterface):
        print(f'[Railgun] Registered to {iface.server_name}')
        iface.execute_command('set controller none')
        iface.register_custom_command('sd')
        iface.log('[Railgun] Use the sd command to set a time range and direction: time_from-time_to sd <direction>')
        iface.register_custom_command('sdmode')
        iface.log(
            '[Railgun] Use the sdmode command to switch between: normal, s4d, s4dirt, wiggle or wiggledirt. normal by default.'
        )

    def on_custom_command(self, iface: TMInterface, time_from: int, time_to: int, command: str, args: list):
        if command == 'sd':
            if time_to == -1 or time_from == -1:
                iface.log('[Railgun] Timerange not set, Usage: time_from-time_to sd <direction>', 'warning')

            elif len(args) == 1 and args[0] in ('left', 'right'):
                self._direction = int(args[0] == 'right') - int(args[0] == 'left')
                self.time_from, self.time_to = time_from, time_to
                iface.log('[Railgun] sd settings changed successfully!', 'success')

            else: iface.log('[Railgun] Usage: time_from-time_to sd <direction>', 'warning')

        elif command == 'sdmode':
            if len(args) == 0:
                self.s4d, self.wiggle = False, False
                iface.log('[Railgun] sdmode reset to normal', 'success')

            elif len(args) == 1:
                if args[0] == 'normal':
                    self.s4d, self.wiggle = False, False

                elif args[0] == 's4d':
                    self.s4d, self.wiggle = True, False
                    self.minLvx = self.minLvxRoad
                
                elif args[0] == 's4dirt':
                    self.s4d, self.wiggle = True, False
                    self.minLvx = self.minLvxDirt

                elif args[0] == 'wiggle':
                    self.s4d, self.wiggle = False, True

                elif args[0] == 'wiggledirt':
                    self.s4d, self.wiggle = True, True
                    self.minLvx = self.minLvxDirt

                else:
                    iface.log('[Railgun] Invalid mode', 'warning')
                    return
                
                iface.log(f'[Railgun] sdmode set to {args[0]}', 'success')

            else: iface.log('[Railgun] Usage: sdmode <normal, s4d, s4dirt, wiggle or wiggledirt>', 'warning')

    def on_simulation_begin(self, iface: TMInterface):
        try:
            print(f'[Railgun] Simulating from {self.time_from} to {self.time_to} trying to sd {self._direction}')
        except:
            iface.log('[Railgun] Usage: time_from-time_to sd <direction>.', 'error')
            iface.close()
            print('[Railgun] Closed due to exception, use the sd command to change timerange and direction.')
            return

        iface.remove_state_validation()

        self.input_time = self.time_from
        self.rewinding = False
        self.direction = self._direction
        self.countersteered = False

        self.resetSeek()
        self.resetWiggleTimer()
        self.stageReset()

        self.input_ops = {self.getFirstInput, self.getInputTimeState}
        if self.s4d: self.input_ops.add(self.s4dExec)

        self.step_ops = set()
        if self.wiggle: self.step_ops.add(self.wiggleExec)

        self.time_limit = self.time_to + self.seek + 10
        iface.set_simulation_time_limit(self.time_limit - 10010) # temporary manual offset

    def on_simulation_step(self, iface: TMInterface, _time: int):
        if _time >= self.time_limit: return

        elif _time == self.input_time + self.seek:
            self.addVelocityData((self.getEvalVelocity(iface.get_simulation_state()), self.steer))
            return iface.rewind_to_state(self.step)

        elif _time == self.input_time:
            [fn(iface) for fn in self.ordered_input_ops if fn in self.input_ops and not self.rewinding]
            if self.rewinding:
                self.rewinding = False
                return

            else: self.steer = self.steering.pop()

        elif _time == self.input_time - 10:
            self.step = iface.get_simulation_state()
            return

        if _time >= self.input_time: iface.set_input_state(sim_clear_buffer=False, steer=self.steer)

    def on_simulation_end(self, iface: TMInterface, result: int):
        print('[Railgun] Saving steering inputs to sd_railgun.txt...')
        self.writeSteerToFile()

    def on_deregistered(self, iface: TMInterface):
        print('[Railgun] Attempting to back up most recent inputs to sd_railgun.txt...')
        self.writeSteerToFile()

    # Steering calculation
    def addVelocityData(self, pair: tuple):
        len_steering = len(self.steering)
        self.v_data[len(self.v_data) - len_steering - 1] = pair
        if not len_steering:
            if self.staging[self.stage - 1][0]: return self.stageSetup()

            self.setBest()
            speeds, steer_values = [s[0] for s in self.v_data], [s[1] for s in self.v_data]
            offset = (self.best == min(steer_values)) - (self.best == max(steer_values))
            if offset and min(speeds) != max(speeds) and abs(self.best) < 0xfff1:
                s0, s1 = self.offsetToBest(offset)
                self.v_data = [self.v_data[0], self.v_data[steer_values.index(s1)], (0, 0)]
                return self.steering.add(s0)

            self.input_ops.add(self.nextStep)

    staging = tuple([(bool(i), 2 ** i, (2 ** i * 7 // 2) if i else 16) for i in (13, 11, 9, 7, 5, 0)])
    def stageReset(self):
        self.stage = 0
        self.resetVelocityData(0x9000)
        self.stageSetup()

    def stageSetup(self):
        self.setBest()

        step, offset = self.staging[self.stage][1:]
        start, stop = self.offsetToBest(offset)
        self.steering = {i for i in range(start, stop + 1, step) if abs(i) <= 0x10000}
        self.steering.discard(self.best)
        self.v_data = [self.v_data[0]] + [(0, 0)] * len(self.steering)

        self.stage += 1

    def resetVelocityData(self, steer: int):
        self.v_data = [(0, steer * self.direction)] * 9

    def setBest(self):
        self.v_data.sort(reverse=True)
        self.best: int = self.v_data[0][1]

    def offsetToBest(self, offset: int):
        return self.best - offset, self.best + offset

    # General purpose
    def stateToLocalVelocity(self, state: SimStateData, idx: int):
        return np.sum([state.velocity[i] * state.rotation_matrix[i][idx] for i in range(3)])

    wheels = tuple([(SIMULATION_WHEELS_SIZE // 4) * i for i in range(4)])
    def getEvalVelocity(self, state: SimStateData):
        not_all_wheels_on_ground = not all(
            [
                unpack('i', state.simulation_wheels[i+292:i+296])[0]
                for i in self.wheels
            ]
        )
        return np.linalg.norm(
            (
                self.stateToLocalVelocity(state, 0),
                self.stateToLocalVelocity(state, 1) * not_all_wheels_on_ground,
                self.stateToLocalVelocity(state, 2)
            )
        )

    def resetSeek(self):
        self.seek = 120

    def resetWiggleTimer(self):
        self.wiggle_timer = 300

    # Input time event handling functions
    def getFirstInput(self, iface: TMInterface):
        self.input_ops.remove(self.getFirstInput)
        self.inputs = [iface.get_simulation_state().input_steer]

    def getInputTimeState(self, iface: TMInterface):
        self.input_ops.remove(self.getInputTimeState)
        state = iface.get_simulation_state()
        self.velocity = np.linalg.norm(state.velocity)
        # Re-use state
        if self.s4d:
            lvx = self.stateToLocalVelocity(state, 0)
            min_lvx = self.minLvx(self.velocity * 3.6)
            self.isUndersteering = lambda: lvx * self.direction < min_lvx

    def s4dExec(self, iface: TMInterface):
        if self.isUndersteering():
            self.resetVelocityData(0x10000)
            self.nextStep(iface)

        else:
            self.input_ops.remove(self.s4dExec)
            self.input_ops.add(self.s4dReset)
            self.seek = 130
            self.seek_reset_time = self.input_time + 60
            self.stageReset()

    def s4dReset(self, iface: TMInterface):
        if self.input_time == self.seek_reset_time:
            self.input_ops.remove(self.s4dReset)
            self.resetSeek()

    # Time step handling functions
    def nextStep(self, iface: TMInterface):
        self.input_ops.discard(self.nextStep)
        self.rewinding = True
        if self.best * self.direction < 0 and not self.countersteered: self.toggleCountersteer()

        else:
            self.setBest()
            iface.set_input_state(sim_clear_buffer=False, steer=self.best)
            self.inputs.append(self.best)
            print(f'{self.input_time} steer {self.best} -> {self.velocity * 3.6} km/h')
            self.input_ops.add(self.getInputTimeState)
            self.input_time += 10
            [fn() for fn in self.ordered_step_ops if fn in self.step_ops]

        self.stageReset()
        iface.rewind_to_state(self.step)

    def toggleCountersteer(self):
        self.changeDirection()
        self.countersteered ^= True
        self.step_ops.discard(self.toggleCountersteer)
        if self.countersteered: self.step_ops.add(self.toggleCountersteer)

    def wiggleExec(self):
        self.wiggle_timer -= 10
        if self.velocity < np.linalg.norm(self.step.velocity): self.resetWiggleTimer()

        elif not self.wiggle_timer:
            self.resetWiggleTimer()
            self.changeDirection()
            if self.s4d: self.input_ops.add(self.s4dExec)

    def changeDirection(self):
        self.direction *= -1

    # Writing string of steering commands to file
    def writeSteerToFile(self):
        msg = 'success!'
        try:
            with open('sd_railgun.txt', 'w') as f:
                f.writelines(
                    [
                        f'{self.generateCmdTime(t[0])} steer {t[1]}\n' for t in
                        enumerate(self.inputs[1:]) if t[1] != self.inputs[t[0]]
                    ]
                )
        except:
            msg = 'failed.'
        finally:
            print(f'[Railgun] Input write {msg}')

    def generateDecimal(self, tick: int):
        t = self.time_from // 10 + tick
        h, m, s, c = t//360000, t//6000%60, t//100%60, t%100

        c = f'{c / 100}'.removeprefix('0')
        s = f'{s}'
        m = f'{m}:' if h or m else ''
        h = f'{h}:' if h else ''

        return h + m + s + c

if __name__ == '__main__':
    try:
        server_id = int(
            input('[Railgun] Enter the TMInterface instance ID you would like to connect to...\n')
        )
        server_name = f'TMInterface{server_id * (server_id > 0)}'
    except:
        server_name = 'TMInterface0'
    finally:
        print(f'[Railgun] Connecting to {server_name}...')
        run_client(Railgun(), server_name)
        print(f'[Railgun] Deregistered from {server_name}')
