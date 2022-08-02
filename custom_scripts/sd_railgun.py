# sd_railgun; sd script by SaiMoen

import numpy as np
from tminterface.client import Client, run_client
from tminterface.interface import TMInterface

from struct import unpack
from tminterface.constants import SIMULATION_WHEELS_SIZE

USE_DECIMAL_NOTATION = False # set to True for decimal notation, False for milliseconds

class MainClient(Client):
    def __init__(self):
        # Default config
        super().__init__()
        self.time_from, self.time_to = -1, -1
        self.default_seek, self.max_wiggle_timer = 120, 300
        self.s4d, self.wiggle = False, False

        self.minLvxRoad = lambda speed: 4 - 0.5 * ((speed > 401) * ((speed - 401) / 100))
        self.minLvxDirt = lambda speed: 0.25 + (speed > 235) * 0.001 * (306 - speed)
        self.minLvx = self.minLvxRoad

        self.ordered_input_ops = (self.getFirstInput, self.getInputTimeState, self.s4dExec, self.s4dReset, self.atInputTime)
        self.ordered_step_ops = (self.setupNextStep, self.toggleCountersteer, self.wiggleExec)

        # set time notation function pointer
        generateMS = lambda tick: self.time_from + tick * 10
        self.generateCmdTime = self.generateDecimal if USE_DECIMAL_NOTATION else generateMS

        # Global to Local velocity vector transpose
        self.stateToLocalVelocity = lambda state, idx: np.sum(
            [state.velocity[i] * state.rotation_matrix[i][idx] for i in range(3)]
        )

        # Evaluation velocity calculation
        notAllWheelsOnGround = lambda state: not all(
            [
                unpack('i', state.simulation_wheels[i+292:i+296])[0]
                for i in [(SIMULATION_WHEELS_SIZE // 4) * i for i in range(4)]
            ]
        )
        self.getEvalVelocity = lambda state: np.linalg.norm(
            (
                self.stateToLocalVelocity(state, 0),
                self.stateToLocalVelocity(state, 1) * notAllWheelsOnGround(state),
                self.stateToLocalVelocity(state, 2)
            )
        )

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
                self.direction = int(args[0] == 'right') - int(args[0] == 'left')
                self.time_from, self.time_to = time_from, time_to
                iface.log('[Railgun] sd settings changed successfully!', 'success')

            else:
                iface.log('[Railgun] Usage: time_from-time_to sd <direction>', 'warning')

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

            else:
                iface.log('[Railgun] Usage: sdmode <normal, s4d, s4dirt, wiggle or wiggledirt>', 'warning')

    def on_simulation_begin(self, iface: TMInterface):
        # Close because otherwise we get an exception and close anyway
        if self.time_to == -1 or self.time_from == -1:
            iface.log('[Railgun] Usage: time_from-time_to sd', 'error')
            iface.log('[Railgun] (closing to prevent exception): You forgot to set a time range', 'error')
            iface.close()
            print('[Railgun] Closed due to exception, use the sd command to change timerange and direction.')
            return

        # Required to properly change inputs
        self.time_limit = self.time_to + self.default_seek + 10
        iface.remove_state_validation()
        iface.set_simulation_time_limit(self.time_limit - 10010) # temporary manual offset

        # Variable config
        self.input_time = self.time_from
        self.countersteered, self.rewinding = False, False
        self.railgun = steerPredictor(self.direction)
        
        self.seek = self.default_seek
        self.seek_reset_time = -1
        self.wiggle_timer = self.max_wiggle_timer

        self.input_ops = {self.getFirstInput, self.getInputTimeState, self.atInputTime}
        if self.s4d:
            self.input_ops.add(self.s4dExec)

        self.step_ops = {self.setupNextStep}
        if self.wiggle:
            self.step_ops.add(self.wiggleExec)

    def on_simulation_step(self, iface: TMInterface, _time: int):
        if _time >= self.time_limit:
            return

        elif _time == self.input_time + self.seek:
            self.railgun + (self.getEvalVelocity(iface.get_simulation_state()), self.steer)
            iface.rewind_to_state(self.step)
            return

        elif _time == self.input_time:
            [fn(iface) for fn in self.ordered_input_ops if fn in self.input_ops and not self.rewinding]

        elif _time == self.input_time - 10:
            self.step = iface.get_simulation_state()
            return

        if self.rewinding:
            self.rewinding = False
            return

        elif _time >= self.input_time:
            iface.set_input_state(sim_clear_buffer=False, steer=self.steer)

    def on_simulation_end(self, iface: TMInterface, result: int):
        print('[Railgun] Saving steering inputs to sd_railgun.txt...')
        self.writeSteerToFile()

    def on_deregistered(self, iface: TMInterface):
        print('[Railgun] Attempting to back up most recent inputs to sd_railgun.txt...')
        self.writeSteerToFile()

    # Input time event handling functions
    def getFirstInput(self, iface: TMInterface):
        self.input_ops.remove(self.getFirstInput)
        self.inputs = [iface.get_simulation_state().input_steer]

    def getInputTimeState(self, iface: TMInterface):
        self.input_ops.remove(self.getInputTimeState)
        state = iface.get_simulation_state()
        self.velocity = np.linalg.norm(state.velocity)

        # Set conditions
        self.isDecelerating = self.velocity < np.linalg.norm(self.step.velocity)
        lvx = self.stateToLocalVelocity(state, 0)
        min_lvx = self.minLvx(self.velocity * 3.6)
        self.isUndersteering = lambda: lvx * self.railgun.direction < min_lvx

    def s4dExec(self, iface: TMInterface):
        if self.isUndersteering():
            self.railgun.fullSteer()
            self.nextStep(iface)

        else:
            self.input_ops.remove(self.s4dExec)
            self.input_ops.add(self.s4dReset)
            self.seek = 130
            self.seek_reset_time = self.input_time + 60

    def s4dReset(self, iface: TMInterface):
        if self.input_time == self.seek_reset_time:
            self.input_ops.remove(self.s4dReset)
            self.seek = self.default_seek

    def atInputTime(self, iface: TMInterface):
        self.steer: int = self.railgun.iter()
        if self.railgun.isLastIteration():
            self.nextStep(iface)

    # Next iteration handling functions
    def nextStep(self, iface: TMInterface):
        self.rewinding = True
        if self.railgun.isCountersteering() and not self.countersteered:
            self.toggleCountersteer(iface)

        else:
            [fn(iface) for fn in self.ordered_step_ops if fn in self.step_ops]

        self.railgun.reset()
        iface.rewind_to_state(self.step)

    def setupNextStep(self, iface: TMInterface):
        iface.set_input_state(sim_clear_buffer=False, steer=self.railgun.best)
        self.inputs.append(self.railgun.best)
        print(f'{self.input_time} steer {self.railgun.best} -> {self.velocity * 3.6} km/h')
        self.input_ops.add(self.getInputTimeState)
        self.input_time += 10

    def toggleCountersteer(self, iface: TMInterface):
        self.railgun.changeDirection()
        self.countersteered = not self.countersteered
        self.step_ops.discard(self.toggleCountersteer)
        if self.countersteered:
            self.step_ops.add(self.toggleCountersteer)

    def wiggleExec(self, iface: TMInterface):
        self.wiggle_timer -= 10
        if self.isDecelerating:
            self.wiggle_timer = self.max_wiggle_timer

        elif not self.wiggle_timer:
            self.wiggle_timer = self.max_wiggle_timer
            self.railgun.changeDirection()
            if self.s4d:
                self.input_ops.add(self.s4dExec)

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

class steerPredictor:
    def __init__(self, direction: int):
        self.direction: int = direction
        self.i: int = 0
        self.max_i: int = 85
        self.vdata: list[tuple] = [(0, 0)] * self.max_i
        self.interval: tuple[int] = (4096, 512, 64, 8, 1, 0)
        self.offset: tuple[int] = (16, 25, 42, 59, 76, 85)

        self.isLastIteration = lambda: self.i == self.max_i
        self.isNewSteer = lambda: self.steer not in [v[1] for v in self.vdata if v != (0, 0)]
        self.isCountersteering = lambda: self.best * self.direction < 0
        self.calculateSteer = lambda: self.base + (
            self.interval[self.idx] * (self.offset[self.idx] - self.i) * self.direction
        )

    def reset(self):
        self.i: int = 0
        self.vdata: list[tuple] = [(0, 0)] * self.max_i

    def __add__(self, pair: tuple):
        self.vdata[self.i] = pair
        self.i += 1

    def iter(self):
        self.vdata.sort(reverse=True)
        self.best: int = self.vdata[0][1]
        self.idx: int = self.i // 17
        if self.i == self.idx * 17:
            self.base = self.best

        self.steer: int = self.calculateSteer()
        if abs(self.steer) > 65536:
            self.steer = 65536 * self.direction

        # Return good steer
        if self.isNewSteer() or self.isLastIteration():
            return self.steer

        # Otherwise continue iterating
        self.i += 1
        return self.iter()

    def changeDirection(self):
        self.direction *= -1

    def fullSteer(self):
        self.best = 65536 * self.direction

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
        run_client(MainClient(), server_name)
        print(f'[Railgun] Deregistered from {server_name}')
