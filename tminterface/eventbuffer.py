import tminterface.util as util
from tminterface.constants import ANALOG_ACCELERATE_NAME, ANALOG_STEER_NAME, BINARY_ACCELERATE_NAME, BINARY_BRAKE_NAME, BINARY_HORN_NAME, BINARY_LEFT_NAME, BINARY_RACE_FINISH_NAME, BINARY_RACE_START_NAME, BINARY_RESPAWN_NAME, BINARY_RIGHT_NAME
from typing import Union
from math import ceil
from bytefield import ByteStruct, IntegerField


class Event(ByteStruct):
    """
    The Event class represents a game event (or input) with its respective time.

    Such event is stored in 8 bytes internally by the game. The first 4 bytes is the time
    of the event. This time is a stored time, which means it is offset by 100000ms.

    The last 4 bytes contain the event data. This data contains the actual event type
    (e.g. whether it was acceleration, braking, steering etc.) and the value of the event.

    The event type is 1 byte long and it signifes an index into an array of available event types.
    This array is variable based on the information contained within the replay file. As such,
    it is required to get index of the desired event type dynamically. You can easily get/set this property
    through the name_index accessors.

    The event value depends on the event type and is 3 bytes long. If the event type is binary
    (e.g. accelerate, brake, steer left), the value can be either 0 or 1. For managing this value type,
    use the binary_value getter/setter.

    If the event type is analog, the value is stored in a custom format. You can convert between this
    format and the format TMInterface uses by using util.data_to_analog_value and utils.analog_value_to_data.
    You can avoid using these functions simply by using the analog_value getter/setter.

    Args:
        time (int): the stored time of the event

    Attributes:
        time (int): the stored time of the event
        data (int): the final data that is written into game's memory
    """
    time        = IntegerField(offset=0)
    input_data  = IntegerField()

    def __init__(self, *args, **kwargs) -> None:
        if len(args) == 1 and isinstance(args[0], int):
            super().__init__(**kwargs)
            self.time = args[0]
        elif len(args) == 2 and isinstance(args[0], int) and isinstance(args[1], int):
            super().__init__(**kwargs)
            self.time = args[0]
            self.input_data = args[1]
        else:
            super().__init__(*args, **kwargs)

    @property
    def name_index(self) -> int:
        return self.input_data >> 24

    @name_index.setter
    def name_index(self, index: int):
        self.input_data = self.input_data & 0xFFFFFF
        self.input_data = self.input_data | (index << 24)

    @property
    def binary_value(self) -> bool:
        return bool(self.input_data & 0xFFFFFF)

    @binary_value.setter
    def binary_value(self, value: bool):
        self.input_data = self.input_data & 0xFF000000 | int(value)

    @property
    def analog_value(self) -> int:
        return util.data_to_analog_value(self.input_data & 0xFFFFFF)

    @analog_value.setter
    def analog_value(self, value: int):
        self.input_data = self.input_data & 0xFF000000 | (util.analog_value_to_data(value) & 0xFFFFFF)


class EventBufferData(object):
    """
    The internal event buffer used to hold player inputs in run or simulation mode.

    While simulating a race, the game loads the inputs from a replay file
    into an internal buffer and begins to apply "events" (inputs) from this
    buffer. The buffer itself consists of 8 byte values, the first 4 bytes
    is used for the event time and the last 4 is used for event data.
    See the Event class for more information.

    The event time is so called a "stored" time. The stored time is
    defined as 100000 + race time. The stored time is saved in the
    replay file and is also used in the internal buffer itself.

    The buffer itself is stored in *decreasing* order. That means that the event
    at index 0 in the list is the last one simulated in the race. The start and end
    of the race is marked by special "_FakeIsRaceRunning" and "_FakeFinishLine" events.
    These events mark the start and finish of the race. Note that without the presence
    of "_FakeIsRaceRunning" event, the race will not start at all. This event has a
    constant stored time of 100000.

    Before the starting event, a "Respawn" event can be generated by the game, this
    event can also be saved in the replay file itself. The very first input that can be applied
    by the player happens at stored time of 100010.

    Arguments:
        events_duration (int): the duration of the events, equalling the finish time, mostly ignored and does not need to be set

    Attributes:
        events_duration (int): the duration of the events
        control_names (list): the list of supported event types by this buffer
        events (list): the list of events held by this buffer
    """
    def __init__(self, events_duration: int):
        self.events_duration = events_duration
        self.control_names = []
        self.events = []

    def copy(self):
        """
        Copies the event buffer with all its events.

        Returns:
            a deep copy of the original event buffer
        """
        cpy = EventBufferData(self.events_duration)
        cpy.control_names = self.control_names[:]
        cpy.events = [Event(ev.time, ev.data) for ev in self.events]
        return cpy

    def clear(self):
        """
        Removes all events in the current event buffer, leaving
        the race running event in the buffer.

        A race running event should always be present in the buffer, to
        make the game start the race.
        """
        self.events = []
        self.add(-10, '_FakeIsRaceRunning', True)
    
    def sort(self):
        """
        Sorts the event buffer by time in decreasing order.

        This is the order that events are stored internally by the game.
        Calling this is not needed, if you are calling set_event_buffer. 
        The server will always take care of properly sorting the events.
        """
        self.events = sorted(self.events, key=lambda ev: ev.time, reverse=True)

    def add(self, time: int, event_name: str, value: Union[int, bool]):
        """
        Adds an event to the event buffer.

        This is a wrapper function that provides easy API for adding new events.
        Depending on the event_name parameter, the method will interpret the value
        in different ways. If the event is an analog event, the value passed should
        be in the range of [-65536, 65536] where negative values represent left steering
        and postive, right steering.
        
        If the event is binary, the value should be False for disabling the input and 
        True for enabling it.

        The time parameter is zero based, where 0 is the first human input that can be injected.
        Internally, 0 translates to stored time 100010, which is the first simulation step
        after the countdown.

        Args:
            time (int): zero based timestamp when the input is injected
            event_name (str): the event name that specifies the input type
            value (Union[int, bool]): the value for the event, based on the event type
        """
        try:
            index = self.control_names.index(event_name)
        except ValueError:
            raise ValueError(f'Event name "{event_name}" does not exist in this event buffer')

        ev = Event(time + 100010)
        ev.name_index = index
        if event_name == ANALOG_ACCELERATE_NAME or event_name == ANALOG_STEER_NAME:
            ev.analog_value = value
        else:
            ev.binary_value = value

        self.events.append(ev)

    def find(self, **kwargs):
        """
        Finds matching events according to keyword arguments.

        Any unspecified parameter will be skipped in the search and will not be compared.
        You may use this method to filter events based on time, event type and value. 

        Find all analog steering events with value -65536:

            matching = event_buffer.find(event_name=structs.ANALOG_STEER_NAME, value=-65536)

        Find all events that happened at input time 0:

            matching = event_buffer.find(time=0)

        Find the finish line event:

            matching = event_buffer.find(event_name=structs.BINARY_RACE_FINISH_NAME, value=True)

        Calling this method without any keyword arguments will return all events in the buffer.

        Args:
            **kwargs: the keyword arguments
        
        Keyword Args:
            event_name (str): match events with this event type
            time (int): match events with this time (zero based)
            value (Union[int, bool]): match events with this value, bool if the event type is binary, int if analog,
                this parameter can only be filtered if event_name is provided

        Returns:
            list: the events that matched the query
        """
        index = -1
        if 'event_name' in kwargs:
            try:
                index = self.control_names.index(kwargs['event_name'])
            except ValueError:
                raise ValueError(f'Event name "{kwargs["event_name"]}" does not exist in this event buffer')

        has_value = 'value' in kwargs
        has_time = 'time' in kwargs

        matched = []
        for ev in reversed(self.events):
            if has_time and ev.time - 100010 != kwargs['time']:
                continue

            if index >= 0:
                if ev.name_index != index:
                    continue

                if has_value:
                    if kwargs['event_name'] == ANALOG_STEER_NAME or kwargs['event_name'] == ANALOG_ACCELERATE_NAME:
                        if ev.analog_value != kwargs['value']:
                            continue
                    else:
                        if ev.binary_value != kwargs['value']:
                            continue

            matched.append(ev)

        return matched

    def to_commands_str(self, all_events=False):
        """
        Converts event buffer events and constructs a string consisting
        of commands compatible with TMInterface's script syntax.

        By default, only events that happened in the race
        (signified by the race running and finish events) will be converted
        and appended to the final string. If you want to convert all commands
        that are available in the buffer, call this method with all_events set to True.

        Args:
            all_events (bool): whether to convert all commands available in the buffer
        
        Returns:
            str: the string containing commands compatible with TMInterface's script syntax
        """
        ACTION_MAPPINGS = {
            BINARY_ACCELERATE_NAME: 'up',
            BINARY_BRAKE_NAME: 'down',
            BINARY_LEFT_NAME: 'left',
            BINARY_RIGHT_NAME: 'right',
            BINARY_RESPAWN_NAME: 'enter',
            BINARY_HORN_NAME: 'horn'
        }

        sorted_events = sorted(self.events, key=lambda ev: ev.time)

        commands = ''
        try:
            start_events = self.find(event_name=BINARY_RACE_START_NAME)
            if start_events:
                start_time = start_events[0].time
            else:
                start_time = 100000
        except ValueError:
            start_time = 100000

        for ev in sorted_events:
            event_name = self.control_names[ev.name_index]
            if not all_events:
                if ev.time < start_time:
                    continue

                if event_name == BINARY_RACE_FINISH_NAME:
                    break

            if event_name in [BINARY_RESPAWN_NAME, BINARY_HORN_NAME] and not ev.binary_value:
                continue

            time = int(ceil((ev.time - start_time - 10) / 10) * 10)
            if event_name in ACTION_MAPPINGS.keys():
                if ev.binary_value:
                    commands += f'{time} press {ACTION_MAPPINGS[event_name]}\n'
                else:
                    commands += f'{time} rel {ACTION_MAPPINGS[event_name]}\n'
                
            elif event_name == ANALOG_ACCELERATE_NAME:
                commands += f'{time} gas {ev.analog_value}\n'
            elif event_name == ANALOG_STEER_NAME:
                commands += f'{time} steer {ev.analog_value}\n'
        
        return commands
