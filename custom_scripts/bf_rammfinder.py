# bf_rammfinder; ramm bruteforce script by SaiMoen

from tminterface.client import Client, run_client
from tminterface.interface import TMInterface
from tminterface.structs import BFEvaluationDecision, BFEvaluationInfo, BFEvaluationResponse, BFPhase

TAG = "[RammFinder] "
def getServerName():
    try:
        server_id = int(
            input(TAG + "Enter the TMInterface instance ID you would like to connect to...\n")
        )
        server_id *= (server_id > 0)
    except:
        server_id = 0
    finally:
        return f"TMInterface{server_id}"

class RammFinder(Client):
    """Main Client Implementation."""
    def __init__(self):
        self.time_from = -1
        self.time_to = -1
        self.lift = set()
        self.ground = set()

    def on_registered(self, iface: TMInterface):
        print(TAG + f"Registered to {iface.server_name}")
        iface.execute_command("set controller bruteforce")
        iface.register_custom_command("ramm")
        iface.log(TAG + "Use the ramm command to set an evaluation timerange: eval_start-eval_end ramm")
        iface.register_custom_command("contact")
        iface.log(TAG + "Use the contact command to force a wheel to be raised or grounded during evaluation time.")
        iface.log(TAG + "contact <wheel> <state>; state 0 means no contact, 1 means contact, nothing means reset.")
        iface.log(TAG + "Allowed wheels: fl, fr, bl, br. (front/back left/right).")

    def on_custom_command(self, iface: TMInterface, time_from: int, time_to: int, command: str, args: list):
        msg = ("Invalid command", "error")
        if command == "ramm":
            msg = self.on_ramm(time_from, time_to)
        elif command == "contact":
            msg = self.on_contact(args)
        iface.log(TAG + msg[0], msg[1])

    def on_ramm(self, time_from: int, time_to: int):
        self.time_from = time_from
        self.time_to = time_to
        return "Evaluation timerange changed correctly!", "success"

    def on_contact(self, args: list):
        invalid_wheel = ("Pick a wheel: fl, fr, bl, br.", "warning")
        if not args:
            return invalid_wheel
        elif args[0] in ("fl", "fr", "bl", "br"):
            idx = (args[0] == "fr") + 2 * (args[0] == "br") + 3 * (args[0] == "bl")
            if len(args) == 1:
                self.lift.discard(idx)
                self.ground.discard(idx)
                return args[0] + " deactivated successfully", "success"
            elif len(args) == 2:
                if args[1] == "0":
                    self.ground.discard(idx)
                    self.lift.add(idx)
                    return args[0] + " added to lifted wheels successfully!", "success"
                elif args[1] == "1":
                    self.lift.discard(idx)
                    self.ground.add(idx)
                    return args[0] + " added to grounded wheels successfully!", "success"
            return "Invalid argument!", "warning"
        return invalid_wheel

    def on_simulation_begin(self, iface: TMInterface):
        if self.time_from == -1 or self.time_to == -1:
            print(TAG + "Do not forget to set a timerange!")
            iface.close()
            return
        self.time = -1
        self.current = -1
        self.best = -1

    def on_bruteforce_evaluate(self, iface: TMInterface, info: BFEvaluationInfo) -> BFEvaluationResponse:
        self.current_time = info.time
        self.phase = info.phase

        response = BFEvaluationResponse()
        response.decision = BFEvaluationDecision.DO_NOTHING

        if self.phase == BFPhase.INITIAL:
            if self.is_eval_time() and self.is_better(iface):
                self.best = self.current
                self.time = self.current_time

            if self.is_max_time():
                print(f"base at {self.time}: {self.best}")

        elif self.phase == BFPhase.SEARCH:
            if self.is_eval_time() and self.is_better(iface):
                response.decision = BFEvaluationDecision.ACCEPT

            if self.is_past_eval_time():
                if response.decision != BFEvaluationDecision.ACCEPT:
                    response.decision = BFEvaluationDecision.REJECT

        return response

    def is_better(self, iface: TMInterface):
        state = iface.get_simulation_state()
        for i in self.ground:
            if not state.simulation_wheels[i].real_time_state.has_ground_contact:
                return False

        for i in self.lift:
            if state.simulation_wheels[i].real_time_state.has_ground_contact:
                return False

        self.current = state.position[1]
        return self.current > self.best

    def is_eval_time(self):
        return self.time_from <= self.current_time <= self.time_to

    def is_past_eval_time(self):
        return self.time_to <= self.current_time

    def is_max_time(self):
        return self.time_to == self.current_time

    def main(self, server_name = getServerName()):
        print(TAG + f"Connecting to {server_name}...")
        run_client(self, server_name)
        print(TAG + f"Deregistered from {server_name}")

if __name__ == "__main__":
    RammFinder().main()
