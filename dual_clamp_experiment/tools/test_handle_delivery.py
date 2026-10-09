"""执行正式handle内的生产ST Actions；模拟NC回退，不连接ADS或硬件。"""
import argparse
import csv
import hashlib
import json
import math
import re
import types
import xml.etree.ElementTree as ET
from pathlib import Path

from plc_st_parser import ST, Timer

PLC = Path(r"D:\Work_files\Vessel intervention Robot\260316\250902\250902\Untitled2")


class ActionReturn(Exception):
    pass


def action_return():
    raise ActionReturn()


def normalize(text):
    text = re.sub(r"\(\*.*?\*\)", "", text, flags=re.S)
    text = re.sub(r"(?:WORD|UDINT)#(\d+)", r"\1", text)
    return text.replace("1.0E100", "1" + "0" * 100).replace("RETURN;", "action_return();")


def command():
    return types.SimpleNamespace(Req=False, Busy=False, Done=False, Error=False, ErrorId=0,
                                 TargetAbs=0., Velocity=0., Acc=0., Dec=0., Jerk=0.)


def declarations(text):
    result = {}
    text = re.sub(r"//[^\n]*", "", text)
    for match in re.finditer(r"(?m)^\s*([\w,\s]+?)\s*(?:AT\s+%[IQ]\*)?\s*:\s*([^;]+);", text):
        names, spec = match.groups()
        names = names.replace("VAR_GLOBAL", "").replace("VAR", "").strip()
        kind, _, initial = spec.partition(":=")
        kind = kind.strip()
        def make(kind=kind):
            if kind == "TON":
                return Timer()
            if kind == "ST_AxisPlannedReturnCmd":
                return command()
            if kind.startswith("ARRAY"):
                array = re.match(r"ARRAY\s*\[(\d+)\.\.(\d+)\]\s+OF\s+(.+)", kind)
                return [make(array[3].strip()) for _ in range(int(array[2]) + 1)]
            return False if kind == "BOOL" else 0. if kind in ("LREAL", "REAL") else 0
        for name in names.split(","):
            name = name.strip()
            if not re.fullmatch(r"\w+", name):
                continue
            value = make()
            if initial.strip():
                try:
                    value = eval(ST.expr(ST(normalize(initial)).tokens), {"_init": 0})
                    if kind.startswith("ARRAY"):
                        lower = int(re.search(r"\[(\d+)", kind)[1])
                        value = [0] * lower + value
                except (SyntaxError, NameError):
                    pass
            result[name] = value
    return result


class Simulation:
    def __init__(self, mode=4, cycles=1, final=7.):
        self.xml = ET.parse(PLC / "POUs/handle.TcPOU")
        self.g = types.SimpleNamespace(**declarations(ET.parse(PLC / "GVLs/G.TcGVL").findtext(".//Declaration")))
        self.env = declarations(self.xml.findtext(".//Declaration"))
        self.env.update(G=self.g, ABS=abs, MIN=min,
                        MAX=max, LIMIT=lambda lower, value, upper: min(upper, max(lower, value)),
                        SQRT=math.sqrt, action_return=action_return)
        for name in ("INT_TO_USINT", "UINT_TO_LREAL", "INT_TO_LREAL", "UDINT_TO_TIME", "UDINT_TO_ULINT", "INT_TO_UINT"):
            self.env[name] = lambda value: value
        self.env["TwinCAT_SystemInfoVarList"] = types.SimpleNamespace(_TaskInfo=[None, types.SimpleNamespace(CycleTime=10000)])
        self.actions = {}
        for action in self.xml.findall(".//POU/Action"):
            self.actions[action.attrib["Name"]] = compile("\n".join(ST(normalize(action.findtext("./Implementation/ST"))).block()),
                                                         action.attrib["Name"], "exec")
        self.g.axis = [types.SimpleNamespace(NcToPlc=types.SimpleNamespace(ActPos=float(80 + axis), ActVelo=0., ActAcc=0.))
                       for axis in range(8)]
        self.g.power_ = [types.SimpleNamespace(Enable=True) for _ in range(8)]
        self.g.power_output = [types.SimpleNamespace(Done=True, Error=False) for _ in range(8)]
        self.g.leftlimit = [0., 100., 0., 0., 0., 500., 700., 0.]
        self.g.init_pos = [0., 30., 9., 0., 0., 0., 30., 13.]
        self.g.refer = [0.] + [self.g.axis[axis].NcToPlc.ActPos-self.g.init_pos[axis] for axis in range(1, 8)]
        self.g.self_check_done = True
        self.g.program_test_mode = mode
        self.g.program_test_cycle_count = cycles
        self.g.program_test_final_forward_distance_mm = final
        self.g.program_test_axis2_angle_deg = 17.
        self.g.program_test_axis7_angle_deg = -12.
        self.g.program_test_release_wait_ms = 4
        self.g.program_test_reclamp_wait_ms = 5
        self.g.program_test_release_lead_ms = 3
        self.g.program_test_reclamp_lead_ms = 2
        self.g.fn_1_value, self.g.ft_1_value, self.g.fn_2_value, self.g.ft_2_value = 113, -31, 207, 61
        self.remaining = [0] * 8
        self.moves = []
        self.phases = []
        self.rows = []
        self.tick()

    def action(self, name):
        try:
            exec(self.actions[name], self.env)
        except ActionReturn:
            pass

    def tick(self, input_abs=None, cycle=None, acknowledge=True):
        if input_abs is not None:
            self.g.refer[1] = input_abs[0] - self.g.init_pos[1]
            self.g.refer[2] = input_abs[1] - self.g.init_pos[2]
            self.g.program_test_handle_cycle = self.g.program_test_cycle_index if cycle is None else cycle
        self.action("ProgramDelivery")
        self.action("ProgramRecord")
        phase = self.g.program_test_phase
        if not self.phases or self.phases[-1] != phase:
            self.phases.append(phase)
        self.rows.append((phase, self.g.program_test_cycle_index, self.g.axis[1].NcToPlc.ActPos,
                          self.g.axis[6].NcToPlc.ActPos, self.g.cylinder1_value, self.g.cylinder2_value,
                          self.g.cylinder3_value, self.g.cylinder4_value))
        for axis in range(1, 8):
            request = self.g.return_cmd[axis]
            if request.Req and not request.Busy and not request.Done:
                self.remaining[axis] = 4
                request.Busy = True
                self.env["return_state"][axis] = 20
                self.moves.append((phase, axis, request.TargetAbs, request.Velocity))
            if phase >= 11 and request.Busy:
                request.Busy = False
                self.remaining[axis] = 0
                self.env["return_state"][axis] = 0
            if self.remaining[axis]:
                self.remaining[axis] -= 1
                if self.remaining[axis] == 0:
                    self.g.axis[axis].NcToPlc.ActPos = request.TargetAbs
                    self.env["return_state"][axis] = 31
                continue
            if request.Busy:
                request.Busy = False
                request.Done = True
                self.env["return_state"][axis] = 0
                continue
            if not request.Req:
                request.Done = False
                request.Error = False
            self.g.axis[axis].NcToPlc.ActPos = self.g.refer[axis] + self.g.init_pos[axis]
        if acknowledge:
            for slot in (1, 2):
                if self.g.experiment_record_block_ready[slot]:
                    self.g.experiment_record_block_ack_sequence[slot] = self.g.experiment_record_block_sequence[slot]

    def until(self, condition, limit=5000):
        for _ in range(limit):
            if condition():
                return
            self.tick()
        raise AssertionError((self.g.program_test_phase, self.env["pd_step"], self.g.program_test_status_error_id))

    def prepare(self):
        self.g.program_test_setup_req = True
        self.until(lambda: self.g.program_test_setup_done)
        self.g.experiment_zero_req = True
        self.until(lambda: self.g.experiment_zero_done)
        assert self.g.experiment_zero_sample_count == 1000
        assert self.g.experiment_zero_fn1 == 113 and self.g.experiment_zero_ft2 == 61

    def start(self):
        self.g.program_test_start_req = True
        self.until(lambda: self.g.program_test_phase == 4)
        self.tick()


def external(cycles):
    sim = Simulation(cycles=cycles)
    sim.prepare()
    sim.start()
    for cycle in range(1, cycles + 1):
        prepare = sim.g.program_test_return_target_abs
        trigger = sim.g.program_test_trigger_target_abs
        anchor5 = sim.g.axis[5].NcToPlc.ActPos
        reference = sim.g.axis[6].NcToPlc.ActPos
        for _ in range(3):
            sim.tick((prepare + 15., 27.))
            assert sim.g.program_test_phase == 4
            assert sim.g.axis[1].NcToPlc.ActPos == prepare
            assert sim.g.axis[6].NcToPlc.ActPos == reference
        for target in (prepare - 8., prepare - 3., prepare - 15.):
            sim.tick((target, -21.))
            assert sim.g.program_test_phase == 4
            assert sim.g.axis[5].NcToPlc.ActPos - anchor5 == target - prepare
            assert sim.g.axis[6].NcToPlc.ActPos - reference == target - prepare
            assert sim.g.axis[7].NcToPlc.ActPos == sim.g.axis[2].NcToPlc.ActPos
        for _ in range(3):
            sim.tick((trigger - 10., 35.))
            if sim.g.program_test_phase == 5:
                break
        assert sim.g.program_test_phase == 5
        held6 = sim.g.axis[6].NcToPlc.ActPos
        for _ in range(200):
            sim.tick((prepare + 100., 200.), cycle=cycle)
            assert sim.g.axis[6].NcToPlc.ActPos == held6
            assert sim.g.axis[5].NcToPlc.ActPos - anchor5 == trigger - prepare
            assert sim.g.cylinder4_value == sim.g.program_test_cylinder4_close_word
            if sim.g.program_test_phase in (4, 10):
                break
        if cycle < cycles:
            sim.tick((trigger, 200.), cycle=cycle)
            assert sim.g.axis[1].NcToPlc.ActPos == prepare
            assert sim.g.program_test_sync_state == 1
        else:
            assert sim.g.program_test_phase == 10
            sim.tick((trigger, 300.))
            assert sim.g.axis[1].NcToPlc.ActPos == prepare
    returns = [move for move in sim.moves if move[0] == 6]
    assert len(returns) == cycles and all(move[1] == 1 for move in returns)
    assert 9 not in sim.phases
    assert sim.g.experiment_record_source_mode == 4
    assert not sim.g.experiment_record_enable
    assert sim.g.program_test_sample_phase[sim.g.program_test_sample_count - 1] == 10
    assert sim.g.program_test_sample_axis5_pos[sim.g.program_test_sample_count - 1] == anchor5 + trigger - prepare
    print(f"PASS external {cycles} cycles: clamp, reverse, trigger once, hold, rebase, rotation, terminal record")
    return sim


def automatic(mode, cycles):
    sim = Simulation(mode=mode, cycles=cycles)
    sim.prepare()
    sim.g.program_test_start_req = True
    sim.until(lambda: sim.g.program_test_phase == 10)
    moving = 1 if mode == 1 else 6
    assert len([move for move in sim.moves if move[0] == 4 and move[1] == moving]) == cycles
    assert len([move for move in sim.moves if move[0] == 6 and move[1] == moving]) == cycles
    assert len([move for move in sim.moves if move[0] == 9 and move[1] == moving]) == 1
    assert sim.g.axis[moving].NcToPlc.ActPos == sim.g.program_test_return_target_abs - 7.
    assert all(move[1] in ({1, 2} if mode == 1 else {5, 6, 7}) for move in sim.moves)
    print(f"PASS automatic mode {mode}, {cycles} cycles: setup axes, forward, return, final segment")


def stop_handoff():
    sim = Simulation()
    body = sim.xml.findtext("./POU/Implementation/ST")
    capture = body.split("IF NOT host_timeout_prev THEN", 1)[1].split("END_IF", 1)[0]
    capture_code = compile("\n".join(ST(normalize(capture)).block()), "host_timeout_capture", "exec")
    sim.env["fb_return_move_abs"] = [types.SimpleNamespace(Busy=False) for _ in range(8)]
    sim.env["fb_axis4_move_velocity"] = types.SimpleNamespace(Busy=False)
    for axis in (1, 2, 6, 7):
        for source in ("state", "command", "block"):
            sim.env["return_state"][axis] = 20 if source == "state" else 0
            sim.g.return_cmd[axis].Busy = source == "command"
            sim.env["fb_return_move_abs"][axis].Busy = source == "block"
            exec(capture_code, sim.env)
            assert sim.env["host_return_stop_active"][axis]
            sim.env["return_state"][axis] = 0
            sim.g.return_cmd[axis].Busy = False
            sim.env["fb_return_move_abs"][axis].Busy = False
        exec(capture_code, sim.env)
        assert not any(sim.env["host_return_stop_active"])
        for destination in (0, 1, 2):
            sim.env["pd_mode"] = 4
            sim.env["pd_state"] = 11
            sim.env["return_state"][axis] = 40
            sim.g.return_cmd[axis].Busy = True
            sim.g.program_test_mode = destination
            sim.action("ProgramDelivery")
            assert sim.g.program_test_mode == 4 and sim.env["pd_mode"] == 4
            assert sim.g.program_test_status_error_id == 0x7102
            sim.env["return_state"][axis] = 0
            sim.g.return_cmd[axis].Busy = False
            sim.g.program_test_mode = destination
            sim.action("ProgramDelivery")
            assert sim.env["pd_mode"] == destination
    print("PASS timeout stop capture axes 1/2/6/7 and mode changes wait for motion handoff")


def selfcheck_handshake():
    sim = Simulation()
    body = ET.parse(PLC / "POUs/SelfCheck.TcPOU").findtext("./POU/Implementation/ST")
    gate = body[body.index("IF G.selfcheck_status <> 2 THEN"):body.index("// selfcheck 期间")]
    code = compile("\n".join(ST(normalize(gate)).block()), "selfcheck_start_gate", "exec")
    sim.g.power_output = [types.SimpleNamespace(Done=True, Error=False) for _ in range(8)]
    sim.g.reset_output = [types.SimpleNamespace(Error=False) for _ in range(8)]
    for axis in sim.g.axis:
        axis.Status = types.SimpleNamespace(Error=False)
    def scan():
        try:
            exec(code, sim.env)
            return True
        except ActionReturn:
            return False
    sim.g.selfcheck_status = 1
    sim.g.selfcheck_start_req = False
    sim.g.host_comm_timeout = True
    assert not scan() and sim.g.selfcheck_status == 1
    sim.g.selfcheck_start_req = True
    sim.g.power_output[6].Done = False
    assert not scan() and sim.g.selfcheck_status == 3 and not sim.g.selfcheck_start_req
    sim.g.power_output[6].Done = True
    assert not scan()
    sim.g.selfcheck_start_req = True
    assert scan() and sim.g.selfcheck_status == 2 and not sim.g.selfcheck_start_req
    assert scan()
    assert "G.self_check_done := TRUE;\n    G.selfcheck_status := 4;" in body
    print("PASS selfcheck waits, consumes request, rejects unpowered start, starts despite pre-handle timeout")


def main():
    global PLC
    parser = argparse.ArgumentParser()
    parser.add_argument("--plc", type=Path, default=PLC)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    PLC = args.plc
    handle_source = (PLC / "POUs/handle.TcPOU").read_text(encoding="utf-8-sig")
    assert "G.program_test_axis6_prepare_from_left_mm -" in handle_source
    assert "G.power_[3].Enable := FALSE" in handle_source and "G.power_[5].Enable := TRUE" in handle_source
    assert "pd_ref[5] := pd_anchor5 + pd_input1 - pd_anchor1" in handle_source
    assert "ABS(G.axis[6].NcToPlc.ActPos - (pd_ref[6] + G.init_pos[6])) <= 0.2" in handle_source
    print("PASS external mode derives axis6 trigger from axis1 motion window")
    selfcheck_handshake()
    stop_handoff()
    for cycles in (1, 3):
        external_sim = external(cycles)
        for mode in (1, 2):
            automatic(mode, cycles)
    for phase in (1, 3, 4, 5, 6, 7):
        sim = Simulation()
        sim.g.program_test_setup_req = True
        if phase > 1:
            sim.until(lambda: sim.g.program_test_setup_done)
            sim.g.experiment_zero_done = True
            sim.start()
            if phase == 3:
                sim = Simulation()
                sim.prepare()
                sim.g.program_test_start_req = True
            elif phase > 4:
                sim.tick((sim.g.program_test_trigger_target_abs, 0.))
                sim.tick((sim.g.program_test_trigger_target_abs, 0.))
        sim.until(lambda: sim.g.program_test_phase == phase)
        sim.g.program_test_abort_req = True
        sim.tick()
        assert sim.g.program_test_phase == 11 and not sim.g.program_test_sample_arm
    assert not any(sim.env["pd_req"])

    # 首次外源递送必须确认PLC当前周期，不能把 cycle=0 留在手柄输入阶段。
    sim = Simulation(mode=4, cycles=1)
    sim.prepare()
    sim.start()
    sim.g.program_test_cycle_index = 1
    sim.g.program_test_phase = 4
    sim.g.program_test_handle_cycle = 0
    sim.tick((sim.g.program_test_trigger_target_abs + 5.0, 0.0))
    assert sim.g.program_test_handle_cycle == 1
    print("PASS first external handle frame acknowledges PLC cycle 1")
    sim = Simulation()
    sim.prepare()
    sim.start()
    sim.env["pd_fault"] = True
    sim.tick()
    assert sim.g.program_test_phase == 12
    sim.env["pd_fault"] = False
    sim.tick()
    assert sim.g.program_test_phase == 12
    sim = Simulation()
    sim.prepare()
    sim.start()
    for _ in range(1200):
        sim.tick(acknowledge=False)
    assert sim.g.experiment_record_overflow
    assert sim.g.program_test_phase in (11, 12)
    expected = "C1E27D77C29AD667B676EEDC91DEADD6D771CE49A20ABB681BC3C3FCB1AF436D"
    assert hashlib.sha256((PLC / "POUs/MAIN.TcPOU").read_bytes()).hexdigest().upper() == expected
    print("PASS abort phases, fault latching, recording overflow and unchanged MAIN")
    if args.output:
        args.output.mkdir(parents=True, exist_ok=True)
        columns = {
            "sample_index": "index", "plc_time_us": "time_us", "phase": "phase",
            "event_sequence": "event_sequence", "cycle_index": "cycle_index", "sync_state": "sync_state",
        }
        for axis in (1, 2, 5, 6, 7):
            for quantity in ("pos", "vel", "acc"):
                name = f"axis{axis}_{quantity}"
                columns[name] = name
        for name in ("cylinder1", "cylinder2", "cylinder3", "cylinder4", "fn1", "ft1", "fn2", "ft2"):
            columns[name] = name
        with (args.output / "plc_trace.csv").open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=columns)
            writer.writeheader()
            for index in range(external_sim.g.program_test_sample_count):
                writer.writerow({name: getattr(external_sim.g, "program_test_sample_" + suffix)[index]
                                 for name, suffix in columns.items()})
        report = dict(production_st_actions_executed=["ProgramDelivery", "ProgramRecord"],
                      plc_directory=str(PLC), motion_blocks="offline_mocks",
                      full_handle_executed=False, hardware_connected=False, twincat_compilation=False)
        (args.output / "plc_tests.json").write_text(json.dumps(report, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
