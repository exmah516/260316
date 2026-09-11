"""离线执行生产ST状态机，运动功能块使用确定性的模拟实现；不连接ADS。"""
import argparse
import csv
import json
import math
import re
import types
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PLC = ROOT / "250902/250902/Untitled2"


class ST:
    """仅解析本状态机所用的ST语句；遇到未支持语法立即失败。"""
    def __init__(self, text):
        text = re.sub(r"//[^\n]*", "", text)
        self.tokens = re.findall(r"T#\d+(?:MS|S)|16#[0-9A-F]+|\d+\.\d+|\d+|[A-Za-z_]\w*|:=|<>|<=|>=|[^\s]", text)
        self.i = self.case = 0

    def take(self):
        token = self.tokens[self.i]
        self.i += 1
        return token

    def peek(self):
        return self.tokens[self.i] if self.i < len(self.tokens) else ""

    def until(self, end):
        values = []
        depth = 0
        while self.peek():
            if self.peek() == end and depth == 0:
                self.take()
                return values
            t = self.take()
            if t in ("(", "["): depth += 1
            if t in (")", "]"): depth -= 1
            values.append(t)
        raise AssertionError(f"Missing {end}")

    @staticmethod
    def expr(tokens):
        mapping = {"AND": "and", "OR": "or", "NOT": "not", "TRUE": "True", "FALSE": "False", "<>": "!=", "=": "==", ":=": "="}
        def convert(t):
            if t.startswith("16#"): return str(int(t[3:], 16))
            if t.startswith("T#"): return str(int(re.search(r"\d+", t)[0]) * (1 if t.endswith("MS") else 1000))
            return mapping.get(t, t)
        return " ".join(convert(t) for t in tokens)

    def block(self, end=(), depth=0):
        out = []
        indent = "    " * depth
        while self.peek() and self.peek() not in end:
            t = self.take()
            if t == "IF":
                out.append(indent + "if " + self.expr(self.until("THEN")) + ":")
                out += self.block(("ELSIF", "ELSE", "END_IF"), depth + 1)
                while self.peek() == "ELSIF":
                    self.take()
                    out.append(indent + "elif " + self.expr(self.until("THEN")) + ":")
                    out += self.block(("ELSIF", "ELSE", "END_IF"), depth + 1)
                if self.peek() == "ELSE":
                    self.take(); out.append(indent + "else:")
                    out += self.block(("END_IF",), depth + 1)
                assert self.take() == "END_IF"
            elif t == "FOR":
                name = self.take()
                assert self.take() == ":="
                start = self.expr(self.until("TO"))
                finish = self.expr(self.until("DO"))
                out.append(indent + f"for {name} in range(int({start}), int({finish}) + 1):")
                out += self.block(("END_FOR",), depth + 1)
                assert self.take() == "END_FOR"
            elif t == "CASE":
                value = self.expr(self.until("OF"))
                cases = []
                level = 0
                begin = self.i
                while not (self.peek() == "END_CASE" and level == 0):
                    a = self.take()
                    if a == "CASE": level += 1
                    if a == "END_CASE": level -= 1
                    if level == 0 and a.isdigit() and self.peek() == ":":
                        cases.append((int(a), self.i - 1))
                finish = self.i
                self.take()
                for n, (label, pos) in enumerate(cases):
                    boundary = cases[n + 1][1] if n + 1 < len(cases) else finish
                    sub = ST("")
                    sub.tokens = self.tokens[pos + 2:boundary]
                    out.append(indent + ("if " if n == 0 else "elif ") + f"({value}) == {label}:")
                    out += sub.block(depth=depth + 1)
                assert cases, f"Empty CASE at {begin}"
            elif t == ";":
                continue
            else:
                statement = [t] + self.until(";")
                nesting = 0
                assignment = None
                for k, token in enumerate(statement):
                    if token in ("(", "["): nesting += 1
                    elif token in (")", "]"): nesting -= 1
                    elif token == ":=" and nesting == 0: assignment = k; break
                if assignment is None:
                    out.append(indent + self.expr(statement))
                else:
                    out.append(indent + self.expr(statement[:assignment]) + " = " + self.expr(statement[assignment + 1:]))
        return out or [indent + "pass"]


class Axis:
    def __init__(self, index, position):
        self.index = index
        self.NcToPlc = types.SimpleNamespace(ActPos=position, ActVelo=0., ActAcc=0.)
        self.master = None
        self.move = None

    def __call__(self): pass


class Move:
    def __init__(self):
        self.Done = self.Error = self.CommandAborted = self.previous = False
        self.ErrorID = 0

    def __call__(self, Axis, Execute, Position=0., **kw):
        if Execute and not self.previous:
            assert Axis.master is None, "Independent move issued to coupled slave"
            self.Done = False
            Axis.move = [self, Axis.NcToPlc.ActPos, Position, 0]
            Axis.NcToPlc.ActVelo = (Position - Axis.NcToPlc.ActPos) * 250.
        if not Execute and self.Done: self.Done = False
        self.previous = Execute


class Stop:
    def __call__(self, Axis, Execute, **kw):
        if Execute:
            Axis.move = None
            Axis.master = None
            Axis.NcToPlc.ActVelo = Axis.NcToPlc.ActAcc = 0.


class GearIn:
    def __init__(self):
        self.InGear = self.Error = self.CommandAborted = self.fail = False
        self.ErrorID = 0

    def __call__(self, Master, Slave, Execute, RatioNumerator, RatioDenominator, **kw):
        if Execute:
            if self.fail:
                self.Error = True; self.ErrorID = 7001
            else:
                assert abs(Master.NcToPlc.ActVelo) <= .01 and abs(Slave.NcToPlc.ActVelo) <= .01
                assert RatioNumerator == RatioDenominator == 1
                Slave.master = Master
                self.InGear = True
        else: self.InGear = False


class GearOut:
    def __init__(self):
        self.Done = self.Error = self.fail = False
        self.ErrorID = 0

    def __call__(self, Slave, Execute, **kw):
        if Execute:
            if self.fail:
                self.Error = True; self.ErrorID = 7002
            else:
                assert abs(Slave.NcToPlc.ActVelo) <= .01, "Slave decoupled in motion"
                Slave.master = None
                self.Done = True
        else: self.Done = False


class Timer:
    def __init__(self): self.Q = False; self.elapsed = 0
    def __call__(self, IN, PT):
        self.elapsed = self.elapsed + 1 if IN else 0
        self.Q = IN and self.elapsed >= PT


class Power:
    def __init__(self):
        self.Status = self.Done = self.Busy = self.Active = self.Error = False
        self.ErrorID = 0
    def __call__(self, Enable=False, **kw): self.Status = Enable


class Program:
    def __init__(self, name, extra=None):
        xml = ET.parse(PLC / "POUs" / (name + ".TcPOU"))
        body = xml.findtext(".//Implementation/ST")
        self.code = compile("\n".join(ST(body).block()), name, "exec")
        self.env = dict(ABS=abs, INT_TO_USINT=int, UDINT_TO_INT=int, UDINT_TO_ULINT=int, UDINT_TO_TIME=int,
                        UINT_TO_USINT=int, MC_Aborting=0)
        if extra: self.env.update(extra)
        declaration = re.sub(r"//[^\n]*", "", xml.findtext(".//Declaration"))
        factories = {"MC_MoveAbsolute": Move, "MC_Stop": Stop, "MC_GearIn": GearIn, "MC_GearOut": GearOut, "TON": Timer,
                     "ExternalValidationSync": lambda: Program("ExternalValidationSync")}
        for match in re.finditer(r"(\w+)\s*:\s*(?:ARRAY\[(\d+)\.\.(\d+)\] OF )?(\w+)(?:\s*:=\s*([^;]+))?;", declaration):
            key, lower, upper, kind, initial = match.groups()
            factory = factories.get(kind, lambda: 0)
            value = factory()
            if initial: value = eval(ST.expr(ST(initial).tokens))
            if upper: value = [factory() for _ in range(int(upper) + 1)]
            self.env.setdefault(key, value)

    def __getattr__(self, name): return self.env[name]
    def __call__(self, **kw):
        self.env.update(kw)
        exec(self.code, self.env)


def globals_for(mode, cycles, final):
    g = types.SimpleNamespace()
    declaration = ET.parse(PLC / "GVLs/G.TcGVL").findtext(".//Declaration")
    for match in re.finditer(r"(program_test_\w+)\s*:\s*(?:ARRAY\[(\d+)\.\.(\d+)\] OF )?(\w+)(?:\s*:=\s*([^;]+))?;", declaration):
        key, lower, upper, kind, initial = match.groups()
        value = [0] * (int(upper) + 1) if upper else eval(ST.expr(ST(initial or "0").tokens))
        setattr(g, key, value)
    g.program_test_mode = mode
    g.program_test_cycle_count = cycles
    g.program_test_final_forward_distance_mm = final
    g.axis = [Axis(i, 90. + i) for i in range(8)]
    g.axis[7].NcToPlc.ActPos = 37.
    g.leftlimit = [0.] * 8
    g.power_ = [Power() for _ in range(8)]
    g.power_output = [Power() for _ in range(8)]
    g.SetPointGenEnable = [Power() for _ in range(8)]
    g.SetPointGenDisable = [Power() for _ in range(8)]
    g.SetPointGenDisable_output = [Power() for _ in range(8)]
    g.self_check_done = True
    g.cylinder1_value = g.cylinder2_value = g.cylinder3_value = g.cylinder4_value = 0
    g.fn_1_value, g.ft_1_value, g.fn_2_value, g.ft_2_value = 113, -31, 207, 61
    return g


def advance(g):
    old = [a.NcToPlc.ActPos for a in g.axis]
    for axis in g.axis:
        if axis.move:
            move = axis.move
            move[3] += 1
            axis.NcToPlc.ActPos = move[1] + (move[2] - move[1]) * min(1., move[3] / 4.)
            if move[3] >= 4:
                move[0].Done = True
                axis.move = None
                axis.NcToPlc.ActVelo = 0.
    for axis in g.axis:
        if axis.master:
            axis.NcToPlc.ActPos += axis.master.NcToPlc.ActPos - old[axis.master.index]
            axis.NcToPlc.ActVelo = axis.master.NcToPlc.ActVelo


def run(mode, cycles, final, abort_phase=None, fail_sync=None):
    g = globals_for(mode, cycles, final)
    p = Program("ProgrammedDeliveryExperiment", {"G": g})
    p()
    g.program_test_setup_req = True
    rows = []
    requested = False
    aborted = False
    for tick in range(10000):
        advance(g)
        if g.program_test_setup_done and not requested:
            g.program_test_start_req = True; requested = True
        abort_ready = abort_phase not in (4, 9) or (g.program_test_sync_state == 2 and g.axis[1].NcToPlc.ActVelo != 0)
        if abort_phase is not None and g.program_test_phase == abort_phase and abort_ready and not aborted:
            g.program_test_abort_req = True; aborted = True
        if fail_sync:
            getattr(p.fb_external_sync, "gear_" + fail_sync).fail = True
        p()
        if mode != 2:
            assert g.axis[5].NcToPlc.ActPos == 95.
        if mode == 4:
            assert not p.setup_exec[5] and not p.setup_exec[7]
            assert g.axis[7].NcToPlc.ActPos == 37.
            if g.program_test_phase >= 2:
                assert g.cylinder4_value == g.program_test_cylinder4_close_word
                assert g.cylinder3_value == 400
            if g.program_test_phase in (5, 6, 7):
                assert g.axis[6].master is None
                assert g.axis[6].NcToPlc.ActVelo == 0
                assert p.fb_external_sync.SyncState in (0, 4)
        if g.program_test_sample_arm:
            rows.append(dict(sample_index=len(rows), plc_time_us=len(rows)*1000, phase=g.program_test_phase,
                event_sequence=g.program_test_event_sequence, cycle_index=g.program_test_cycle_index,
                sync_state=g.program_test_sync_state, axis1_pos=g.axis[1].NcToPlc.ActPos,
                axis1_vel=g.axis[1].NcToPlc.ActVelo, axis6_pos=g.axis[6].NcToPlc.ActPos,
                axis6_vel=g.axis[6].NcToPlc.ActVelo, cylinder1=g.cylinder1_value,
                cylinder2=g.cylinder2_value, cylinder3=g.cylinder3_value, cylinder4=g.cylinder4_value))
        if g.program_test_phase in (10, 11, 12):
            break
    else: raise AssertionError(f"State machine did not finish: {mode, cycles, final, p.state, p.fb_external_sync.SyncState}")
    if abort_phase:
        assert g.program_test_phase == 11
    elif fail_sync:
        assert g.program_test_phase == 12 and g.program_test_error_axis == 6
    else:
        assert g.program_test_phase == 10
        if mode == 4:
            assert math.isclose(g.axis[6].NcToPlc.ActPos, 451. - cycles*20. - final)
            coupled = [row for i, row in enumerate(rows) if row["sync_state"] == 2 and (i == 0 or rows[i-1]["sync_state"] != 2)]
            assert len(coupled) == cycles + (1 if final else 0)
    assert not p.forward_exec and not p.final_exec and not p.return_exec
    for _ in range(4): advance(g); p()
    assert g.axis[6].master is None
    return rows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    count = 0
    for mode in (1, 2, 4):
        for cycles in (1, 3):
            for final in (0., 10.):
                rows = run(mode, cycles, final)
                count += 1
                if mode == 4 and cycles == 3 and final == 10:
                    with (args.output / "plc_trace.csv").open("w", encoding="utf-8", newline="") as stream:
                        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                        writer.writeheader(); writer.writerows(rows)
    for phase in (4, 5, 6, 7, 9):
        run(4, 1, 10., abort_phase=phase); count += 1
    for failure in ("in", "out"):
        run(4, 1, 10., fail_sync=failure); count += 1
    result = dict(scenarios=count, production_st_executed=True, motion_blocks="offline_mocks", hardware_connected=False,
                  twincat_compilation=False)
    (args.output / "plc_tests.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result))


if __name__ == "__main__":
    main()
