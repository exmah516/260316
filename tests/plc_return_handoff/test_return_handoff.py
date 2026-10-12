"""执行正式 handle 的 ST；NC 功能块仅模拟握手，不能替代 TwinCAT/驱动验证。"""
import re
import types
import unittest
import xml.etree.ElementTree as ET
from copy import deepcopy
from pathlib import Path

from plc_st_parser import ST

PLC = Path(__file__).resolve().parents[2] / "下位机工程程序/250902/Untitled2"


class ScanReturn(Exception):
    pass


def end_scan():
    raise ScanReturn()


class Block(types.SimpleNamespace):
    def __init__(self, **values):
        super().__init__(Execute=False, Done=False, Busy=False, Error=False,
                         ErrorID=0, Enabled=True, CommandAborted=False, **values)

    def __call__(self, **inputs):
        self.__dict__.update(inputs)


class Timer:
    Q = False

    def __call__(self, **inputs):
        pass


def defaults(declaration):
    declaration = re.sub(r"//[^\n]*", "", declaration)
    declaration = re.sub(r"\b(?:VAR_GLOBAL|VAR_INPUT|VAR_OUTPUT|VAR|END_VAR)\b", "", declaration)

    def value(kind):
        array = re.fullmatch(r"ARRAY\s*\[\d+\.\.(\d+)\]\s+OF\s+(.+)", kind)
        if array:
            return [value(array[2]) for _ in range(int(array[1]) + 1)]
        if kind == "TON":
            return Timer()
        if kind.startswith("MC_"):
            return Block()
        if kind == "ST_AxisPlannedReturnCmd":
            return types.SimpleNamespace(Req=False, Busy=False, Done=False, Error=False,
                                         ErrorId=0, TargetAbs=0., Velocity=0., Acc=0., Dec=0., Jerk=0.)
        return False if kind == "BOOL" else 0.

    result = {}
    for names, spec in re.findall(r"(?m)^\s*([\w,\s]+?)\s*(?:AT\s+%[IQ]\*)?\s*:\s*([^;]+);", declaration):
        kind, _, initial = spec.partition(":=")
        for name in names.split(","):
            name = name.strip()
            result[name] = value(kind.strip())
            if initial.strip():
                try:
                    result[name] = eval(ST.expr(ST(initial).tokens), {"_init": 0})
                    if kind.strip().startswith("ARRAY"):
                        result[name] = [0] + result[name]
                except (NameError, SyntaxError):
                    pass
    return result


class Simulation:
    def __init__(self):
        source = ET.parse(PLC / "POUs/handle.TcPOU")
        self.g = types.SimpleNamespace(**defaults(ET.parse(PLC / "GVLs/G.TcGVL").findtext(".//Declaration")))
        self.env = defaults(source.findtext(".//Declaration"))
        # ST 的结构体赋值复制值，不能用 Python 的对象引用代替。
        self.env.update(G=self.g, st_copy=deepcopy, ABS=abs, MIN=min, MAX=max, TO_LREAL=float,
                        MC_Direction=types.SimpleNamespace(MC_Positive_Direction=1, MC_Negative_Direction=-1),
                        _handle=2, _err=4, end_scan=end_scan)
        body = source.findtext("./POU/Implementation/ST").replace("RETURN;", "end_scan();")
        self.code = compile("\n".join(ST(body).block()), "production_handle", "exec")
        self.g.axis = [types.SimpleNamespace(
            NcToPlc=types.SimpleNamespace(ActPos=500., ActVelo=0.),
            PlcToNc=types.SimpleNamespace(ExtSetPos=500., ExtSetVelo=0.),
            Status=types.SimpleNamespace(Error=False, ErrorID=0, DriveDeviceError=False,
                                         Disabled=False, DriveLimitActive=False)) for _ in range(8)]
        for power in self.g.power_:
            power.Enable = power.Enable_Positive = power.Enable_Negative = True
        self.g.power_output = [types.SimpleNamespace(Done=True, Error=False, ErrorID=0) for _ in range(8)]
        self.g.reset_output = [types.SimpleNamespace(Error=False, ErrorID=0) for _ in range(8)]
        self.g.host_session_id = 1
        self.g.host_comm_timeout = False
        self.g.gen_state = 2
        self.g.handle_reinit_done = True
        self.env["init_done"] = True
        self.env["ext_setpoint_enabled"] = [True] * 8
        self.env["right_slimit"] = [self.g.leftlimit[i] + self.env["rslimit_distance"][i] for i in range(8)]
        self.feed = {}

        def feed(**inputs):
            axis = next(i for i, ref in enumerate(self.g.axis) if ref is inputs["Axis"])
            self.feed[axis] = (inputs["Position"], inputs["Velocity"], inputs["Acceleration"])
            inputs["Axis"].PlcToNc.ExtSetPos = inputs["Position"]
            inputs["Axis"].PlcToNc.ExtSetVelo = inputs["Velocity"]
        self.env["MC_ExtSetPointGenFeed"] = feed

    def scan(self):
        self.g.host_heartbeat_sequence += 1
        try:
            exec(self.code, self.env)
        except ScanReturn:
            pass

    def arrived(self, axis=6):
        self.g.axis[axis].NcToPlc.ActPos = 580.02
        self.g.return_cmd[axis].Req = True
        self.g.return_cmd[axis].Busy = True
        self.env["return_state"][axis] = 20
        self.env["return_target_abs"][axis] = 580.
        self.env["fb_return_move_abs"][axis].Done = True
        self.env["ext_setpoint_enabled"][axis] = False
        self.scan()
        assert self.env["return_state"][axis] == 30
        self.scan()
        assert self.env["return_state"][axis] == 31
        return self.feed[axis]


class HandoffTests(unittest.TestCase):
    def test_target_write_cannot_replace_preloaded_hold(self):
        for axis in (1, 3, 5, 6):
            for bypass in (False, True):
                with self.subTest(axis=axis, bypass=bypass):
                    sim = Simulation()
                    sim.g.startup_smoothing_bypass = bypass
                    expected = sim.arrived(axis)
                    self.assertEqual(expected, (580.02, 0., 0.))
                    for _ in range(12):
                        # 上位机未收到 Done 时，100 Hz 输出仍包含回退目标。
                        sim.g.refer[axis] = 580. - sim.g.init_pos[axis]
                        sim.scan()
                        self.assertEqual(sim.feed[axis], expected)

    def test_done_waits_for_host_handoff_before_accepting_reference(self):
        sim = Simulation()
        expected = sim.arrived()
        sim.g.SetPointGenEnable[6].Done = True
        sim.scan()
        self.assertTrue(sim.g.return_cmd[6].Done)
        for _ in range(12):
            # Done 通知可早于下一份位置快照，不能立即接收旧快照的参考。
            sim.g.refer[6] = 570. - sim.g.init_pos[6]
            sim.scan()
            self.assertEqual(sim.feed[6], expected)
        sim.g.return_cmd[6].Req = False
        sim.g.refer[6] = 580.02 - sim.g.init_pos[6]
        sim.scan()
        self.assertFalse(sim.g.return_cmd[6].Done)
        sim.g.refer[6] += 1.
        sim.scan()
        self.assertGreater(sim.feed[6][0], expected[0])

    def test_move_error_at_target_is_not_reported_as_done(self):
        sim = Simulation()
        sim.env["return_state"][6] = 20
        sim.env["return_target_abs"][6] = 500.
        sim.g.return_cmd[6].Req = True
        sim.env["fb_return_move_abs"][6].Error = True
        sim.env["fb_return_move_abs"][6].ErrorID = 0x4650
        sim.scan()
        self.assertTrue(sim.env["return_finish_error"][6])

    def test_near_target_without_done_still_waits_for_move(self):
        sim = Simulation()
        sim.env["return_state"][6] = 20
        sim.env["return_target_abs"][6] = 500.
        sim.g.return_cmd[6].Req = True
        sim.scan()
        self.assertEqual(sim.env["return_state"][6], 20)
        self.assertTrue(sim.env["fb_return_move_abs"][6].Execute)
        self.assertFalse(sim.g.return_cmd[6].Done)
        sim.env["fb_return_move_abs"][6].Done = True
        sim.scan()
        self.assertEqual(sim.env["return_state"][6], 30)

    def test_move_failure_is_preserved_until_host_clears_request(self):
        for fault in ("Error", "CommandAborted"):
            with self.subTest(fault=fault):
                sim = Simulation()
                sim.env["return_state"][6] = 20
                sim.env["return_target_abs"][6] = 500.
                sim.g.return_cmd[6].Req = True
                setattr(sim.env["fb_return_move_abs"][6], fault, True)
                sim.env["fb_return_move_abs"][6].ErrorID = 0x4650 if fault == "Error" else 0
                sim.scan()
                sim.scan()
                expected = sim.feed[6]
                sim.g.SetPointGenEnable[6].Done = True
                sim.scan()
                self.assertTrue(sim.g.return_cmd[6].Error)
                self.assertFalse(sim.g.return_cmd[6].Done)
                self.assertEqual(sim.g.return_cmd[6].ErrorId, 0x4650 if fault == "Error" else 0)
                for _ in range(10):
                    sim.g.refer[6] = 400.
                    sim.scan()
                    self.assertEqual(sim.feed[6], expected)
                sim.g.return_cmd[6].Req = False
                sim.scan()
                self.assertFalse(sim.g.return_cmd[6].Error)

    def test_disabled_without_axis_error_still_trips_on_power_status(self):
        sim = Simulation()
        expected = sim.arrived()
        sim.g.power_output[6].Done = False
        sim.g.axis[6].Status.Disabled = True
        sim.g.refer[6] = 570. - sim.g.init_pos[6]
        sim.scan()
        self.assertTrue(sim.g.estop_hold_req)
        self.assertEqual(sim.g.gen_state, sim.env["_err"])
        self.assertEqual(sim.g.estop_diag_count, 1)
        self.assertFalse(sim.g.estop_diag_host_timeout)
        self.assertFalse(sim.g.estop_diag_power[6].Done)
        self.assertTrue(sim.g.estop_diag_power_enable[6])
        self.assertTrue(sim.g.estop_diag_axis[6].Disabled)
        self.assertFalse(sim.g.estop_diag_axis[6].Error)
        self.assertFalse(sim.g.estop_diag_return_cmd[6].Error)
        self.assertEqual(sim.g.estop_diag_return_state[6], 31)
        self.assertEqual(sim.g.estop_diag_feed_pos[6], expected[0])
        self.assertEqual(sim.g.estop_diag_refer_abs[6], 570.)
        # 模拟清错后重新进入手柄初始化，原始快照必须保留。
        sim.g.power_output[6].Done = True
        sim.g.axis[6].Status.Disabled = False
        sim.g.return_cmd[6].Req = False
        sim.g.gen_state = sim.env["_handle"]
        sim.g.handle_reinit_req = True
        for block in sim.g.SetPointGenEnable:
            block.Done = True
        sim.scan()
        sim.scan()
        self.assertFalse(sim.g.estop_hold_req)
        self.assertEqual(sim.g.estop_diag_count, 1)
        self.assertFalse(sim.g.estop_diag_power[6].Done)
        self.assertTrue(sim.g.estop_diag_axis[6].Disabled)
        self.assertTrue(sim.g.estop_diag_return_cmd[6].Req)
        sim.g.reset_output[3].Error = True
        sim.g.reset_output[3].ErrorID = 123
        sim.scan()
        self.assertEqual(sim.g.estop_diag_count, 2)
        self.assertEqual(sim.g.estop_diag_reset[3].ErrorID, 123)

    def test_fault_on_any_axis_is_captured_before_recovery(self):
        for axis in range(1, 8):
            for source in ("axis", "power_output", "reset_output"):
                with self.subTest(axis=axis, source=source):
                    sim = Simulation()
                    status = sim.g.axis[axis].Status if source == "axis" else getattr(sim.g, source)[axis]
                    status.Error = True
                    status.ErrorID = 100 + axis
                    sim.scan()
                    self.assertTrue(sim.g.estop_hold_req)
                    name = {"axis": "axis", "power_output": "power", "reset_output": "reset"}[source]
                    self.assertEqual(getattr(sim.g, "estop_diag_" + name)[axis].ErrorID, 100 + axis)


if __name__ == "__main__":
    unittest.main()
