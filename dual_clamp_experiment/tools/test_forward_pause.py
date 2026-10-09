"""离线执行生产PLC定位/停留与分块记录；不连接设备。"""
import argparse
import csv
import math
import re
from pathlib import Path
import xml.etree.ElementTree as ET

from test_external_plc import PLC, Program, advance, globals_for, run
from test_handle_delivery import declarations


class Simulation:
    def __init__(self, distance=10., duration=7, final=15., cycles=2, enabled=True, mode=1, stream=False):
        self.g = globals_for(mode, cycles, final)
        self.g.program_test_forward_pause_enabled = enabled
        self.g.program_test_forward_pause_distance_mm = distance
        self.g.program_test_forward_pause_duration_ms = duration
        self.p = Program("ProgrammedDeliveryExperiment", {"G": self.g})
        self.p()
        self.rows, self.dwells, self.blocks = [], [], []
        self.clock = 0
        self.recorder = None
        if stream:
            for name, value in declarations(ET.parse(PLC / "GVLs/G.TcGVL").findtext(".//Declaration")).items():
                if not hasattr(self.g, name):
                    setattr(self.g, name, value)
            self.recorder = Program("ExperimentRecorder", {"G": self.g, "SQRT": math.sqrt, "INT_TO_LREAL": float})
            self.g.experiment_record_clear = True
            self.recorder()
        self.g.program_test_setup_req = True

    def tick(self):
        advance(self.g)
        self.p()
        self.clock += 1
        g = self.g
        waiting = g.program_test_wait_action == 3
        if waiting:
            assert g.program_test_phase in (4, 9) and g.program_test_sample_arm
            assert g.axis[1].move is None and g.axis[1].NcToPlc.ActVelo == 0
            assert math.isclose(g.axis[1].NcToPlc.ActPos, 23. - self.p.pause_distance)
            assert g.cylinder1_value == 400 and g.cylinder2_value == g.program_test_cylinder2_close_word
            if not self.dwells or self.dwells[-1][1] is not None:
                self.dwells.append([self.clock, None, g.program_test_phase, g.program_test_cycle_index])
        elif self.dwells and self.dwells[-1][1] is None:
            self.dwells[-1][1] = self.clock
        if g.program_test_sample_arm:
            self.rows.append((g.program_test_phase, g.axis[1].NcToPlc.ActPos, waiting))
        if self.recorder:
            self.recorder()
            for slot in (1, 2):
                if not g.experiment_record_block_ready[slot]:
                    continue
                prefix = f"experiment_record_block{slot-1}_"
                names = ("index", "time_us", "phase", "event_sequence", "cycle_index", "axis1_pos", "axis1_vel",
                         "axis1_acc", "cylinder1", "cylinder2", "fn1", "ft1")
                for i in range(g.experiment_record_block_count[slot]):
                    self.blocks.append({name: getattr(g, prefix + name)[i] for name in names})
                g.experiment_record_block_ack_sequence[slot] = g.experiment_record_block_sequence[slot]
            assert not g.experiment_record_overflow

    def until(self, condition, limit=200000):
        for _ in range(limit):
            if condition():
                return
            self.tick()
        raise AssertionError((self.p.state, self.g.program_test_phase, self.g.program_test_status_error_id))

    def start(self):
        self.until(lambda: self.g.program_test_setup_done)
        self.g.program_test_start_req = True

    def finish(self):
        self.until(lambda: self.g.program_test_phase in (10, 11, 12))
        assert self.g.program_test_phase == 10
        assert not self.p.pause_pending and not self.p.pause_wait_active and not self.p.pause_timer.Q


def main(output):
    scenarios = 0
    for distance, duration, final, cycles in ((10., 7, 15., 3), (10., 1, 10., 1), (10., 3, 5., 1),
                                             (20., 8, 20., 2), (0.1, 5, 0., 1), (10., 60000, 0., 1)):
        sim = Simulation(distance, duration, final, cycles)
        sim.start()
        sim.finish()
        assert len(sim.dwells) == cycles + int(final >= distance)
        assert all(end - start == duration for start, end, _, _ in sim.dwells)
        assert math.isclose(sim.g.axis[1].NcToPlc.ActPos, 23. - final)
        scenarios += 1

    for mode in (1, 2, 4):
        run(mode, 3, 10.)
        scenarios += 1
    sim = Simulation(enabled=False)
    sim.start()
    sim.finish()
    assert not sim.dwells
    scenarios += 1

    for distance, duration, mode in ((0., 3, 1), (-1., 3, 1), (21., 3, 1), (float("nan"), 3, 1),
                                     (float("inf"), 3, 1), (10., 0, 1), (10., 60001, 1), (10., 3, 2), (10., 3, 4)):
        sim = Simulation(distance, duration, mode=mode)
        sim.tick()
        assert sim.p.state == 12 and sim.g.program_test_status_error_id == 0x7208
        assert all(axis.move is None for axis in sim.g.axis)
        scenarios += 1

    for phase in (4, 9):
        for fault in (False, True):
            sim = Simulation(cycles=1)
            sim.start()
            sim.until(lambda: sim.g.program_test_phase == phase and sim.g.program_test_wait_action == 3)
            if fault:
                move = sim.p.fb_forward_move if phase == 4 else sim.p.fb_final_move
                move.Error = True
                move.ErrorID = 1234
            else:
                sim.g.program_test_abort_req = True
            sim.tick()
            sim.tick()
            assert sim.g.program_test_phase == (12 if fault else 11)
            for _ in range(20):
                sim.tick()
            assert sim.g.axis[1].move is None and not sim.p.pause_wait_active and not sim.p.pause_timer.Q
            if not fault:
                sim.g.program_test_setup_req = True
                sim.tick()
                sim.start()
                previous = len(sim.dwells)
                sim.finish()
                assert len(sim.dwells) == previous + 2
            scenarios += 1

    # 暂停前及恢复后的运动错误/指令中断均进入错误终态。
    for phase in (4, 9):
        for after_pause in (False, True):
            for fault in ("Error", "CommandAborted"):
                sim = Simulation(cycles=1)
                sim.start()
                if after_pause:
                    sim.until(lambda: sim.g.program_test_phase == phase and sim.g.program_test_wait_action == 3)
                sim.until(lambda: sim.g.program_test_phase == phase and sim.g.axis[1].move is not None)
                move = sim.p.fb_forward_move if phase == 4 else sim.p.fb_final_move
                setattr(move, fault, True)
                move.ErrorID = 1234
                sim.tick()
                sim.tick()
                assert sim.g.program_test_phase == 12 and sim.g.axis[1].move is None
                assert not sim.p.pause_pending and not sim.p.pause_wait_active and not sim.p.pause_timer.Q
                scenarios += 1

    # 准备后更改ADS配置，不得改变已锁存的暂停行为。
    sim = Simulation(cycles=1, final=0.)
    sim.start()
    sim.g.program_test_forward_pause_distance_mm = 2.
    sim.g.program_test_forward_pause_duration_ms = 50
    sim.finish()
    assert sim.dwells[0][1] - sim.dwells[0][0] == 7
    scenarios += 1

    # 执行生产分块记录器，跨过旧32768点缓冲上限并确认最后一个不满块。
    sim = Simulation(duration=35000, final=15., cycles=1, stream=True)
    sim.start()
    sim.finish()
    assert len(sim.blocks) == sim.g.experiment_record_total_count == len(sim.rows)
    assert len(sim.blocks) > 70000 and sim.g.program_test_sample_overflow
    assert all(row["index"] == i and row["time_us"] == i * 1000 for i, row in enumerate(sim.blocks))
    assert sum(waiting for _, _, waiting in sim.rows) == 70000
    output.mkdir(parents=True, exist_ok=True)
    with (output / "pause_trace.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(sim.blocks[0]))
        writer.writeheader()
        writer.writerows(sim.blocks)
    sizes = dict(BOOL=1, USINT=1, UINT=2, WORD=2, UDINT=4, LREAL=8)
    declaration = ET.parse(PLC / "GVLs/G.TcGVL").findtext(".//Declaration")
    with (output / "pause_symbols.txt").open("w", encoding="utf-8") as f:
        for name, kind in re.findall(r"\b(program_test_\w+)\s*:\s*(\w+)", declaration):
            if kind in sizes:
                f.write(f"G.{name} {sizes[kind]}\n")
    print(f"PASS {scenarios + 1} pause scenarios; production PLC stream samples={len(sim.blocks)}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    main(parser.parse_args().output)
