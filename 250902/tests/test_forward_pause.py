"""执行当前handle的生产Action，模拟NC握手；不连接ADS或设备。"""
import json
import math

from test_handle_delivery import Simulation


class PauseSimulation(Simulation):
    def __init__(self, distance=10., duration=7, final=15., cycles=2, enabled=True, mode=1):
        self.clock = 0
        self.dwells = []
        self.stream = []
        self.sequences = set()
        super().__init__(mode=mode, cycles=cycles, final=final)
        self.g.program_test_forward_pause_enabled = enabled
        self.g.program_test_forward_pause_distance_mm = distance
        self.g.program_test_forward_pause_duration_ms = duration

    def tick(self, *args, **kwargs):
        super().tick(*args, **kwargs)
        self.clock += 1
        g = self.g
        if g.program_test_wait_action == 3:
            assert g.program_test_phase in (4, 9) and g.program_test_sample_arm
            assert not g.return_cmd[1].Req and not g.return_cmd[1].Busy
            assert self.env['pd_step'] == 2
            expected = g.program_test_return_target_abs - self.env['pd_pause_distance']
            assert math.isclose(g.axis[1].NcToPlc.ActPos, expected)
            assert g.cylinder1_value == 400 and g.cylinder2_value == self.env['pd_close2']
            if not self.dwells or self.dwells[-1][1] is not None:
                self.dwells.append([self.clock, None, g.program_test_phase, g.program_test_cycle_index])
        elif self.dwells and self.dwells[-1][1] is None:
            self.dwells[-1][1] = self.clock
        for slot in (1, 2):
            sequence = g.experiment_record_block_sequence[slot]
            if not g.experiment_record_block_ready[slot] or sequence in self.sequences:
                continue
            self.sequences.add(sequence)
            prefix = f'experiment_record_block{slot-1}_'
            for i in range(g.experiment_record_block_count[slot]):
                self.stream.append(tuple(getattr(g, prefix + field)[i] for field in ('index', 'time_us', 'fn1', 'phase')))

    def finish(self):
        self.until(lambda: self.g.program_test_phase in (10, 11, 12), limit=160000)
        assert self.g.program_test_phase == 10
        assert not self.env['pd_pause_pending'] and not self.env['pd_pause_timer'].Q
        assert self.g.program_test_wait_action == 0


def main():
    scenarios = 0
    for distance, duration, final, cycles in ((10., 7, 15., 3), (10., 1, 10., 1), (10., 3, 5., 1),
                                             (20., 8, 20., 2), (.1, 5, 0., 1), (10., 60000, 0., 1)):
        sim = PauseSimulation(distance, duration, final, cycles)
        sim.prepare()
        sim.start()
        sim.finish()
        assert len(sim.dwells) == cycles + int(final >= distance)
        assert all(end-start == duration for start, end, _, _ in sim.dwells)
        assert math.isclose(sim.g.axis[1].NcToPlc.ActPos, sim.g.program_test_return_target_abs-final)
        # 每段的定位目标顺序：暂停点、原终点；在终点暂停时不能再发零距离定位。
        forward = [target for phase, axis, target, _ in sim.moves if phase in (4, 9) and axis == 1]
        start = sim.g.program_test_return_target_abs
        per_cycle = [start-distance] + ([sim.g.program_test_trigger_target_abs] if distance < 20 else [])
        expected = per_cycle * cycles
        if final:
            expected += ([start-distance] if distance < final else []) + [start-final]
        assert forward == expected, (forward, expected)
        scenarios += 1

    for mode in (1, 2, 4):
        sim = PauseSimulation(enabled=False, mode=mode)
        sim.prepare()
        sim.start()
        if mode == 4:
            for _ in range(20): sim.tick()
            assert sim.g.program_test_phase == 4 and not sim.dwells
        else:
            sim.finish()
            assert not sim.dwells
        scenarios += 1

    for distance, duration, mode in ((0., 3, 1), (-1., 3, 1), (21., 3, 1), (float('nan'), 3, 1),
                                     (float('inf'), 3, 1), (10., 0, 1), (10., 60001, 1), (10., 3, 2), (10., 3, 4)):
        sim = PauseSimulation(distance, duration, mode=mode)
        sim.g.program_test_setup_req = True
        sim.tick()
        assert sim.g.program_test_phase == 12 and sim.g.program_test_status_error_id == 0x7209
        assert not sim.moves
        scenarios += 1

    for phase in (4, 9):
        for fault in ('abort', 'axis', 'host', 'overflow'):
            sim = PauseSimulation(cycles=1)
            sim.prepare()
            sim.start()
            sim.until(lambda: sim.g.program_test_phase == phase and sim.g.program_test_wait_action == 3)
            if fault == 'abort': sim.g.program_test_abort_req = True
            elif fault == 'overflow': sim.g.experiment_record_overflow = True
            else:
                sim.env['pd_fault'] = True
                sim.env['all_powered'] = True
                sim.env['has_axis_error'] = fault == 'axis'
                sim.g.host_comm_timeout = fault == 'host'
            sim.tick()
            assert sim.g.program_test_phase == (11 if fault == 'abort' else 12)
            moves = len(sim.moves)
            for _ in range(20): sim.tick()
            assert len(sim.moves) == moves and not sim.env['pd_pause_timer'].Q
            assert not sim.env['pd_pause_pending'] and sim.g.program_test_wait_action == 0
            if fault == 'abort':
                sim.g.experiment_record_clear = True
                sim.tick()
                sim.prepare()
                previous = len(sim.dwells)
                sim.start()
                sim.finish()
                assert len(sim.dwells) == previous+2
            scenarios += 1

    for phase in (4, 9):
        for after_pause in (False, True):
            sim = PauseSimulation(cycles=1)
            sim.prepare()
            sim.start()
            if after_pause:
                sim.until(lambda: sim.g.program_test_phase == phase and sim.g.program_test_wait_action == 3)
            sim.until(lambda: sim.g.program_test_phase == phase and sim.g.return_cmd[1].Busy)
            target = sim.g.return_cmd[1].TargetAbs
            sim.g.return_cmd[1].Error = True
            sim.g.return_cmd[1].ErrorId = 1234
            sim.tick()
            assert sim.g.program_test_phase == 12 and not sim.env['pd_pause_pending']
            assert sim.g.program_test_error_target_abs == target
            scenarios += 1

    # 即使Done未及时复位，也不能重发请求或跳过续行段。
    sim = PauseSimulation(duration=1, cycles=1)
    sim.prepare()
    sim.start()
    sim.until(lambda: sim.g.program_test_wait_action == 3)
    count = len(sim.moves)
    for _ in range(5):
        sim.g.return_cmd[1].Done = True
        sim.tick()
        assert len(sim.moves) == count and sim.g.program_test_wait_action == 3
    sim.finish()
    scenarios += 1

    sim = PauseSimulation(cycles=1, final=0.)
    sim.prepare()
    sim.g.program_test_forward_pause_distance_mm = 2.
    sim.g.program_test_forward_pause_duration_ms = 50
    sim.start()
    sim.finish()
    assert sim.dwells[0][1]-sim.dwells[0][0] == 7
    scenarios += 1

    sim = PauseSimulation(duration=35000, final=15., cycles=1)
    sim.prepare()
    sim.start()
    sim.finish()
    assert all(end-start == 35000 for start, end, _, _ in sim.dwells)
    assert len(sim.stream) == sim.g.experiment_record_total_count > 70000
    assert sim.g.program_test_sample_overflow and not sim.g.experiment_record_overflow
    assert [row[0] for row in sim.stream] == list(range(len(sim.stream)))
    assert all(b[1]-a[1] == 1000 for a, b in zip(sim.stream, sim.stream[1:]))
    assert all(row[2] == 113 for row in sim.stream) and sim.stream[-1][3] == 10
    scenarios += 1
    print(json.dumps(dict(passed=scenarios, streamed_samples=len(sim.stream),
                         source='handle.ProgramDelivery / ProgramRecord',
                         full_handle_executed=False, hardware_connected=False, twincat_compiled=False)))


if __name__ == '__main__':
    main()
