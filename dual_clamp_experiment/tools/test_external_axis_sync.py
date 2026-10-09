from pathlib import Path
import sys

TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))
from test_handle_delivery import Simulation


def main():
    sim = Simulation(cycles=1)
    sim.prepare()
    sim.start()
    anchor1 = sim.g.axis[1].NcToPlc.ActPos
    anchor5 = sim.g.axis[5].NcToPlc.ActPos
    anchor6 = sim.g.axis[6].NcToPlc.ActPos
    target = sim.g.program_test_return_target_abs - 8.0
    sim.tick((target, sim.g.axis[2].NcToPlc.ActPos), cycle=1)
    assert sim.g.program_test_phase == 4
    d1 = sim.g.axis[1].NcToPlc.ActPos - anchor1
    d5 = sim.g.axis[5].NcToPlc.ActPos - anchor5
    d6 = sim.g.axis[6].NcToPlc.ActPos - anchor6
    assert abs(d1 - d5) <= 1e-9, (d1, d5)
    assert abs(d1 - d6) <= 1e-9, (d1, d6)
    print("PASS external axis 1/5/6 equal-displacement follower")


if __name__ == "__main__":
    main()
