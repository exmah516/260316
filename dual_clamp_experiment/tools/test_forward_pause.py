"""统一执行目标PLC的handle暂停回归，避免测试未被编译的历史功能块。"""
import argparse
import runpy
from pathlib import Path

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    tests = Path(__file__).resolve().parents[2] / "250902/tests"
    import sys
    sys.path.insert(0, str(tests))
    runpy.run_path(str(tests / "test_forward_pause.py"))["main"](args.output)
