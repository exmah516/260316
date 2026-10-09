"""从目标PLC声明生成离线ADS类型表，核对当前三模式使用的符号。"""
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

from test_handle_delivery import PLC


def main():
    declaration = ET.parse(PLC / "GVLs/G.TcGVL").findtext(".//Declaration")
    declaration = re.sub(r"//[^\n]*", "", declaration).replace("VAR_GLOBAL", "")
    sizes = dict(BOOL=1, USINT=1, SINT=1, INT=2, UINT=2, WORD=2,
                 DINT=4, UDINT=4, REAL=4, TIME=4, ULINT=8, LREAL=8)
    symbols = {}
    for match in re.finditer(r"(?m)^\s*([\w,\s]+?)\s*(?:AT\s+%[IQ]\*)?\s*:\s*([^;]+);", declaration):
        names, specification = match.groups()
        kind = specification.split(":=")[0].strip()
        array = re.fullmatch(r"ARRAY\s*\[(\d+)\.\.(\d+)\]\s+OF\s+(\w+)", kind)
        if kind not in sizes and not array:
            continue
        for name in names.split(","):
            symbol = "G." + name.strip()
            assert symbol not in symbols, symbol
            if array:
                lower, upper, element = int(array[1]), int(array[2]), array[3]
                if element not in sizes:
                    continue
                symbols[symbol] = (upper - lower + 1) * sizes[element]
                for index in (range(lower, upper + 1) if upper - lower < 8 else (lower, upper)):
                    symbols[f"{symbol}[{index}]"] = sizes[element]
            else:
                symbols[symbol] = sizes[kind]
    for axis in range(1, 8):
        for field in ("ActPos", "ActVelo", "ActAcc"):
            symbols[f"G.axis[{axis}].NcToPlc.{field}"] = 8
    root = Path(__file__).resolve().parent.parent
    for filename in ("ProgrammedDeliveryAds.cpp", "ExperimentStreamAds.cpp"):
        source = (root / filename).read_text(encoding="utf-8-sig")
        references = set(re.findall(r'"(G\.[\w.\[\]]+)"', source))
        references = {name for name in references if not name.endswith("[") and name != "G.experiment_record_block"}
        if filename == "ExperimentStreamAds.cpp":
            for field in re.findall(r'block_symbol\((?:slot|block), "(\w+)"\)', source):
                references.update(f"G.experiment_record_block{slot}_{field}" for slot in (0, 1))
        assert not references - symbols.keys(), (filename, sorted(references - symbols.keys()))
    assert symbols["G.selfcheck_start_req"] == 1
    assert symbols["G.self_check_done"] == 1
    assert symbols["G.selfcheck_status"] == 4
    assert symbols["G.program_interface_version"] == 4
    if len(sys.argv) > 1:
        Path(sys.argv[1]).write_text("".join(f"{name} {size}\n" for name, size in symbols.items()), encoding="utf-8")
    print("PASS current-mode ADS symbols and PLC scalar/array sizes")


if __name__ == "__main__":
    main()
