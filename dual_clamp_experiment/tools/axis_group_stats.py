import csv
from pathlib import Path

import numpy as np

base = Path(r"D:\Work_files\Vessel intervention Robot\260316\dual_clamp_experiment\x64\Debug\records\analysis")
rows = list(csv.DictReader((base / "cycle_metrics.csv").open(encoding="utf-8-sig")))
groups = {
    "axis1_catheter": [x for x in rows if x["record_name"] in [
        "电缸不好使20260831_222416_有夹爪轴1往复五次-空载-0度-正常推-500-1000-1000-15",
        "电缸不好使20260831_222447_有夹爪轴1往复五次-空载-0度-正常推-500-1000-1000-15",
    ]],
    "axis6_guidewire": [x for x in rows if x["record_name"] in [
        "20260831_221559_有夹爪轴6往复五次-空载-0度-正常推-500-1000-1000-15",
        "20260831_221622_有夹爪轴6往复五次-空载-0度-正常推-500-1000-1000-15",
        "20260831_221641_有夹爪轴6往复五次-空载-0度-正常推-500-1000-1000-15",
    ]],
}
for name, group in groups.items():
    print(name, "cycles", len(group))
    for key in ["return_fn_range_N", "return_ft_range_N", "reclamp_fn_range_N", "reclamp_ft_range_N", "return_velocity_peak_mm_s", "return_acceleration_p95_mm_s2"]:
        values = np.asarray([float(x[key]) for x in group if x[key] not in ("", "nan")])
        print(key, "mean", round(float(values.mean()), 5), "sd", round(float(values.std(ddof=1)), 5), "median", round(float(np.median(values)), 5))
