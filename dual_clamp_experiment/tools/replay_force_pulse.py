"""Replay the seven reference files through the actual C++ causal guard."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

import numpy as np
import pandas as pd


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--reference",type=Path,required=True)
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[1]
    exe=root/"x64/Debug_inertia25g/test_force_pulse.exe"
    args.output.mkdir(parents=True,exist_ok=True)
    rows=[]
    for stamp in ["074959","075124","075215","075309","075356","075449","075539"]:
        source=next((root/"x64/Debug/records").glob(f"20260911_{stamp}_*"))
        hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest()
                for p in source.iterdir() if p.is_file()}
        s=pd.read_csv(source/"samples_1khz.csv")
        old=pd.read_csv(args.reference/f"{stamp}_cleaned.csv")
        meta=json.loads((source/"experiment.json").read_text(encoding="utf-8-sig"))
        fn=np.rint(s.fn1_zeroed+meta["fn1_zero"]).astype(int)
        ft=np.rint(s.ft1_zeroed+meta["ft1_zero"]).astype(int)
        fixture=pd.DataFrame(dict(index=s.sample_index,time_us=s.plc_time_us,fn=fn,ft=ft))
        input_path=args.output/f"{stamp}_input.csv"
        output_path=args.output/f"{stamp}_causal.csv"
        fixture.to_csv(input_path,index=False)
        subprocess.run([str(exe),str(input_path),str(output_path)],check=True)
        result=pd.read_csv(output_path)
        mask=result.replaced.to_numpy()==1
        oldmask=old.periodic_pulse_excluded.to_numpy()==1
        assert len(result)==len(s)
        assert np.array_equal(result.fn_counts[~mask],fn[~mask])
        assert np.array_equal(result.ft_counts[~mask],ft[~mask])
        for i in np.flatnonzero(mask):
            k=int(result.source_index.iloc[i])
            assert k<i
            assert result.fn_counts.iloc[i]==fn.iloc[k]
            assert result.ft_counts.iloc[i]==ft.iloc[k]
            assert result.age_us.iloc[i]==s.plc_time_us.iloc[i]-s.plc_time_us.iloc[k]
            assert result.age_us.iloc[i]<=103000
        row=dict(record=stamp,n=len(s),replaced=int(mask.sum()),
                 offline_excluded=int(oldmask.sum()),outside_offline_mask=int(sum(mask&~oldmask)),
                 passthrough_offline_points=int(sum(~mask&oldmask)),
                 maximum_hold_age_ms=float(result.age_us.max()/1000))
        assert row["outside_offline_mask"]==0, row
        slope=meta["sensor_slopes_N_per_count"]
        cleanfn=(result.fn_counts-meta["fn1_zero"])*slope["fn1"]*meta["installation_axial_gain"]
        cleanft=(result.ft_counts-meta["ft1_zero"])*slope["ft1"]*meta["installation_torque_gain_Nmm_per_N"]/meta["tangential_arm_mm"]
        r=pd.read_csv(source/"causal_force.csv")
        # Existing UI replay schema plus the six new pulse fields.
        ui=pd.DataFrame(dict(time=s.plc_time_us*1e-6,velocity=s.axis1_vel_mm_s,
            moving=s.cylinder2_cmd,fixed=s.cylinder1_cmd,angle=s.axis2_pos_deg,
            phase=s.phase,cycle=s.cycle_index,fn=r.fn_original_N,ft=r.ft_original_N,
            pred_fn=r.fn_prediction_N,pred_ft=r.ft_prediction_N,mode=1,valid=r.model_valid,
            model2fn=cleanfn,model2ft=cleanft,model2fnvalid=0,model2ftvalid=0,
            cleanfn=cleanfn,cleanft=cleanft,replaced=result.replaced,status=result.status,
            age=result.age_us,locked=result.locked))
        ui.to_csv(args.output/f"{stamp}_ui.csv",index=False)
        if stamp=="075539":
            # Count signature is reused only to check the guidewire UI/calibration
            # path; these are not actual guidewire validation data.
            ui["mode"]=2
            ui["fn"]=(fn-meta["fn1_zero"])*slope["fn2"]*meta["installation_axial_gain"]
            ui["ft"]=(ft-meta["ft1_zero"])*slope["ft2"]*meta["installation_torque_gain_Nmm_per_N"]/meta["tangential_arm_mm"]
            ui["cleanfn"]=(result.fn_counts-meta["fn1_zero"])*slope["fn2"]*meta["installation_axial_gain"]
            ui["cleanft"]=(result.ft_counts-meta["ft1_zero"])*slope["ft2"]*meta["installation_torque_gain_Nmm_per_N"]/meta["tangential_arm_mm"]
            ui.to_csv(args.output/"synthetic_guidewire_ui.csv",index=False)
            changed=fixture.copy()
            cut=len(changed)//2
            changed.loc[cut:,"fn"]=0
            changed.loc[cut:,"ft"]=0
            changed.to_csv(args.output/"future_changed.csv",index=False)
            subprocess.run([str(exe),str(args.output/"future_changed.csv"),
                            str(args.output/"future_changed_output.csv")],check=True)
            other=pd.read_csv(args.output/"future_changed_output.csv")
            pd.testing.assert_frame_equal(result.iloc[:cut],other.iloc[:cut])
        for name,expected in hashes.items():
            assert hashlib.sha256((source/name).read_bytes()).hexdigest()==expected
        rows.append(row)
    report=dict(records=rows,source_files_unchanged=True,causality_prefix_test=True,
                guidewire_real_data_validated=False,
                outside_offline_mask=sum(r["outside_offline_mask"] for r in rows),
                replacements=sum(r["replaced"] for r in rows))
    (args.output/"replay_summary.json").write_text(json.dumps(report,indent=2),encoding="utf-8")
    print(json.dumps(report,indent=2))


if __name__=="__main__":
    main()
