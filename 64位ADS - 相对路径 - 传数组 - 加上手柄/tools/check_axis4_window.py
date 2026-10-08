"""隔离编译 main.cpp 中的真实双手柄分支，不加载 ADS/手柄 SDK。"""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / 'main.cpp').read_text(encoding='utf-8-sig')
start = source.index('// 双手柄同时控制（不区分递送/撤出）')
start = source.index('const double tol =', start)
end = source.index('\n\t\t\t}\n\t\t\telse', start)
branch = source[start:end]
clamp_start = source.index('cylinder3_cmd = (restore_clamps && planned_return.restore_stage')
clamp_end = source.index(';', source.index('cylinder4_cmd =', clamp_start)) + 1
clamps = source[clamp_start:clamp_end]
guard_start = source.index('if (planned_return.active() &&\n')
guard_end = source.index('\n\t\t}', guard_start) + len('\n\t\t}')
guard = source[guard_start:guard_end]
prefix = r'''
#include <algorithm>
#include <cmath>
#include <cassert>
#include <iostream>
using ULONGLONG = unsigned long long;
ULONGLONG GetTickCount64() { return 2000; }
enum class GuidewireMode { None };
enum class PlannedReturnMode { Dual };
enum class PlannedReturnRebaseScope { Both };
struct Crawl { double base_rel=0,rot_base_rel=0,rot_ref=0; bool window_active=true;
 double min_abs(){return 0;} double max_abs(){return 100;} };
bool is_within_range(double x,double a,double b,double t){return x>=a-t && x<=b+t;}
void capture_axis1_follow_baseline(){}
void apply_axis1_mirror_from_abs(double,bool){}
double compute_axis7_cmd_rel(){return 0;}
void apply_planned_return_outputs(){}
struct { bool active(){return false;} } planned_return;
double target=-1; bool returned=false;
bool begin_planned_return(PlannedReturnMode,PlannedReturnRebaseScope,bool,double,bool wire,double t){
 returned=wire; target=t; return true;
}
void run(bool forward,bool backward,bool handleReverse,double gap,double input6,double soft=1000){
 returned=false; target=-1;
 struct {double crawl_arrive_tol_mm=.1,catheter_axis6_window_min_gap_from_axis5_mm=10,
 catheter_axis6_window_size_mm=20,relocation_inset_mm=2,transaction_merge_eps_mm=.2,
 axis6_soft_limit_from_left_mm,axis_rot_scale_deg=1;} cfg;
 cfg.axis6_soft_limit_from_left_mm=soft;
 Crawl axis1_crawl;
 bool axis1_reverse_pressed=handleReverse,guidewire_b6_pressed=false;
 bool axis4_forward_request=forward,axis4_reverse_request=backward;
 auto guidewire_mode=GuidewireMode::None;
 double axis1_abs=50,axis1_linear_increment_mm=0,axis6_linear_increment_mm=input6;
 bool axis3_delivery_stop_active=false,axis4_axis6_coupling_active_prev=true;
 ULONGLONG axis4_axis6_coupling_last_ms=1000;
 double axis4_coupled_axis6_speed_mm_s=1;
 double axis1_follow_cmd_abs=50,axis6_follow_cmd_abs=50+gap;
 double plc_init_pos[7]={},plc_leftlimit[7]={},axis5_base_rel=0;
 double pos[7]={},axis1_rot_filtered=0,axis2_hold_rel=0,axis5_abs=50,axis6_abs=50+gap;
 int cylinder1_cmd,cylinder2_cmd,cylinder3_cmd,cylinder4_cmd;
 struct {int cyl1_open=1,cyl2_clamp=2,cyl3_open=450,cyl4_clamp=500;} cyl;
'''
suffix = r'''
 assert(cylinder3_cmd==450 && cylinder4_cmd==500);
 assert(pos[0]==50); // 轴1方向与静止输入不受轴4影响。
 assert(axis6_follow_cmd_abs<=std::max(50+gap,soft));
}
void check_guards();
int main(){
 run(false,true,false,30,0); assert(returned && target==60); // A：后退触发大端，快进小端。
 run(true,false,false,10,0); assert(returned && target==82); // B：前进保持原行为。
 run(false,false,false,30,1); assert(!returned); // C：正常递送大端为限位。
 run(false,false,false,10,-1); assert(returned && target==82);
 run(false,false,true,30,1); assert(returned && target==60);
 run(false,true,false,20,0,70); assert(!returned); // D：软限位早于窗口，不跨越触发。
 check_guards();
 std::cout << "PASS: actual dual-handle branch A-D\n";
}
'''
protection = r'''
bool cancelled=false;
bool cancel_active_return_motion(bool){cancelled=true;return true;}
void check_guards(){
 struct {int clamp_stage=0,restore_stage=0;bool active(){return true;}} planned_return;
 struct {int cyl3_open=450,cyl3_clamp=50,cyl4_open=5,cyl4_clamp=500;} cyl;
 int cylinder3_cmd=0,cylinder4_cmd=0;bool restore_clamps=false;
 auto output=[&](){
''' + clamps + r'''
 };
 output();assert(cylinder3_cmd==50 && cylinder4_cmd==500);
 planned_return.clamp_stage=1;output();assert(cylinder3_cmd==50 && cylinder4_cmd==5);
 restore_clamps=true;output();assert(cylinder3_cmd==50 && cylinder4_cmd==500);
 planned_return.restore_stage=1;output();assert(cylinder3_cmd==450 && cylinder4_cmd==500);
 bool control_active=true,connection_hold_active=false,estop_hold_active=false,return_ads_fault_hold=false,
 startup_sequence_active=false,axis6_soft_limit_hold=false;
 struct {bool requested=false;bool active(){return false;}} spacing_recovery;
 auto safety=[&](){cancelled=false;
''' + guard + r'''
 };
 safety();assert(!cancelled);
 bool* faults[]={&connection_hold_active,&estop_hold_active,&return_ads_fault_hold,&startup_sequence_active,&axis6_soft_limit_hold};
 for(auto f:faults){*f=true;safety();assert(cancelled);*f=false;}
 control_active=false;safety();assert(cancelled);
 std::cout<<"PASS: actual clamp outputs and stop/estop/soft-limit cancellation guard\n";
}
'''
# 状态机仍等待输出实际落地以及原有夹紧/释放等待时间。
assert 'planned_return.clamp_output_applied &&\n\t\t\t\t\t\t(now_ms - planned_return.phase_t0_ms) >= clamp_wait_ms' in source
assert '? cfg.axis_clamp_wait_ms : cfg.axis_release_wait_ms' in source
vcvars = Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2019\Professional\VC\Auxiliary\Build\vcvars64.bat')
with tempfile.TemporaryDirectory(prefix='axis4-check-') as tmp:
    tmp = Path(tmp)
    (tmp / 'check.cpp').write_text(prefix + branch + suffix + protection, encoding='utf-8')
    command = f'@call "{vcvars}" >nul\n@cl /nologo /EHsc /utf-8 check.cpp /Fe:check.exe\n@if errorlevel 1 exit /b 1\n@check.exe\n'
    (tmp / 'check.cmd').write_text(command, encoding='ascii')
    subprocess.run(['cmd', '/d', '/c', 'check.cmd'], cwd=tmp, check=True)
