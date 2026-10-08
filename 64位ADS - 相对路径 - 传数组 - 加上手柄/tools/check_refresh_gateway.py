"""真实网关 TCP/UDP + main 刷新分支隔离回归；替换 PLC/设备，不启动 ADS.exe。"""
from pathlib import Path
import hashlib
import hmac
import json
import contextlib
import queue
import socket
import struct
import subprocess
import tempfile
import threading
import time

root = Path(__file__).resolve().parents[1]
source = (root / 'main.cpp').read_text(encoding='utf-8-sig')
a = source.index('case VisCommandType::RefreshHandles:')
b = source.index('case VisCommandType::RequestModeSwitch:', a)
branch = source[a:b]
prefix = r'''
#include "remote_gateway.h"
#include "remote_handle_bridge.h"
#include <iostream>
#include <chrono>
struct TestHandle {
 unsigned long sn; double fJoints2[2]={}; unsigned char buttons2=0;
 unsigned long serial(){return sn;} bool is_open(){return true;} bool init(){return true;}
 bool poll(){remote_handle_bridge::Sample s; if(!remote_handle_bridge::get_sample(sn==582?0:1,s,250))return false;
 for(int i=0;i<2;++i)fJoints2[i]=s.joints[i]; buttons2=s.buttons; return true;}
};
struct Filter {double linear=0,rot=0; void reset(double a,double b){linear=a;rot=b;} } axis1_handle_filter,axis6_handle_filter;
TestHandle handle_axis1{587},handle_axis6{582};
TestHandle *axis1_input_handle=&handle_axis1,*axis6_input_handle=&handle_axis6;
namespace motion_sync {
 bool rebase_axis1_after_return(int){return axis1_handle_filter.linear==222;}
 bool rebase_axis6_after_return(int){return axis6_handle_filter.linear==111;}
 bool rebase_dual_after_return(int){return rebase_axis1_after_return(0)&&rebase_axis6_after_return(0);}
}
bool remote_handles_active=true,handle_refresh_hold=false,refresh_cylinder5_press=false;
int remote_handle_refresh_ticket=0,remote_handle_refresh_mask=-1;
struct {bool cylinder5_press_req=true;} ads_output;
bool axis4_ui_forward_pressed=true,axis4_ui_reverse_pressed=false,axis4_axis6_coupling_active_prev=true;
int axis4_ui_jog_deadline_ms=10,axis4_axis6_coupling_last_ms=1,injector_ui_direction[2]={1,1},injector_ui_jog_deadline_ms[2]={1,1};
bool ads_motion_cycle_valid=true,return_ads_fault_hold=false,startup_sequence_active=false,emergency_retract_active=false,single_handle_mode=false;
enum class GuidewireMode { None, Independent }; auto guidewire_mode=GuidewireMode::None;
bool catheter_mode_button_pressed_prev=false,guidewire_mode_button_pressed_prev=false,axis1_fast_return=false,axis6_fast_retract=false;
struct {bool active(){return false;}} planned_return;
struct {int btn_b7=128;} cfg;
int ctx=0;
void clear_force_output(){remote_handle_bridge::clear_outputs();}
bool cancel_active_return_motion(bool){return true;}
void handle(VisCommand vcmd){switch(vcmd.type){
'''
suffix = r'''
default: break;}}
int main(int argc,char**argv){
 RemoteGateway g; RemoteGatewayConfig config;
 config.token_path="test.token"; config.tcp_port=atoi(argv[1]); config.udp_port=atoi(argv[2]);
 if(!g.start(config))return 2; g.set_handles_remote(true);
 std::atomic<int> steps{0}; std::atomic<bool> stop{false},automatic{false},skip_confirm{false};
 std::thread input([&](){std::string s;while(std::getline(std::cin,s)){
 if(s=="no-confirm"){skip_confirm=true;automatic=true;}else if(s=="auto")automatic=true;else if(s=="step")++steps;else break;}stop=true;});
 std::cout<<"READY"<<std::endl;
 while(!stop){
 if(automatic || steps>0){if(steps>0)--steps; VisCommand c;while(g.poll_command(c)){if(!(skip_confirm && c.type==VisCommandType::RefreshHandles && c.param1>0))handle(c);}
 std::cout<<"STEP "<<remote_handle_refresh_ticket<<" "<<remote_handle_refresh_mask<<" "<<handle_refresh_hold<<std::endl;}
 VisState s{};s.ads_state=2;s.control_active=true;s.startup_completed=true;s.self_check_done=true;
 RemoteExtraState e;e.initial_sync_done=true;e.handle_refresh_ticket=remote_handle_refresh_ticket;e.handle_refresh_mask=remote_handle_refresh_mask;
 g.publish_state(s,e); Sleep(10);
 }
 g.stop();input.join();return 0;
}
'''

def port():
    with socket.socket() as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]

with tempfile.TemporaryDirectory(prefix='refresh-gateway-') as tmp:
    tmp = Path(tmp)
    (tmp / 'check.cpp').write_text(prefix + branch + suffix, encoding='utf-8')
    vcvars = r'C:\Program Files (x86)\Microsoft Visual Studio\2019\Professional\VC\Auxiliary\Build\vcvars64.bat'
    (tmp / 'build.cmd').write_text(f'@call "{vcvars}" >nul\n@cl /nologo /EHsc /std:c++17 /utf-8 /I"{root}" /I"{root / "protocol/cpp"}" /I"{root / "手柄"}" check.cpp "{root / "remote_gateway.cpp"}" /Fe:check.exe\n', encoding='utf-8-sig')
    # cmd 只在这个临时目录构建，源文件均只读。
    subprocess.run('cmd /d /c "chcp 65001 >nul & build.cmd"', cwd=tmp, check=True)
    tcp_port, udp_port = port(), port()
    token = b'isolated-regression-token'
    (tmp / 'test.token').write_bytes(token)
    process = subprocess.Popen([str(tmp / 'check.exe'), str(tcp_port), str(udp_port)], cwd=tmp,
                               stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               creationflags=subprocess.CREATE_NO_WINDOW)
    lines = queue.Queue()
    def reader():
        for line in process.stdout:
            lines.put(line.decode('utf-8', errors='replace').strip())
    threading.Thread(target=reader, daemon=True).start()
    def line_until(prefix):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            line = lines.get(timeout=5)
            if line.startswith(prefix): return line
        raise AssertionError(prefix)
    def step():
        process.stdin.write(b'step\n'); process.stdin.flush()
        return line_until('STEP ')
    try:
        line_until('READY')
        with socket.create_connection(('127.0.0.1', tcp_port)) as tcp, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
            tcp.settimeout(.25)
            def send(obj):
                body=json.dumps(obj).encode(); tcp.sendall(struct.pack('<I',len(body))+body)
            def exact(n):
                result=b''
                while len(result)<n:
                    block=tcp.recv(n-len(result)); assert block
                    result+=block
                return result
            def receive(): return json.loads(exact(struct.unpack('<I',exact(4))[0]))
            def ack(id,state):
                deadline=time.monotonic()+3
                while time.monotonic()<deadline:
                    try: message=receive()
                    except socket.timeout: continue
                    if message.get('id')==id and message.get('t')=='ack':
                        assert message['state']==state,message
                        return message
                raise AssertionError(f'ack {id} {state}')
            def no_done(id):
                try:
                    while True:
                        m=receive()
                        assert not(m.get('id')==id and m.get('state')=='done'),m
                except socket.timeout: pass
            nc=bytes(range(16))
            send(dict(t='hello',proto=1,nonce_c=nc.hex()))
            ns=bytes.fromhex(receive()['nonce_s'])
            send(dict(t='auth',mac=hmac.new(token,b'auth'+nc+ns,hashlib.sha256).hexdigest()))
            hello=receive(); session=hello['session']; key=hmac.new(token,b'key'+nc+ns,hashlib.sha256).digest()[:16]
            send(dict(t='acquire',id=1)); ack(1,'done')
            def frame(seq,mask=3):
                header=struct.pack('<HBBIII',0x4956,1,1,session,seq,seq)
                samples=b''.join(struct.pack('<BBiiffff',0,(mask>>i)&1,0,0,111 if i==0 else 222,0,0,0) for i in range(2))
                body=header+samples+struct.pack('<bbb',0,0,-1)
                udp.sendto(body+hmac.new(key,body,hashlib.sha256).digest()[:8],('127.0.0.1',udp_port))
            @contextlib.contextmanager
            def stream(first, mask=3):
                stop = threading.Event()
                def send_frames():
                    seq=first
                    while not stop.is_set():
                        frame(seq,mask); seq+=1; stop.wait(.01)
                sender=threading.Thread(target=send_frames);sender.start()
                try: yield
                finally: stop.set();sender.join()
            send(dict(t='cmd',id=2,name='refresh_handles_begin')); ack(2,'accepted')
            no_done(2)
            assert step().endswith('-1 1')
            ack(2,'done')
            print('PASS: begin done only after main hold')
            send(dict(t='cmd',id=3,name='refresh_handles',success_mask=3,after_seq=100)); ack(3,'accepted')
            frame(100)
            no_done(3); assert step().endswith('-1 1')
            print('PASS: stale UDP sequence cannot rebuild baseline')
            with stream(101):
                no_done(3) # 即使新采样已到，命令尚未被主循环执行也不能 done。
                assert step().endswith('3 0')
                ack(3,'done')
            print('PASS: fresh UDP + real main branch rebase required, 587/582 roles correct')
            send(dict(t='cmd',id=4,name='refresh_handles_begin')); ack(4,'accepted'); step(); ack(4,'done')
            send(dict(t='cmd',id=5,name='refresh_handles',success_mask=1,after_seq=1000)); ack(5,'accepted')
            with stream(1001,1):
                no_done(5); assert step().endswith('1 1'); ack(5,'done')
            print('PASS: partial baseline remains in hold, no automatic single-handle fallback')
            send(dict(t='cmd',id=6,name='refresh_handles_begin')); ack(6,'accepted'); step(); ack(6,'done')
            send(dict(t='cmd',id=7,name='refresh_handles',success_mask=3,after_seq=2000)); ack(7,'accepted')
            send(dict(t='cmd',id=8,name='refresh_handles_begin')); ack(8,'rejected')
            # 无有效新采样；持续发无效帧以保持租约，确认下游超时不会变成成功。
            deadline=time.monotonic()+5; seq=2001; result=None
            while time.monotonic()<deadline:
                frame(seq,0); seq+=1
                try:
                    m=receive()
                    if m.get('id')==7 and m.get('state')!='accepted': result=m;break
                except socket.timeout: pass
            assert result and result['state']=='rejected',result
            assert step().endswith('-1 1')
            print('PASS: repeated refresh rejected; missing fresh sample times out without resuming')
        process.stdin.write(b'auto\n'); process.stdin.flush()
        subprocess.run(['dotnet','run','--project',str(root/'tools/HandleRefreshCheck/HandleRefreshCheck.csproj'),'--','--link',str(tcp_port)],check=True)
        process.stdin.write(b'no-confirm\n'); process.stdin.flush()
        subprocess.run(['dotnet','run','--project',str(root/'tools/HandleRefreshCheck/HandleRefreshCheck.csproj'),'--','--link-budget',str(tcp_port)],check=True)
    finally:
        process.stdin.write(b'quit\n'); process.stdin.flush()
        try: process.wait(timeout=5)
        except subprocess.TimeoutExpired: process.kill(); process.wait()
