"""编译真实加载器 + 临时假 DLL；从不加载项目的硬件 SDK。"""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
vs = Path(r'C:\Program Files (x86)\Microsoft Visual Studio\2019\Professional')
native = r'''
#include <windows.h>
#define API extern "C" __declspec(dllexport)
API int openDevice(DWORD sn) { return sn == 582 ? 0 : -1; }
#ifndef BROKEN
static bool running = false;
API void closeDevice() {}
API int startServoLoop(int(__stdcall *cb)(void*), void* p) { running = true; return cb(p); }
API int stopServoLoop() { running = false; return 0; }
API bool isServoLoopRunning() { return running; }
API void enableForces(bool, int) {}
API int getSerialNumber(int) { return 582; }
API int deviceStatus(int) { return 0; }
API void sendForce(double[], double, int) {}
API void getEncoders(long x[], int) { x[0] = -123; x[1] = 456; }
API void getEncVel(double x[], int) { x[0] = 1.25; x[1] = 0; }
API void getJoints(double x[], int) { x[0] = POSE; x[1] = 0; }
API void getSwitch(unsigned char& b, int) { b = 129; }
#endif
'''
harness = r'''
using System;
using System.IO;
using System.Runtime.InteropServices;
using MasterConsole.Services;
class LoaderCheck {
 [DllImport("kernel32.dll", CharSet=CharSet.Unicode)] static extern IntPtr GetModuleHandleW(string name);
 static void Check(bool b,string s) { if(!b)throw new Exception(s); Console.WriteLine("PASS: "+s); }
 static void Fails(Action f,string s) { try { f(); } catch(Exception ex) { Check(ex.Message.Contains(s),"reported "+s); return; } throw new Exception("expected "+s); }
 static void Main() {
  Fails(FlCatheterNative.Load,"加载失败");
  File.Copy("broken.dll","FLCatheter.dll");
  Fails(FlCatheterNative.Load,"绑定失败：closeDevice");
  Check(!FlCatheterNative.IsLoaded && GetModuleHandleW("FLCatheter.dll")==IntPtr.Zero,"failed binding frees its module");
  File.Copy("first.dll","FLCatheter.dll",true);
  FlCatheterNative.Load(); var x=new double[2]; FlCatheterNative.getJoints(x,0); Check(x[0]==10,"first delegates bound");
  FlCatheterNative.Unload();
  Check(GetModuleHandleW("FLCatheter.dll")==IntPtr.Zero,"unused device layer module released");
  Fails(()=>FlCatheterNative.getJoints(x,0),"SDK 未加载");
  File.Copy("second.dll","FLCatheter.dll",true);
  FlCatheterNative.Load(); FlCatheterNative.getJoints(x,0); Check(x[0]==20,"reload uses new delegates");
  var enc=new int[2]; FlCatheterNative.getEncoders(enc,0); Check(enc[0]==-123 && enc[1]==456,"Windows long array marshaling");
  FlCatheterNative.getEncVel(x,0); Check(x[0]==1.25,"double array marshaling");
  FlCatheterNative.getSwitch(out byte b,0); Check(b==129,"switch byte marshaling");
  Check(FlCatheterNative.openDevice(582)==0 && FlCatheterNative.openDevice(587)==-1,"open return codes preserved");
  Check(FlCatheterNative.getSerialNumber(0)==582,"serial return preserved");
  Check(FlCatheterNative.startServoLoop(_=>FlCatheterNative.deviceStatus(0),IntPtr.Zero)==0 && FlCatheterNative.isServoLoopRunning(),"callback and bool marshaling");
  FlCatheterNative.enableForces(false,0); FlCatheterNative.sendForce(new double[3],0,0);
  Check(FlCatheterNative.stopServoLoop()==0 && !FlCatheterNative.isServoLoopRunning(),"mock stop returns success");
  FlCatheterNative.closeDevice();
  Fails(FlCatheterNative.Unload,"卸载未执行");
  Check(FlCatheterNative.IsLoaded && GetModuleHandleW("FLCatheter.dll")!=IntPtr.Zero,"stop flag alone never authorizes unloading used SDK");
 }
}
'''
with tempfile.TemporaryDirectory(prefix='handle-loader-') as directory:
    tmp = Path(directory)
    (tmp / 'sdk.cpp').write_text(native, encoding='utf-8')
    (tmp / 'check.cs').write_text(harness, encoding='utf-8-sig')
    commands = [f'@call "{vs / "VC/Auxiliary/Build/vcvars64.bat"}" >nul']
    for name, define in [('broken', '/DBROKEN'), ('first', '/DPOSE=10'), ('second', '/DPOSE=20')]:
        commands += [f'@cl /nologo /LD /EHsc {define} "{tmp / "sdk.cpp"}" /Fo:"{tmp / "sdk.obj"}" /Fe:"{tmp / (name + ".dll")}" /link /IMPLIB:"{tmp / (name + ".lib")}"', '@if errorlevel 1 exit /b 1']
    (tmp / 'build.cmd').write_text('\n'.join(commands), encoding='utf-8')
    subprocess.run(['cmd', '/d', '/c', str(tmp / 'build.cmd')], cwd=root, check=True)
    subprocess.run([str(vs / 'MSBuild/Current/Bin/Roslyn/csc.exe'), '/nologo', '/platform:x64',
                    '/out:' + str(tmp / 'check.exe'), str(root / 'master/MasterConsole/Services/FlCatheterNative.cs'),
                    str(tmp / 'check.cs')], cwd=root, check=True)
    subprocess.run([str(tmp / 'check.exe')], cwd=tmp, check=True)
