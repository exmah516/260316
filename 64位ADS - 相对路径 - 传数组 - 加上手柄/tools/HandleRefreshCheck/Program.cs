using System;
using System.Collections.Generic;
using System.Threading;
using System.Linq;
using System.Threading.Tasks;
using MasterConsole.Protocol;
using MasterConsole.Services;

namespace MasterConsole.Services
{
    // 同名模拟 SDK：测试编译真实 HandleService，不加载硬件 DLL。
    internal static class FlCatheterNative
    {
        public delegate int ServoCallback(IntPtr p);
        public static readonly List<string> Calls = new List<string>();
        public static bool Running;
        public static uint FailSerial;
        public static int Transient587;
        public static bool FailStart, FailStop, BlockStop, FailLoad, FailBind, RefuseUnload, FailClose;
        public static bool IsLoaded { get; private set; }
        public static int InvalidSamples, StopDelayMs;
        public static void Load() { if(IsLoaded)return; Record("load"); if(FailLoad)throw new Exception("加载失败：mock"); if(FailBind)throw new Exception("绑定失败：mock"); IsLoaded=true; }
        public static void Unload() { Record("unload-check"); if(RefuseUnload)throw new Exception("卸载未执行：未确认线程退出"); Record("unload"); IsLoaded=false; }
        public static double Pose=10;
        public static bool NetworkPose;
        public static readonly ManualResetEventSlim Stopping=new ManualResetEventSlim(false);
        public static readonly ManualResetEventSlim Continue=new ManualResetEventSlim(false);
        public static readonly HashSet<int> Threads=new HashSet<int>();
        public static void Record(string s) { lock (Calls) { Calls.Add(s); Threads.Add(Thread.CurrentThread.ManagedThreadId); } }
        public static int openDevice(uint sn) { Record("open:"+sn); if(sn==587 && Transient587>0) { --Transient587; return -1; } return sn==FailSerial?-1:sn==582?0:1; }
        public static void closeDevice() { Record("close"); if(FailClose)throw new Exception("mock close"); }
        public static int startServoLoop(ServoCallback cb, IntPtr p) { Record("start"); Running=!FailStart; return FailStart?-1:0; }
        public static int stopServoLoop() {
            Record("stop");
            if(StopDelayMs>0) Thread.Sleep(StopDelayMs);
            if(BlockStop) { Stopping.Set(); Continue.Wait(); }
            if(FailStop) return -1;
            Running=false; return 0;
        }
        public static bool isServoLoopRunning() => Running;
        public static int getSerialNumber(int id) => id==0?582:587;
        public static int deviceStatus(int id) => 0;
        public static void enableForces(bool en,int id) { Record("enable:"+en); }
        public static void sendForce(double[] f,double t,int id) { Record(f.Any(x=>x!=0)||t!=0?"force":"zero"); }
        public static void getEncVel(double[] x,int id) { }
        public static void getJoints(double[] x,int id) { if(InvalidSamples>0) { --InvalidSamples; x[0]=double.NaN; return; } x[0]=NetworkPose?(id==0?111:222):id+Pose; }
        public static void getEncoders(int[] x,int id) { }
        public static void getSwitch(out byte b,int id) { b=0; }
    }
}
class Program
{
    static void Check(bool value,string message) { if(!value) throw new Exception(message); Console.WriteLine("PASS: "+message); }
    static void Wait(Func<bool> condition,string message) { Check(SpinWait.SpinUntil(condition,2000),message); }
    static string Calls() { lock(FlCatheterNative.Calls) return string.Join(",",FlCatheterNative.Calls); }
    static void Clear() { lock(FlCatheterNative.Calls) FlCatheterNative.Calls.Clear(); }
    static int Main(string[] args)
    {
        try {
        if(args.Length==2 && args[0].StartsWith("--link")) { CheckLink(int.Parse(args[1]),args[0]=="--link-budget").GetAwaiter().GetResult(); return 0; }
        using(var service=new HandleService())
        {
            Wait(()=>service.GetSample(0).Valid && service.GetSample(1).Valid,"two stationary handles valid");
            service.SetHaptic(0,new HapticOut{Enable=true,ForceN=1});
            Wait(()=>Calls().Contains("force"),"normal haptic delivered");
            Clear();
            service.PauseInput();
            Wait(()=>Calls().Contains("zero"),"pause clears actual force before remote hold acknowledgement");
            Check(!service.GetSample(0).Valid && !Calls().Contains("stop"),"input paused while SDK still awaits hold acknowledgement");
            Clear();
            FlCatheterNative.Pose=20;
            Check(service.RefreshNow(2000)==3,"both devices reinitialized");
            Check(Calls().Contains("zero,enable:False,zero,enable:False,stop,close,unload-check,unload,load,open:582,open:587,start"),"zero/disable both before stop-close-unload-load-open both-start");
            Check(!service.GetSample(0).Valid && service.GetSample(0,true).Joint0==20,"fresh pose available only for baseline handshake");
            Check(!Calls().Contains(",force"),"old force cleared");
            service.ResumeInput();
            Check(service.GetSample(0).Valid,"input resumes explicitly");

            FlCatheterNative.Transient587=1;
            Clear();
            Check(service.RefreshNow(4000)==3,"transient open failure retried within same refresh");
            Check(Calls().Split(',').Count(x=>x=="open:582")==1 && Calls().Split(',').Count(x=>x=="open:587")==2,"only failed device retried");
            FlCatheterNative.FailSerial=587;
            Clear();
            Check(service.RefreshNow(4000)==1,"partial open failure reported per serial");
            Check(Calls().Split(',').Count(x=>x=="open:587")==3 && Calls().Split(',').Count(x=>x=="open:582")==1,"finite retries retain opened device");
            Check(service.RefreshDetails.Contains("587：打开失败（-1）（打开 3 次）") && service.RefreshDetails.Contains("582：有效新采样"),"independent results and attempts preserved");
            Check(service.IsOnline(0) && !service.IsOnline(1) && !service.GetSample(1,true).Valid,"failed device offline, no stale sample");
            FlCatheterNative.FailSerial=0;
            FlCatheterNative.FailStart=true;
            Check(service.RefreshNow(2000)==0 && !service.IsOnline(0) && !FlCatheterNative.Running,"servo start failure is not online");
            FlCatheterNative.FailStart=false;
            Check(service.RefreshNow(2000)==3,"retry recovers");
            FlCatheterNative.FailStop=true;
            Clear();
            Check(service.RefreshNow(2000)==0 && !Calls().Contains("open:"),"stop failure prevents reopening");
            FlCatheterNative.FailStop=false;
            Check(!Calls().Contains("unload"),"failed stop never unloads");
            FlCatheterNative.RefuseUnload=true; Clear();
            Check(service.RefreshNow(2000)==0 && Calls().Contains("unload-check") && !Calls().Split(',').Contains("unload") && !Calls().Contains("open:"),"unknown thread termination refuses unload and reopen");
            FlCatheterNative.RefuseUnload=false;
            FlCatheterNative.FailClose=true; Clear();
            Check(service.RefreshNow(2000)==0 && !Calls().Contains("unload"),"close exception prevents unload");
            FlCatheterNative.FailClose=false;
            FlCatheterNative.FailLoad=true;
            Check(service.RefreshNow(2000)==0 && service.RefreshDetails.Contains("加载失败"),"load failure reported");
            FlCatheterNative.FailLoad=false; FlCatheterNative.FailBind=true;
            Check(service.RefreshNow(2000)==0 && service.RefreshDetails.Contains("绑定失败"),"binding failure reported");
            FlCatheterNative.FailBind=false;
            FlCatheterNative.InvalidSamples=6;
            Check(service.RefreshNow(2000)==3,"transient invalid samples wait then recover");
            FlCatheterNative.FailSerial=587;
            var watch=System.Diagnostics.Stopwatch.StartNew();
            Check(service.RefreshNow(150)==-2 && watch.ElapsedMilliseconds<600,"retry interval bounded by shared device budget");
            Wait(()=>!service.RefreshInProgress,"expired retry operation completes");
            Check(!service.GetSample(0,true).Valid,"budget expiry invalidates handshake sample");
            FlCatheterNative.FailSerial=0;

            FlCatheterNative.BlockStop=true;
            var timeout=Task.Run(()=>service.RefreshNow(100));
            Wait(()=>FlCatheterNative.Stopping.IsSet,"mock SDK blocked during stop");
            Check(timeout.Result==-2 && service.RefreshInProgress,"bounded timeout retains unfinished operation");
            Check(service.RefreshNow(100)==-1,"repeated click rejected while SDK still running");
            service.SetHaptic(0,new HapticOut{Enable=true,ForceN=9});
            Check(!service.GetSample(0,true).Valid,"old sample invalid even for handshake while restarting");
            Clear();
            FlCatheterNative.BlockStop=false;
            FlCatheterNative.Continue.Set();
            Wait(()=>!service.RefreshInProgress,"late SDK completion managed");
            Check(!service.GetSample(0).Valid && !Calls().Contains("force") && !Calls().Contains("unload") && !Calls().Contains("open:"),"late completion never unloads, reopens, resumes or replays force");
            Check(service.RefreshNow(2000)==3,"next refresh allowed after actual completion");
            Check(FlCatheterNative.Threads.Count==1,"SDK lifecycle, polling and force output serialized");
        }
        return 0;
        } catch(Exception ex) { Console.WriteLine("FAIL: "+ex.Message); return 1; }
    }

    static async Task CheckLink(int port, bool exhaustBudget)
    {
        FlCatheterNative.NetworkPose=true;
        var handles=new HandleService();
        using(var link=new RemoteRobotLink(new RemoteLinkSettings { Host="127.0.0.1",TcpPort=port,
            Token=System.Text.Encoding.ASCII.GetBytes("isolated-regression-token") },handles))
        {
            await link.ConnectAsync();
            Check((await link.AcquireControlAsync()).Ok,"real master acquired isolated gateway");
            if(exhaustBudget)
            {
                FlCatheterNative.StopDelayMs=7200;
                var budget=System.Diagnostics.Stopwatch.StartNew();
                var pending=link.RefreshHandlesAsync();
                while(!pending.IsCompleted && (!handles.GetSample(0,true).Valid || handles.RefreshInProgress)) await Task.Delay(5);
                Check(!pending.IsCompleted && !handles.GetSample(0).Valid && handles.GetSample(0,true).Valid,"new sample waits paused for remote baseline confirmation");
                var expired=await pending;
                FlCatheterNative.StopDelayMs=0;
                Check(!expired.Ok && expired.Reason.Contains("从端新采样/基准确认失败"),"queued command never counts as baseline success");
                Check(budget.ElapsedMilliseconds>=10000 && budget.ElapsedMilliseconds<12500,"device recovery and missing remote ack share 12 second total budget");
                Check(!handles.GetSample(0).Valid && !handles.GetSample(1).Valid,"remote confirmation timeout keeps both inputs paused");
                return;
            }
            link.SetAxis4(-1);
            var refresh=link.RefreshHandlesAsync();
            Check(!(await link.RefreshHandlesAsync()).Ok,"real master rejects concurrent click");
            var result=await refresh;
            Check(result.Ok,"real master + gateway + main branch completed: "+result.Reason);
            Check(handles.GetSample(0).Valid && handles.GetSample(1).Valid,"input resumes only after remote rebase ack");
            var axis4=typeof(RemoteRobotLink).GetField("_axis4",System.Reflection.BindingFlags.Instance|System.Reflection.BindingFlags.NonPublic);
            link.SetAxis4(-1);
            Check((int)axis4.GetValue(link)==0,"held jog cannot revive after refresh");
            link.SetAxis4(0);link.SetAxis4(-1);
            Check((int)axis4.GetValue(link)==-1,"release then new jog accepted");
            link.SetAxis4(0);
            FlCatheterNative.FailSerial=587;
            result=await link.RefreshHandlesAsync();
            Check(result.Ok && result.Reason.Contains("部分成功"),"partial result reaches master");
            Check(!handles.GetSample(0).Valid && !handles.IsOnline(1),"partial refresh remains paused");
            var sendLock=typeof(RemoteRobotLink).GetField("_writeLock",System.Reflection.BindingFlags.Instance|System.Reflection.BindingFlags.NonPublic).GetValue(link);
            using(var locked=new ManualResetEventSlim())
            using(var release=new ManualResetEventSlim())
            {
                var owner=Task.Run(()=>{lock(sendLock){locked.Set();release.Wait();}});
                Check(locked.Wait(2000),"isolated sender lock held");
                try
                {
                    var timer=System.Diagnostics.Stopwatch.StartNew();
                    var blocked=link.RefreshHandlesAsync();
                    Check(timer.ElapsedMilliseconds<500,"refresh send does not block caller thread");
                    var rejected=await blocked;
                    Check(!rejected.Ok && rejected.Reason.Contains("发送锁等待超时") && timer.ElapsedMilliseconds<5000,"send lock wait is included in refresh hold budget");
                    Check(!handles.GetSample(0).Valid,"send timeout preserves input pause");
                }
                finally { release.Set(); await owner; }
            }
        }
    }
}
