"""在 --sim 模式离屏加载真实 WPF 窗口和 VTK 模型，验证布局、绑定与模型更新。"""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
output = root / 'master/MasterConsole/bin/Debug/net472'
code = r'''
using System;
using System.Diagnostics;
using System.Reflection;
using System.Threading;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Media;
using System.Windows.Threading;
using MasterConsole;
using MasterConsole.Views;
using MasterConsole.Controls;
class LayoutCheck {
 class BindingErrors : TraceListener {
  public int Count;
  public override void Write(string s){}
  public override void WriteLine(string s){if(s.Contains("Error:")){Count++;Console.WriteLine(s);}}
 }
 static void Check(bool ok,string s){if(!ok)throw new Exception(s);Console.WriteLine("PASS: "+s);}
 static void Pump(int ms){var t=Stopwatch.StartNew();do{var frame=new DispatcherFrame();
 Dispatcher.CurrentDispatcher.BeginInvoke(DispatcherPriority.Background,new Action(()=>frame.Continue=false));
 Dispatcher.PushFrame(frame);Thread.Sleep(5);}while(t.ElapsedMilliseconds<ms);}
 static int CountMessages(DependencyObject o){int n=0;var g=o as GroupBox;
 if(g!=null && Convert.ToString(g.Header)=="消息")n++;
 for(int i=0;i<VisualTreeHelper.GetChildrenCount(o);i++)n+=CountMessages(VisualTreeHelper.GetChild(o,i));return n;}
 static TextBlock RefreshText(DependencyObject o){var t=o as TextBlock;
 if(t!=null){var b=BindingOperations.GetBinding(t,TextBlock.TextProperty);if(b!=null && b.Path.Path=="HandleRefreshText")return t;}
 for(int i=0;i<VisualTreeHelper.GetChildrenCount(o);i++){var found=RefreshText(VisualTreeHelper.GetChild(o,i));if(found!=null)return found;}return null;}
 [STAThread] static int Main(){MainWindow w=null;try{
 var app=new Application();
 app.Resources.MergedDictionaries.Add(new ResourceDictionary{Source=new Uri("/MasterConsole;component/Themes/LightClinical.xaml",UriKind.Relative)});
 var errors=new BindingErrors();PresentationTraceSources.DataBindingSource.Listeners.Add(errors);
 PresentationTraceSources.DataBindingSource.Switch.Level=SourceLevels.Error;
 w=new MainWindow();w.WindowStartupLocation=WindowStartupLocation.Manual;w.Left=-20000;w.Top=-20000;w.ShowInTaskbar=false;
 w.Show();Pump(500);
 var model=(RobotViewport)w.FindName("RobotModelViewport");
 Check(CountMessages(w)==0 && w.FindName("DisplayLogColumn")==null,"message controls and log column removed");
 var flags=BindingFlags.Instance|BindingFlags.NonPublic;
 Check((bool)typeof(RobotViewport).GetField("_modelReady",flags).GetValue(model),"real URDF/VTK model loaded offline");
 foreach(double width in new[]{1480.0,1180.0,1800.0}){
  w.Width=width;w.Height=width==1180?720:880;w.UpdateLayout();Pump(50);
  var parent=(FrameworkElement)VisualTreeHelper.GetParent(model);
  Check(Math.Abs(model.ActualWidth-parent.ActualWidth)<1,"model uses full display width at "+width+" ("+model.ActualWidth+" DIP)");
  Check(model.ActualHeight>100,"model height remains usable");
 }
 var vm=w.DataContext;
 vm.GetType().GetField("_handleRefreshText",flags).SetValue(vm,"正在刷新…");
 w.DataContext=null;w.DataContext=vm;w.UpdateLayout();Pump(50);
 var feedback=RefreshText(w);var feedbackParent=(FrameworkElement)VisualTreeHelper.GetParent(feedback);
 Check(feedback.ActualWidth>100 && feedback.ActualWidth<feedbackParent.ActualWidth && feedback.TextWrapping==TextWrapping.Wrap,
       "refresh feedback has bounded wrapping width next to button");
 typeof(MainWindow).GetMethod("SetDisplayLayout",flags).Invoke(w,new object[]{"model"});w.UpdateLayout();Pump(50);
 Check(Math.Abs(model.ActualWidth-((FrameworkElement)VisualTreeHelper.GetParent(model)).ActualWidth)<1,"model emphasis layout also has no empty log column");
 Check(model.PublishDsaArmState(new DsaArmDisplayState(new double[7],new double[2],0,0)),"model state input still accepted");
 var renderer=typeof(RobotViewport).GetField("_renderer",flags).GetValue(model);
 var camera=renderer.GetType().GetMethod("GetActiveCamera",Type.EmptyTypes).Invoke(renderer,null);
 foreach(string method in new[]{"Azimuth","Elevation","Zoom"})
  camera.GetType().GetMethod(method,new[]{typeof(double)}).Invoke(camera,new object[]{method=="Zoom"?1.1:5.0});
 typeof(RobotViewport).GetMethod("ResetView_Click",flags).Invoke(model,new object[]{null,new RoutedEventArgs()});
 Check(true,"VTK rotation/zoom and existing reset-view handler remain callable");
 Pump(100);
 Check(typeof(RobotViewport).GetField("_lastState",flags).GetValue(model)!=null,"model timer applies incoming state");
 Check(errors.Count==0,"no WPF binding errors at default and resized layouts");
 w.Close();return 0;
 }catch(Exception e){Console.WriteLine("FAIL: "+e);if(w!=null)w.Close();return 1;}}
}
'''
csc = Path(r'C:\Windows\Microsoft.NET\Framework64\v4.0.30319\csc.exe')
wpf = Path(r'C:\Windows\Microsoft.NET\Framework64\v4.0.30319\WPF')
exe = output / 'IsolatedLayoutCheck.exe'
with tempfile.TemporaryDirectory(prefix='master-layout-') as temp:
    source = Path(temp) / 'check.cs'
    source.write_text(code, encoding='utf-8-sig')
    refs = [output/'MasterConsole.exe', wpf/'PresentationFramework.dll', wpf/'PresentationCore.dll', wpf/'WindowsBase.dll', Path(r'C:\Windows\Microsoft.NET\Framework64\v4.0.30319\System.Xaml.dll')]
    subprocess.run([str(csc),'/nologo','/platform:x64','/out:'+str(exe)]+['/r:'+str(p) for p in refs]+[str(source)],check=True)
    try:
        result = subprocess.run([str(exe),'--sim'],cwd=output,check=False,timeout=45,
                                creationflags=subprocess.CREATE_NO_WINDOW, capture_output=True, text=True)
        print(result.stdout, end='')
        print(result.stderr, end='')
        if result.returncode: raise RuntimeError('layout check exit '+str(result.returncode))
    finally:
        exe.unlink(missing_ok=True)
