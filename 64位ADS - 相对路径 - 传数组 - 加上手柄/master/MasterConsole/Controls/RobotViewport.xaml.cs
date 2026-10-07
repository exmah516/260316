using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Threading;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Threading;
using System.Xml;
using Kitware.VTK;
using Urdf;

namespace MasterConsole.Controls
{
    /// <summary>已经转换到旧 URDF 显示坐标的完整 DSA 机械臂快照。</summary>
    public sealed class DsaArmDisplayState
    {
        /// <summary>sj1/sj2/sj6 为米；其余 sj 为弧度；顺序固定为 sj1..sj7。</summary>
        public double[] Sj { get; }
        /// <summary>xj1/xj2 均为弧度；顺序固定为 xj1..xj2。</summary>
        public double[] Xj { get; }
        /// <summary>旧 ct.v1 原值，单位未知时不改名、不换算。</summary>
        public double V1 { get; }
        /// <summary>旧 ct.v2 原值，单位未知时不改名、不换算。</summary>
        public double V2 { get; }
        public long LocalReceiveTimestamp { get; internal set; }

        public DsaArmDisplayState(double[] sj, double[] xj, double v1, double v2)
        {
            Sj = sj;
            Xj = xj;
            V1 = v1;
            V2 = v2;
        }
    }

    /// <summary>旧 DSA URDF 的静态和离线状态显示，不接入 ADS 或运动命令。</summary>
    public partial class RobotViewport : UserControl, IDisposable
    {
        private const int StateTimeoutMs = 1000;
        private readonly UrdfData _urdf = new UrdfData();
        private readonly DispatcherTimer _stateTimer;
        private bool _loaded;
        private bool _disposed;
        private bool _modelReady;
        private bool _testPose;
        private int _testPoseIndex;
        private vtkRenderer _renderer;
        private RenderWindowControl ModelShow;
        private PendingState _pendingState;
        private DsaArmDisplayState _lastState;
        private long _lastStateTimestamp;

        private sealed class PendingState
        {
            public DsaArmDisplayState State;
            public bool IsTest;
        }

        public RobotViewport()
        {
            InitializeComponent();
            ModelShow = new RenderWindowControl();
            FormsHost.Child = ModelShow;
            _stateTimer = new DispatcherTimer(DispatcherPriority.Render)
            {
                Interval = TimeSpan.FromMilliseconds(50)
            };
            _stateTimer.Tick += StateTimer_Tick;
            Loaded += RobotViewport_Loaded;
        }

        /// <summary>发布已转换的完整快照；返回 false 表示输入非法且未覆盖旧姿态。</summary>
        public bool PublishDsaArmState(DsaArmDisplayState state)
        {
            return PublishState(state, false);
        }

        /// <summary>仅供人工离线验收；明确标记为测试数据。</summary>
        public bool PublishTestDsaArmState(DsaArmDisplayState state)
        {
            return PublishState(state, true);
        }

        private bool PublishState(DsaArmDisplayState state, bool isTest)
        {
            if (_disposed || state == null || !IsValidState(state)) return false;

            var copy = new DsaArmDisplayState(
                (double[])state.Sj.Clone(),
                (double[])state.Xj.Clone(),
                state.V1,
                state.V2)
            {
                LocalReceiveTimestamp = Stopwatch.GetTimestamp()
            };
            Interlocked.Exchange(ref _pendingState, new PendingState
            {
                State = copy,
                IsTest = isTest
            });
            return true;
        }

        private static bool IsValidState(DsaArmDisplayState state)
        {
            if (state.Sj == null || state.Sj.Length != 7 || state.Xj == null || state.Xj.Length != 2)
                return false;
            for (int i = 0; i < state.Sj.Length; i++)
                if (double.IsNaN(state.Sj[i]) || double.IsInfinity(state.Sj[i])) return false;
            for (int i = 0; i < state.Xj.Length; i++)
                if (double.IsNaN(state.Xj[i]) || double.IsInfinity(state.Xj[i])) return false;
            return !double.IsNaN(state.V1) && !double.IsInfinity(state.V1)
                && !double.IsNaN(state.V2) && !double.IsInfinity(state.V2);
        }

        private void RobotViewport_Loaded(object sender, RoutedEventArgs e)
        {
            if (_loaded || _disposed) return;
            _loaded = true;
            try
            {
                string baseDir = AppDomain.CurrentDomain.BaseDirectory;
                string urdfPath = Path.Combine(baseDir, "URDF", "urdf.urdf");
                string meshDir = Path.Combine(baseDir, "URDF");
                if (!File.Exists(urdfPath))
                    throw new FileNotFoundException("未找到 URDF 入口", urdfPath);
                if (!Directory.Exists(Path.Combine(meshDir, "urdf", "meshes")))
                    throw new DirectoryNotFoundException("未找到 URDF 网格目录：" + Path.Combine(meshDir, "urdf", "meshes"));
                VerifyMeshFiles(urdfPath, meshDir);

                if (!_urdf.Load(urdfPath))
                    throw new InvalidDataException("URDF 加载返回失败");

                _renderer = ModelShow.RenderWindow.GetRenderers().GetFirstRenderer();
                if (_renderer == null)
                    throw new InvalidOperationException("VTK renderer 未创建");

                ViewX();
                _urdf.AddModel(meshDir, _renderer);
                _urdf.Init();
                ApplyDefaultPose();
                ModelShow.RenderWindow.Render();
                _modelReady = true;
                _stateTimer.Start();
                StatusText.Text = "旧模型默认姿态 / 未接入 DSA 机械臂";
            }
            catch (Exception ex)
            {
                StatusText.Text = "模型不可用：" + ex.GetType().Name + " - " + ex.Message;
                StatusText.Foreground = System.Windows.Media.Brushes.IndianRed;
            }
        }

        private void StateTimer_Tick(object sender, EventArgs e)
        {
            if (!_modelReady || _disposed) return;

            var pending = Interlocked.Exchange(ref _pendingState, null);
            if (pending != null)
            {
                ApplyState(pending.State);
                _lastState = pending.State;
                _lastStateTimestamp = pending.State.LocalReceiveTimestamp;
                _testPose = pending.IsTest;
                UpdateStateText(IsExpired(_lastStateTimestamp));
                ModelShow.RenderWindow.Render();
                return;
            }

            if (_lastState != null)
                UpdateStateText(IsExpired(_lastStateTimestamp));
        }

        private void ApplyState(DsaArmDisplayState state)
        {
            for (int i = 0; i < 7; i++) _urdf.MoveJoint("sj" + (i + 1), state.Sj[i]);
            _urdf.MoveJoint("xj1", state.Xj[0]);
            _urdf.MoveJoint("xj2", state.Xj[1]);
            _urdf.UpdateMove();
        }

        private void UpdateStateText(bool expired)
        {
            string source = _testPose ? "测试数据" : "DSA机械臂状态";
            int age = FormatAge(_lastStateTimestamp);
            string state = expired ? "过期（保留最后姿态）" : "在线";
            StatusText.Text = string.Format("{0}：{1}；帧龄 {2} ms；ct v1={3:0.###}, v2={4:0.###}",
                source, state, age, _lastState.V1, _lastState.V2);
            StatusText.Foreground = expired
                ? System.Windows.Media.Brushes.Gold
                : System.Windows.Media.Brushes.LightGreen;
        }

        private bool IsExpired(long timestamp)
        {
            return FormatAge(timestamp) > StateTimeoutMs;
        }

        private int FormatAge(long timestamp)
        {
            long delta = Stopwatch.GetTimestamp() - timestamp;
            if (delta <= 0) return 0;
            return (int)(delta * 1000L / Stopwatch.Frequency);
        }

        private static void VerifyMeshFiles(string urdfPath, string baseDir)
        {
            var doc = new XmlDocument();
            doc.Load(urdfPath);
            var seen = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            foreach (XmlNode node in doc.SelectNodes("//*[local-name()='mesh']"))
            {
                var file = node.Attributes?["filename"]?.Value;
                if (string.IsNullOrWhiteSpace(file)) continue;
                int marker = file.IndexOf("//", StringComparison.Ordinal);
                string relative = marker >= 0 ? file.Substring(marker + 2) : file;
                string path = Path.GetFullPath(Path.Combine(baseDir, relative.Replace('/', Path.DirectorySeparatorChar)));
                if (seen.Add(path) && !File.Exists(path))
                    throw new FileNotFoundException("URDF 网格文件缺失：" + path, path);
            }
        }

        private void ApplyDefaultPose()
        {
            // 旧 MsStatus.GetJointValue 无数据时的离线默认：9 个动态关节和 v1/v2 均为 0。
            for (int i = 1; i <= 7; i++) _urdf.MoveJoint("sj" + i, 0.0);
            _urdf.MoveJoint("xj1", 0.0);
            _urdf.MoveJoint("xj2", 0.0);
            _urdf.UpdateMove();
        }

        private void PublishTest_Click(object sender, RoutedEventArgs e)
        {
            if (!RunOfflineContractCheck())
            {
                StatusText.Text = "接口自检失败";
                StatusText.Foreground = System.Windows.Media.Brushes.IndianRed;
                return;
            }

            int joint = _testPoseIndex++ % 9;
            var sj = new double[7];
            var xj = new double[2];
            double value = 0.1 + _testPoseIndex * 0.01;
            if (joint < sj.Length) sj[joint] = value;
            else xj[joint - sj.Length] = value;
            PublishTestDsaArmState(new DsaArmDisplayState(sj, xj, _testPoseIndex, -_testPoseIndex));
        }

        private bool RunOfflineContractCheck()
        {
            if (PublishDsaArmState(new DsaArmDisplayState(new double[6], new double[2], 0, 0))) return false;
            var bad = new double[7];
            bad[3] = double.NaN;
            if (PublishDsaArmState(new DsaArmDisplayState(bad, new double[2], 0, 0))) return false;

            var sj = new[] { 1.0, 0, 0, 0, 0, 0, 0 };
            var xj = new double[2];
            if (!PublishDsaArmState(new DsaArmDisplayState(sj, xj, 0, 0))) return false;
            sj[0] = 99;
            var captured = Interlocked.Exchange(ref _pendingState, null);
            if (captured == null || captured.State.Sj[0] != 1.0) return false;

            long old = Stopwatch.GetTimestamp() - Stopwatch.Frequency * (StateTimeoutMs + 1) / 1000;
            return IsExpired(old) && !IsExpired(Stopwatch.GetTimestamp());
        }

        private void ResetView_Click(object sender, RoutedEventArgs e) => ViewX();

        public void ViewX()
        {
            if (_renderer == null) return;
            var camera = _renderer.GetActiveCamera();
            camera.SetPosition(5.8, 0.7, 0);
            camera.SetFocalPoint(0, 0.7, 0);
            camera.SetViewUp(0, 0, 1);
            camera.ParallelProjectionOff();
            camera.SetClippingRange(0.1, 1000);
            ModelShow.RenderWindow.Render();
        }

        public void Dispose()
        {
            if (!Dispatcher.CheckAccess())
            {
                Dispatcher.Invoke(new Action(Dispose));
                return;
            }
            if (_disposed) return;
            _disposed = true;
            _stateTimer.Stop();
            Interlocked.Exchange(ref _pendingState, null);
            try { ModelShow?.Dispose(); } catch { }
            _renderer = null;
        }
    }
}
