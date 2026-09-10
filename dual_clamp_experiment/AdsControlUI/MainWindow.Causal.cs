using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;
using System.Windows.Threading;

namespace DualClampExperimentUI
{
    public partial class MainWindow
    {
        private sealed class CurveSample
        {
            public ulong Sequence;
            public double Time, Fn, Ft, CorrectedFn, CorrectedFt;
            public double Model2Fn, Model2Ft;
            public bool Model2FnValid, Model2FtValid;
            public bool Valid;
        }
        private readonly List<CurveSample> _curvePoints = new List<CurveSample>();
        private ulong _curveCursor, _curveGeneration;
        private double _lastDrawMs;
        private bool _curveGap;
        private readonly bool[,] _model2Choices = new bool[3, 2];
        private bool _restoringModel2;
        private int _model2Mode;
        private readonly int[] _dynamicsSigns = new int[3];
        private readonly bool[] _dynamicsValidation = new bool[3];
        private bool _restoringDynamics, _appliedValidation;
        private int _dynamicsMode;
        private void RestoreDynamicsOptions()
        {
            _restoringDynamics = true;
            _dynamicsMode = CurrentMode == "guidewire" ? 2 : CurrentMode == "catheter" ? 1 : 0;
            DynamicsSign.SelectedIndex = _dynamicsSigns[_dynamicsMode];
            DynamicsValidation.IsChecked = _dynamicsValidation[_dynamicsMode];
            DynamicsConditions.IsChecked = false;
            DynamicsConditions.IsEnabled = DynamicsValidation.IsChecked == true;
            _restoringDynamics = false;
        }
        private void DynamicsOption_Changed(object sender, RoutedEventArgs e)
        {
            if (!_loaded || _restoringDynamics) return;
            _dynamicsSigns[_dynamicsMode] = DynamicsSign.SelectedIndex;
            _dynamicsValidation[_dynamicsMode] = DynamicsValidation.IsChecked == true;
            DynamicsConditions.IsChecked = false;
            DynamicsConditions.IsEnabled = DynamicsValidation.IsChecked == true;
            ResetCurveView();
        }
        private void RestoreModel2Visibility()
        {
            _restoringModel2 = true;
            _model2Mode = CurrentMode == "guidewire" ? 2 : CurrentMode == "catheter" ? 1 : 0;
            Model2ForceToggle.IsChecked = _model2Choices[_model2Mode, 0];
            Model2TorqueToggle.IsChecked = _model2Choices[_model2Mode, 1];
            _restoringModel2 = false;
        }
        private void Model2Visibility_Changed(object sender, RoutedEventArgs e)
        {
            if (!_loaded || _restoringModel2) return;
            _model2Choices[_model2Mode, 0] = Model2ForceToggle.IsChecked == true;
            _model2Choices[_model2Mode, 1] = Model2TorqueToggle.IsChecked == true;
            DrawCausalCurves();
        }
        private static bool IsCurveReplay => Environment.GetCommandLineArgs().Contains("--curve-replay");

        private void ResetCurveView()
        {
            _curveCursor = _curveGeneration = 0;
            _curveGap = false;
            _curvePoints.Clear();
            if (CausalForceLine == null) return;
            CausalForceLine.Points.Clear(); CausalTorqueLine.Points.Clear();
            Model2ForceLine.Points.Clear(); Model2TorqueLine.Points.Clear();
            if (CurrentMode != "legacy") {
                Force1Line.Points.Clear(); Torque1Line.Points.Clear();
                CausalStatusText.Text = "模型：等待数据";
            }
        }

        private void ParseCurveResponse(string response)
        {
            string[] p = response.Split('|');
            if (p.Length < 11 || p[2] != (CurrentMode == "guidewire" ? "2" : "1")) return;
            ulong generation = ulong.Parse(p[1], CultureInfo.InvariantCulture);
            if (generation != _curveGeneration) {
                ResetCurveView(); _curveGeneration = generation;
            }
            if (p[9] == "1") {
                _curveGap = true;
                _curvePoints.Clear();
            }
            foreach (string item in p[10].Split(';')) {
                if (item.Length == 0) continue;
                string[] f = item.Split(',');
                if (f.Length != 8 && f.Length != 12) continue;
                var s = new CurveSample {
                    Sequence = ulong.Parse(f[0], CultureInfo.InvariantCulture),
                    Time = D(f[1]), Fn = D(f[2]), Ft = D(f[3]),
                    CorrectedFn = D(f[4]), CorrectedFt = D(f[5]), Valid = f[6] == "1",
                    Model2Fn = f.Length == 12 ? D(f[8]) : D(f[2]),
                    Model2Ft = f.Length == 12 ? D(f[9]) : D(f[3]),
                    Model2FnValid = f.Length == 12 && f[10] == "1",
                    Model2FtValid = f.Length == 12 && f[11] == "1"
                };
                if (s.Sequence <= _curveCursor) continue;
                _curveCursor = s.Sequence;
                if (new[] {s.Time, s.Fn, s.Ft, s.CorrectedFn, s.CorrectedFt, s.Model2Fn, s.Model2Ft}.Any(
                    x => double.IsNaN(x) || double.IsInfinity(x))) continue;
                // Never draw a connecting segment over a missing acquisition interval.
                if (_curvePoints.Count > 0 && (s.Time <= _curvePoints.Last().Time ||
                    s.Time - _curvePoints.Last().Time > .003000001)) {
                    _curvePoints.Clear(); _curveGap = true;
                }
                _curvePoints.Add(s);
            }
            if (_curvePoints.Count > 0) {
                double oldest = _curvePoints.Last().Time - 10;
                int trim = _curvePoints.FindIndex(s => s.Time >= oldest);
                if (trim > 0) _curvePoints.RemoveRange(0, trim);
                if (_curvePoints.Count > 10001) _curvePoints.RemoveRange(0, _curvePoints.Count - 10001);
            }
            bool dynamics = p.Length > 11 && p[11] == "dynamics_25g";
            string sign = p.Length > 12 && D(p[12]) < 0 ? "-1" : "+1";
            _appliedValidation = dynamics && p.Length > 13 && p[13] == "1";
            CausalForceToggle.Content = dynamics ? "25 g 惯性试算" : "模型处理";
            CausalTorqueToggle.Content = dynamics ? "ft 原值（未补偿）" : "模型处理";
            string state = p[4] != "1" ? "未取零，未补偿" :
                p[3] != "1" ? "模型配置无效或条件待确认" :
                _curvePoints.Count == 0 ? "试验模型，等待采样" :
                !_curvePoints.Last().Valid ? "输入无效，未补偿" :
                dynamics ? "25 g · 符号" + sign + "未验证\n反馈及同步待验证 · " + (_appliedValidation ? "无器械全程" : "操作门控") :
                p.Length > 11 && p[11] == "borrowed_guidewire" ? "借用导丝参数 · 未验证，仅作对比" : "试验模型，仅作曲线对比";
            CausalStatusText.Text = string.Format(CultureInfo.InvariantCulture,
                "{0}\n单点≤{1:F2} μs · 块跨度 {2:F0} ms · 绘图 {3:F1} ms{4}{5}",
                state, D(p[7]), D(p[8]), _lastDrawMs,
                _curveGap ? " · 曲线缺段" : "", p[5] == "1" ? "" : " · 记录错误");
            CausalStatusText.ToolTip = "模型版本：" + p[6] + "\n块跨度不是完整端到端延迟；原信号和处理信号共用采样时间。"
                + (dynamics ? "\n传感器预测=s*0.025*a/1000 N；显示增量=安装增益*传感器预测，不加截距。"
                    + "\n直接反馈加速度，无差分、滤波或时间平移；ft不补偿。数值有效不代表物理模型已验证。"
                    + "\n状态：" + (p.Length > 14 ? p[14] : "") + "；重置：" + (p.Length > 15 ? p[15] : "") : "");
            DrawCausalCurves();
        }

        private void CausalVisibility_Changed(object sender, RoutedEventArgs e)
        {
            if (!_loaded) return;
            DrawCausalCurves();
        }

        private void DrawCausalCurves()
        {
            if (!_loaded || CurrentMode == "legacy") {
                if (CausalForceLine != null) CausalForceLine.Visibility = CausalTorqueLine.Visibility = Visibility.Collapsed;
                if (Model2ForceLine != null) Model2ForceLine.Visibility = Model2TorqueLine.Visibility = Visibility.Collapsed;
                return;
            }
            var watch = Stopwatch.StartNew();
            CausalForceLine.Visibility = CausalForceToggle.IsChecked == true ? Visibility.Visible : Visibility.Collapsed;
            CausalTorqueLine.Visibility = CausalTorqueToggle.IsChecked == true ? Visibility.Visible : Visibility.Collapsed;
            Model2ForceLine.Visibility = Model2ForceToggle.IsChecked == true ? Visibility.Visible : Visibility.Collapsed;
            Model2TorqueLine.Visibility = Model2TorqueToggle.IsChecked == true ? Visibility.Visible : Visibility.Collapsed;
            DrawTimed(ForceCanvas, Force1Line, CausalForceLine, true);
            DrawTimed(TorqueCanvas, Torque1Line, CausalTorqueLine, false);
            if (_curvePoints.Count > 0) {
                var s = _curvePoints.Last();
                ForceValueText.Text = string.Format(CultureInfo.InvariantCulture,
                    "原始 {0:F4} N   处理 {1:F4} N   t={2:F3} s", s.Fn, s.CorrectedFn, s.Time);
                TorqueValueText.Text = string.Format(CultureInfo.InvariantCulture,
                    "原始 {0:F4} N   处理 {1:F4} N", s.Ft, s.CorrectedFt);
                if (Model2ForceToggle.IsChecked == true)
                    ForceValueText.Text += s.Model2FnValid ? "\n示意有效" : "\n示意未启用/参考不足";
                if (Model2TorqueToggle.IsChecked == true)
                    TorqueValueText.Text += s.Model2FtValid ? "\n示意有效" : "\n示意未启用/参考不足";
            }
            _lastDrawMs = watch.Elapsed.TotalMilliseconds;
        }

        private void DrawTimed(Canvas canvas, Polyline original, Polyline corrected, bool axial)
        {
            if (canvas.ActualWidth < 10 || canvas.ActualHeight < 10) return;
            var illustration = axial ? Model2ForceLine : Model2TorqueLine;
            if (_curvePoints.Count == 0) { original.Points.Clear(); corrected.Points.Clear(); illustration.Points.Clear(); return; }
            double min = double.PositiveInfinity, max = double.NegativeInfinity;
            foreach (var s in _curvePoints) {
                double a = axial ? s.Fn : s.Ft, b = axial ? s.CorrectedFn : s.CorrectedFt;
                min = Math.Min(min, Math.Min(a, b)); max = Math.Max(max, Math.Max(a, b));
                if (illustration.Visibility == Visibility.Visible) {
                    double c = axial ? s.Model2Fn : s.Model2Ft;
                    min = Math.Min(min, c); max = Math.Max(max, c);
                }
            }
            double pad = Math.Max(.001, (max - min) * .05);
            min -= pad; max += pad;
            double end = _curvePoints.Last().Time, start = Math.Max(0, end - 10);
            double duration = Math.Max(.001, end - start);
            var aPoints = new PointCollection(_curvePoints.Count);
            var bPoints = new PointCollection(_curvePoints.Count);
            var cPoints = new PointCollection(_curvePoints.Count);
            foreach (var s in _curvePoints) {
                double x = (s.Time - start) / duration * canvas.ActualWidth;
                aPoints.Add(new Point(x, (max - (axial ? s.Fn : s.Ft)) / (max - min) * canvas.ActualHeight));
                bPoints.Add(new Point(x, (max - (axial ? s.CorrectedFn : s.CorrectedFt)) / (max - min) * canvas.ActualHeight));
                cPoints.Add(new Point(x, (max - (axial ? s.Model2Fn : s.Model2Ft)) / (max - min) * canvas.ActualHeight));
            }
            original.Points = aPoints; corrected.Points = bPoints;
            illustration.Points = cPoints;
        }

        // Offline visual QA never opens the pipe, connects ADS, or starts a backend.
        private void LoadCurveReplay()
        {
            string[] args = Environment.GetCommandLineArgs();
            string path = args[Array.IndexOf(args, "--curve-replay") + 1];
            string[] lines = File.ReadAllLines(path, System.Text.Encoding.UTF8);
            var rows = lines.Skip(1).Where(x => x.Length > 0).Select(x => x.Split(',')).ToArray();
            bool guidewire = rows.Length > 0 && rows[0][11] == "2";
            bool dynamicsReplay = args.Contains("--dynamics-replay");
            bool validationReplay = args.Contains("--validation-replay");
            string signReplay = args.Contains("--negative-sign-replay") ? "-1" : "1";
            // Exercise defaults and per-mode independent choices without a hardware connection.
            ExperimentModeBox.SelectedIndex = 1; UpdateModeView();
            if (Model2ForceToggle.IsChecked == true || Model2TorqueToggle.IsChecked == true)
                throw new InvalidOperationException("Model2 must default off");
            Model2ForceToggle.IsChecked = true;
            ExperimentModeBox.SelectedIndex = 2; UpdateModeView();
            if (Model2ForceToggle.IsChecked == true || Model2TorqueToggle.IsChecked == true)
                throw new InvalidOperationException("Model2 mode isolation failed");
            Model2TorqueToggle.IsChecked = true;
            ExperimentModeBox.SelectedIndex = 1; UpdateModeView();
            if (Model2ForceToggle.IsChecked != true || Model2TorqueToggle.IsChecked == true)
                throw new InvalidOperationException("Model2 channel persistence failed");
            // 在离线入口检查两侧符号隔离、条件确认清除及默认未验证状态。
            if (DynamicsSign.SelectedIndex != 0 || DynamicsValidation.IsChecked == true)
                throw new InvalidOperationException("Dynamics defaults failed");
            DynamicsSign.SelectedIndex = 1;
            DynamicsValidation.IsChecked = true;
            DynamicsConditions.IsChecked = true;
            ExperimentModeBox.SelectedIndex = 2; UpdateModeView();
            if (DynamicsSign.SelectedIndex != 0 || DynamicsValidation.IsChecked == true || DynamicsConditions.IsChecked == true)
                throw new InvalidOperationException("Dynamics side isolation failed");
            ExperimentModeBox.SelectedIndex = 1; UpdateModeView();
            if (DynamicsSign.SelectedIndex != 1 || DynamicsValidation.IsChecked != true || DynamicsConditions.IsChecked == true)
                throw new InvalidOperationException("Dynamics persistence/confirmation reset failed");
            Array.Clear(_dynamicsSigns, 0, _dynamicsSigns.Length);
            Array.Clear(_dynamicsValidation, 0, _dynamicsValidation.Length);
            RestoreDynamicsOptions();
            Array.Clear(_model2Choices, 0, _model2Choices.Length);
            RestoreModel2Visibility();
            int widthArg = Array.IndexOf(args, "--replay-width");
            int heightArg = Array.IndexOf(args, "--replay-height");
            if (widthArg >= 0) Width = D(args[widthArg + 1]);
            if (heightArg >= 0) Height = D(args[heightArg + 1]);
            ExperimentModeBox.SelectedIndex = guidewire ? 2 : 1;
            UpdateModeView();
            DynamicsSign.SelectedIndex = signReplay == "-1" ? 1 : 0;
            DynamicsValidation.IsChecked = validationReplay;
            DynamicsOptions.IsEnabled = false;
            if (args.Contains("--model2-replay")) {
                Model2ForceToggle.IsChecked = Model2TorqueToggle.IsChecked = true;
            }
            ExperimentModeBox.IsEnabled = false;
            SetPipeStatus(false, path.Contains("synthetic_") ? "合成数据界面测试 · 未连接硬件" : "离线曲线回放 · 未连接硬件");
            foreach (var b in FindVisualChildren<Button>(this)) b.IsEnabled = false;
            ulong seq = 0;
            for (int i = 0; i < rows.Length; i += 512) {
                var payload = new List<string>();
                foreach (var row in rows.Skip(i).Take(512)) {
                    payload.Add(string.Join(",", (++seq).ToString(CultureInfo.InvariantCulture), row[0],
                        row[7], row[8], (D(row[7]) - D(row[9])).ToString("R", CultureInfo.InvariantCulture),
                        (D(row[8]) - D(row[10])).ToString("R", CultureInfo.InvariantCulture), row[12], row[5])
                        + (row.Length >= 17 ? "," + string.Join(",", row.Skip(13).Take(4)) : ""));
                }
                ParseCurveResponse("PROGRAM_CURVES|1|" + (guidewire ? "2|1" : dynamicsReplay ? "1|1" : "1|0")
                    + "|1|1|offline-parity-fixture|0|511|0|" + string.Join(";", payload)
                    + (dynamicsReplay ? "|dynamics_25g|" + signReplay + "|" + (validationReplay ? "1" : "0")
                        + "|computed_physics_unverified|initial|2.85" : ""));
            }
            Dispatcher.BeginInvoke(new Action(() => {
                UpdateLayout(); DrawCausalCurves();
                UpdateLayout(); DrawCausalCurves();
                int snapshot = Array.IndexOf(args, "--curve-snapshot");
                if (snapshot >= 0 && snapshot + 1 < args.Length) {
                    var bitmap = new RenderTargetBitmap((int)ActualWidth, (int)ActualHeight, 96, 96, PixelFormats.Pbgra32);
                    bitmap.Render(this);
                    var png = new PngBitmapEncoder();
                    png.Frames.Add(BitmapFrame.Create(bitmap));
                    using (var stream = File.Create(args[snapshot + 1])) png.Save(stream);
                    File.WriteAllText(args[snapshot + 1] + ".json",
                        "{\"points\":" + _curvePoints.Count + ",\"draw_ms\":"
                        + _lastDrawMs.ToString("R", CultureInfo.InvariantCulture)
                        + ",\"purpose\":\"" + (dynamicsReplay ? "inertia_preview_ui_qa" : "target_illustration")
                        + "\",\"visibility_tests_passed\":true"
                        + ",\"model2_fn_visible\":" + (Model2ForceToggle.IsChecked == true ? "true" : "false")
                        + ",\"model2_ft_visible\":" + (Model2TorqueToggle.IsChecked == true ? "true" : "false")
                        + ",\"dynamics_ui_tests_passed\":true,\"hardware_connected\":false"
                        + ",\"dynamics_sign\":" + signReplay
                        + ",\"validation_mode\":" + (validationReplay ? "true" : "false") + "}", System.Text.Encoding.UTF8);
                    Close();
                }
            }), DispatcherPriority.ApplicationIdle);
        }
    }
}
