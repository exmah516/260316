using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;
using System.Windows.Threading;
using Microsoft.VisualBasic.FileIO;

namespace DualClampExperimentUI
{
    public partial class MainWindow
    {
        private bool IsExternalMode => CurrentMode == "external_validation";
        private int CurrentModeNumber => IsExternalMode ? 4 : CurrentMode == "guidewire" ? 2 : CurrentMode == "catheter" ? 1 : 0;
        private static bool IsExternalReplay => Environment.GetCommandLineArgs().Contains("--external-replay");
        private readonly bool[,] _curveChoices = { { true, true }, { true, true }, { true, true }, { false, false }, { false, false } };
        private int _curveChoiceMode;
        private bool _restoringCurveChoices;
        private bool _externalCalibrated, _externalRecordOk = true;
        private bool? _catheterCouplingBeforeExternal;
        private readonly List<ExternalSample> _externalPoints = new List<ExternalSample>();

        private sealed class ExternalSample
        {
            public ulong Sequence;
            public double Time, Fn1, Torque1, Fn6, Torque6, ModelFn1, ModelTorque1;
            public bool Valid, ModelValid;
            public int Phase, Cycle, Sync;
        }

        private void SaveCurveVisibility()
        {
            if (_restoringCurveChoices) return;
            _curveChoices[_curveChoiceMode, 0] = CausalForceToggle.IsChecked == true;
            _curveChoices[_curveChoiceMode, 1] = CausalTorqueToggle.IsChecked == true;
        }

        private void ConfigureExternalView()
        {
            if (IsExternalMode && !_catheterCouplingBeforeExternal.HasValue)
                _catheterCouplingBeforeExternal = ProgramCylinder1Coupling.IsChecked == true;
            if (!IsExternalMode && _catheterCouplingBeforeExternal.HasValue) {
                ProgramCylinder1Coupling.IsChecked = _catheterCouplingBeforeExternal.Value;
                _catheterCouplingBeforeExternal = null;
            }
            _restoringCurveChoices = true;
            _curveChoiceMode = CurrentModeNumber;
            CausalForceToggle.IsChecked = _curveChoices[_curveChoiceMode, 0];
            CausalTorqueToggle.IsChecked = _curveChoices[_curveChoiceMode, 1];
            _restoringCurveChoices = false;
            ExternalParameters.Visibility = ExternalForcePhases.Visibility = ExternalTorquePhases.Visibility =
                IsExternalMode ? Visibility.Visible : Visibility.Collapsed;
            DynamicsValidation.Visibility = DynamicsConditions.Visibility =
                IsExternalMode ? Visibility.Collapsed : Visibility.Visible;
            PulseGuardToggle.Visibility = IsExternalMode ? Visibility.Collapsed : Visibility.Visible;
            Force2LegendText.Text = IsExternalMode ? "轴6参考 (N)" : "fn2 (N)";
            Torque2LegendText.Text = IsExternalMode ? "轴6参考 (N·mm)" : "ft2 (N)";
            CausalTorqueToggle.Content = IsExternalMode ? "轴1惯性试算" : "ft 原值（未补偿）";
            CausalForceToggle.Content = IsExternalMode ? "轴1惯性试算" : "25 g 惯性试算";
            ProgramCylinder1Coupling.IsEnabled = !IsExternalMode;
            if (!IsExternalMode)
                ProgramAxis1PreparePos.IsEnabled = ProgramAxis1TriggerPos.IsEnabled =
                    ProgramCycleCount.IsEnabled = ProgramFinalDistance.IsEnabled = true;
            if (IsExternalMode)
            {
                _appliedValidation = false;
                ProgramPanelTitle.Text = "外源验证参数";
                ProgramCylinder1Coupling.IsChecked = true;
                ForceTitle.Text = "轴向力对比 (N)";
                TorqueTitle.Text = "扭矩对比 (N·mm)";
                Force1LegendText.Text = "轴1实测 (N)";
                Torque1LegendText.Text = "轴1实测 (N·mm)";
                Force2Legend.Visibility = Torque2Legend.Visibility = Force2Line.Visibility = Torque2Line.Visibility = Visibility.Visible;
                Model2ForceToggle.Visibility = Model2TorqueToggle.Visibility =
                    Model2ForceLine.Visibility = Model2TorqueLine.Visibility = Visibility.Collapsed;
            }
            UpdateExternalTravel();
        }

        private void UpdateExternalTravel()
        {
            if (!_loaded || !IsExternalMode) return;
            try
            {
                double total = D(Number(ProgramCycleCount)) * (D(Number(ProgramAxis1PreparePos)) - D(Number(ProgramAxis1TriggerPos)))
                    + D(Number(ProgramFinalDistance));
                double end = D(Number(ExternalAxis6PreparePos)) - total;
                ExternalTravelText.Text = string.Format(CultureInfo.InvariantCulture,
                    "轴6累计前进：{0:F3} mm\n预计终点距左限位：{1:F3} mm", total, end);
            }
            catch { ExternalTravelText.Text = "轴6预计行程：参数未完整"; }
        }

        private static string ExternalSyncName(int state)
        {
            switch (state) {
                case 1: return "等待主从耦合";
                case 2: return "1:1已耦合";
                case 3: return "等待主从解除";
                case 4: return "已解除耦合";
                default: return "轴6独立保持";
            }
        }

        private void UpdateExternalState(string[] p, bool editable)
        {
            _externalCalibrated = p.Length > 59 && p[59] == "1";
            ExternalParameters.IsEnabled = editable;
            ProgramCylinder1Coupling.IsEnabled = false;
            ProgramAxis1PreparePos.IsEnabled = ProgramAxis1TriggerPos.IsEnabled =
                ProgramCycleCount.IsEnabled = ProgramFinalDistance.IsEnabled = editable;
            LiveMotionText.Text = string.Format(CultureInfo.InvariantCulture,
                "轴1：{0:F3} mm / {1:F3} mm/s / {2:F3} mm/s²\n轴6：{3:F3} mm / {4:F3} mm/s / {5:F3} mm/s²\n轴2/轴7：{6:F3}° / {7:F3}°\n电缸1/2/3/4：{8} / {9} / {10} / {11}",
                D(p[8]), D(p[9]), D(p[10]), D(p[17]), D(p[18]), D(p[19]), D(p[11]), D(p[20]), p[27], p[28], p[29], p[30]);
            if (p.Length >= 70 && p[1] == "4") {
                int sync = int.Parse(p[65], CultureInfo.InvariantCulture);
                LiveMotionText.Text += "\n" + ExternalSyncName(sync);
                if (sync == 1 || sync == 3) PhaseText.Text = ExternalSyncName(sync);
                if (p[59] == "1" && _externalPoints.Count == 0) {
                    ForceValueText.Text = string.Format(CultureInfo.InvariantCulture, "轴1 {0:F4} N\n轴6参考 {1:F4} N", D(p[66]), D(p[68]));
                    TorqueValueText.Text = string.Format(CultureInfo.InvariantCulture, "轴1 {0:F4} N·mm\n轴6参考 {1:F4} N·mm", D(p[67]), D(p[69]));
                }
            }
            if (_externalPoints.Count > 0) DrawExternalCurves();
        }

        private void ResetExternalCurves()
        {
            _externalPoints.Clear();
            _externalCalibrated = false;
            if (ExternalForcePhases == null) return;
            ExternalForcePhases.Children.Clear();
            ExternalTorquePhases.Children.Clear();
            RemoveExternalSegments(ForceCanvas);
            RemoveExternalSegments(TorqueCanvas);
            Force2Line.Points.Clear(); Torque2Line.Points.Clear();
        }

        private void ParseExternalResponse(string response)
        {
            if (!IsExternalMode) return;
            string[] p = response.Split('|');
            if (p.Length != 7 || p[2] != "4") return;
            ulong generation = ulong.Parse(p[1], CultureInfo.InvariantCulture);
            if (generation != _curveGeneration) { ResetCurveView(); _curveGeneration = generation; }
            _externalCalibrated = p[3] == "1";
            _externalRecordOk = p[4] == "1";
            if (p[5] == "1") { _externalPoints.Clear(); _curveGap = true; }
            foreach (var item in p[6].Split(';')) {
                if (item.Length == 0) continue;
                string[] f = item.Split(',');
                if (f.Length != 13) continue;
                var s = new ExternalSample {
                    Sequence = ulong.Parse(f[0], CultureInfo.InvariantCulture), Time = D(f[1]),
                    Fn1 = D(f[2]), Torque1 = D(f[3]), Fn6 = D(f[4]), Torque6 = D(f[5]),
                    ModelFn1 = D(f[6]), ModelTorque1 = D(f[7]), Valid = f[8] == "1", ModelValid = f[9] == "1",
                    Phase = int.Parse(f[10], CultureInfo.InvariantCulture), Cycle = int.Parse(f[11], CultureInfo.InvariantCulture),
                    Sync = int.Parse(f[12], CultureInfo.InvariantCulture)
                };
                if (s.Sequence <= _curveCursor) continue;
                _curveCursor = s.Sequence;
                if (new[] {s.Time, s.Fn1, s.Torque1, s.Fn6, s.Torque6, s.ModelFn1, s.ModelTorque1}
                    .Any(x => double.IsNaN(x) || double.IsInfinity(x))) { _curveGap = true; continue; }
                _externalPoints.Add(s);
            }
            if (_externalPoints.Count > 0) {
                double oldest = _externalPoints.Last().Time - 10.0;
                _externalPoints.RemoveAll(s => s.Time < oldest);
                if (_externalPoints.Count > 10001) _externalPoints.RemoveRange(0, _externalPoints.Count - 10001);
            }
            DrawExternalCurves();
        }

        private static void RemoveExternalSegments(Canvas canvas)
        {
            foreach (var line in canvas.Children.OfType<Polyline>().Where(x => Equals(x.Tag, "externalSegment")).ToArray())
                canvas.Children.Remove(line);
        }

        private void DrawExternalCurves()
        {
            if (!_loaded || !IsExternalMode) return;
            CausalForceLine.Visibility = CausalForceToggle.IsChecked == true ? Visibility.Visible : Visibility.Collapsed;
            CausalTorqueLine.Visibility = CausalTorqueToggle.IsChecked == true ? Visibility.Visible : Visibility.Collapsed;
            Model2ForceLine.Visibility = Model2TorqueLine.Visibility = Visibility.Collapsed;
            DrawExternalChart(ForceCanvas, ExternalForcePhases, Force1Line, Force2Line, CausalForceLine, true);
            DrawExternalChart(TorqueCanvas, ExternalTorquePhases, Torque1Line, Torque2Line, CausalTorqueLine, false);
            CausalStatusText.Text = (_externalCalibrated ? "轴6参考 · 已取零" : "轴6参考 · 未取零")
                + (_curveGap ? " · 曲线缺段" : "") + (_externalRecordOk ? "" : " · 记录错误");
            CausalStatusText.ToolTip = "轴6按实验假设作为外源参考；两侧使用原符号、安装标定与解耦增量。";
            if (_externalPoints.Count == 0) return;
            var s = _externalPoints.Last();
            if (!s.Valid || !_externalCalibrated) { ForceValueText.Text = TorqueValueText.Text = "未取零"; return; }
            ForceValueText.Text = string.Format(CultureInfo.InvariantCulture, "轴1 {0:F4} N\n轴6参考 {1:F4} N", s.Fn1, s.Fn6);
            TorqueValueText.Text = string.Format(CultureInfo.InvariantCulture, "轴1 {0:F4} N·mm\n轴6参考 {1:F4} N·mm", s.Torque1, s.Torque6);
            if (CausalForceToggle.IsChecked == true)
                ForceValueText.Text += s.ModelValid ? "\n试算 " + s.ModelFn1.ToString("F4", CultureInfo.InvariantCulture) + " N" : "\n试算无效";
            if (CausalTorqueToggle.IsChecked == true)
                TorqueValueText.Text += s.ModelValid ? "\n试算 " + s.ModelTorque1.ToString("F4", CultureInfo.InvariantCulture) + " N·mm" : "\n试算无效";
            CausalStatusText.Text += "\n" + ExternalSyncName(s.Sync) + " · t=" + s.Time.ToString("F3", CultureInfo.InvariantCulture) + " s";
        }

        private void DrawExternalChart(Canvas canvas, Canvas bands, Polyline measured, Polyline reference, Polyline model, bool axial)
        {
            RemoveExternalSegments(canvas);
            bands.Children.Clear();
            measured.Points.Clear(); reference.Points.Clear(); model.Points.Clear();
            if (_externalPoints.Count == 0 || canvas.ActualWidth < 10 || canvas.ActualHeight < 10) return;
            double width = canvas.ActualWidth, height = canvas.ActualHeight;
            double start = _externalPoints.First().Time, end = _externalPoints.Last().Time;
            double duration = Math.Max(.001, end - start);
            var values = _externalPoints.Where(s => s.Valid).SelectMany(s => {
                var v = new List<double> { axial ? s.Fn1 : s.Torque1, axial ? s.Fn6 : s.Torque6 };
                if (model.Visibility == Visibility.Visible && s.ModelValid) v.Add(axial ? s.ModelFn1 : s.ModelTorque1);
                return v;
            }).ToArray();
            if (values.Length == 0 || !_externalCalibrated) return;
            double min = values.Min(), max = values.Max(), pad = Math.Max(.001, (max - min) * .08);
            min -= pad; max += pad;
            Func<double, double> xcoord = time => (time - start) / duration * width;
            Action<Polyline, Func<ExternalSample, double>, bool> draw = (line, select, isModel) => {
                if (line.Visibility != Visibility.Visible) return;
                var points = new PointCollection();
                bool first = true;
                Action flush = () => {
                    if (points.Count == 0) return;
                    if (first) { line.Points = points; first = false; }
                    else canvas.Children.Add(new Polyline { Points = points, Stroke = line.Stroke,
                        StrokeThickness = line.StrokeThickness, StrokeDashArray = line.StrokeDashArray,
                        Tag = "externalSegment", IsHitTestVisible = false });
                    points = new PointCollection();
                };
                ExternalSample? previous = null;
                foreach (var s in _externalPoints) {
                    if (previous != null && (s.Time <= previous.Time || s.Time - previous.Time > .003000001)) {
                        flush(); _curveGap = true;
                    }
                    if (!s.Valid || (isModel && !s.ModelValid)) flush();
                    else points.Add(new Point(xcoord(s.Time), 22 + (max - select(s)) / (max - min) * Math.Max(1, height - 42)));
                    previous = s;
                }
                flush();
            };
            draw(measured, s => axial ? s.Fn1 : s.Torque1, false);
            draw(reference, s => axial ? s.Fn6 : s.Torque6, false);
            draw(model, s => axial ? s.ModelFn1 : s.ModelTorque1, true);
            for (int i = 0; i < _externalPoints.Count;) {
                int j = i + 1, phase = _externalPoints[i].Phase;
                while (j < _externalPoints.Count && _externalPoints[j].Phase == phase &&
                    _externalPoints[j].Time - _externalPoints[j-1].Time <= .003000001) ++j;
                if (phase >= 5 && phase <= 7) {
                    double left = xcoord(_externalPoints[i].Time);
                    double right = xcoord(j < _externalPoints.Count ? _externalPoints[j].Time : end);
                    var band = new Rectangle { Width = Math.Max(0, right - left), Height = height,
                        Fill = new SolidColorBrush(phase == 5 ? Color.FromArgb(22, 234, 179, 8)
                            : phase == 6 ? Color.FromArgb(22, 225, 29, 72) : Color.FromArgb(22, 22, 163, 74)) };
                    Canvas.SetLeft(band, left); bands.Children.Add(band);
                    if (right - left > 55) {
                        var label = new TextBlock { Text = phase == 5 ? "释放" : phase == 6 ? "快退" : "重夹紧", FontSize = 10, Foreground = Brushes.DimGray };
                        Canvas.SetLeft(label, left + 4); Canvas.SetTop(label, 2); bands.Children.Add(label);
                    }
                }
                i = j;
            }
            var timeLabel = new TextBlock { Text = string.Format(CultureInfo.InvariantCulture, "{0:F3} - {1:F3} s", start, end), FontSize = 10, Foreground = Brushes.DimGray };
            Canvas.SetLeft(timeLabel, 5); Canvas.SetTop(timeLabel, Math.Max(0, height - 16)); bands.Children.Add(timeLabel);
            var legend = new StackPanel { Orientation = Orientation.Horizontal };
            foreach (var phase in new[] {5, 6, 7}) {
                legend.Children.Add(new TextBlock {
                    Text = phase == 5 ? "释放" : phase == 6 ? "快退" : "重夹紧",
                    Foreground = new SolidColorBrush(phase == 5 ? Color.FromRgb(161, 98, 7)
                        : phase == 6 ? Color.FromRgb(190, 18, 60) : Color.FromRgb(21, 128, 61)),
                    FontSize = 10, Margin = new Thickness(8, 0, 0, 0) });
            }
            Canvas.SetRight(legend, 6); Canvas.SetTop(legend, Math.Max(0, height - 16)); bands.Children.Add(legend);
        }

        private void LoadExternalReplay()
        {
            string[] args = Environment.GetCommandLineArgs();
            string path = args[Array.IndexOf(args, "--external-replay") + 1];
            ExperimentModeBox.SelectedIndex = 3;
            UpdateModeView();
            if (CausalForceToggle.IsChecked == true || CausalTorqueToggle.IsChecked == true)
                throw new InvalidOperationException("External model must default off");
            CausalForceToggle.IsChecked = true;
            ExperimentModeBox.SelectedIndex = 1; UpdateModeView();
            ExperimentModeBox.SelectedIndex = 3; UpdateModeView();
            if (CausalForceToggle.IsChecked != true || CausalTorqueToggle.IsChecked == true)
                throw new InvalidOperationException("External model choices are not isolated");
            CausalForceToggle.IsChecked = false;
            if (args.Contains("--external-model")) CausalForceToggle.IsChecked = CausalTorqueToggle.IsChecked = true;
            if (DynamicsValidation.IsChecked == true || Model2ForceToggle.Visibility != Visibility.Collapsed ||
                PulseGuardToggle.Visibility != Visibility.Collapsed)
                throw new InvalidOperationException("External mode exposes incompatible processing");
            int width = Array.IndexOf(args, "--replay-width"), height = Array.IndexOf(args, "--replay-height");
            if (width >= 0) Width = D(args[width + 1]);
            if (height >= 0) Height = D(args[height + 1]);
            var payload = new List<string>();
            using (var csv = new TextFieldParser(path)) {
                csv.SetDelimiters(",");
                csv.HasFieldsEnclosedInQuotes = true;
                string[] header = csv.ReadFields() ?? Array.Empty<string>();
                var columns = header.Select((name, i) => new { name, i }).ToDictionary(x => x.name, x => x.i);
                while (!csv.EndOfData) {
                    var row = csv.ReadFields();
                    if (row == null) continue;
                    Func<string, string> field = name => row[columns[name]];
                    Func<string, string> numeric = name => string.IsNullOrEmpty(field(name)) ? "0" : field(name);
                    payload.Add(string.Join(",", (ulong.Parse(field("sample_index"), CultureInfo.InvariantCulture) + 1).ToString(CultureInfo.InvariantCulture),
                        (D(field("plc_time_us")) * 1e-6).ToString("R", CultureInfo.InvariantCulture),
                        numeric("fn1_decoupled_delta_N"), numeric("torque1_decoupled_delta_Nmm"),
                        numeric("fn2_decoupled_delta_N"), numeric("torque2_decoupled_delta_Nmm"),
                        numeric("fn1_model_decoupled_N"), numeric("torque1_model_decoupled_Nmm"),
                        field("force_valid"), field("model_valid"), field("phase"), field("cycle_index"), field("sync_state")));
                    if (payload.Count == 512) {
                        ParseExternalResponse("PROGRAM_EXTERNAL_CURVES|1|4|1|1|0|" + string.Join(";", payload));
                        payload.Clear();
                    }
                }
            }
            ParseExternalResponse("PROGRAM_EXTERNAL_CURVES|1|4|1|1|0|" + string.Join(";", payload));
            if (_externalPoints.Count > 0) ProgramCycleCount.Text = _externalPoints.Max(s => s.Cycle).ToString(CultureInfo.InvariantCulture);
            SetPipeStatus(false, "外源模式离线回放 · 未连接硬件");
            ExperimentModeBox.IsEnabled = false;
            foreach (var button in FindVisualChildren<Button>(this)) button.IsEnabled = false;
            Dispatcher.BeginInvoke(new Action(() => {
                UpdateLayout(); DrawExternalCurves(); UpdateLayout();
                int snapshot = Array.IndexOf(args, "--curve-snapshot");
                if (snapshot >= 0) {
                    if (_externalPoints.Count == 0 || Force1Line.Points.Count == 0 || Force2Line.Points.Count == 0 ||
                        Torque1Line.Points.Count == 0 || Torque2Line.Points.Count == 0)
                        throw new InvalidOperationException("External replay charts are blank");
                    var bitmap = new RenderTargetBitmap((int)ActualWidth, (int)ActualHeight, 96, 96, PixelFormats.Pbgra32);
                    var visual = new DrawingVisual();
                    using (var context = visual.RenderOpen()) {
                        var bounds = new Rect(0, 0, ActualWidth, ActualHeight);
                        context.DrawRectangle(Background, null, bounds);
                        context.DrawRectangle(new VisualBrush(this) { Stretch = Stretch.None,
                            AlignmentX = AlignmentX.Left, AlignmentY = AlignmentY.Top }, null, bounds);
                    }
                    bitmap.Render(visual);
                    var png = new PngBitmapEncoder(); png.Frames.Add(BitmapFrame.Create(bitmap));
                    using (var stream = File.Create(args[snapshot + 1])) png.Save(stream);
                    File.WriteAllText(args[snapshot + 1] + ".json", "{\"mode\":\"external_validation\",\"hardware_connected\":false,"
                        + "\"mode_isolation_passed\":true,\"points\":" + _externalPoints.Count
                        + ",\"phase_bands\":" + ExternalForcePhases.Children.OfType<Rectangle>().Count()
                        + ",\"model_visible\":" + (CausalForceToggle.IsChecked == true ? "true" : "false") + "}");
                    Close();
                }
            }), DispatcherPriority.ApplicationIdle);
        }
    }
}
