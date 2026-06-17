using System.Windows;

namespace AdsControlUI
{
    public partial class MainWindow : Window
    {
        private readonly AdsControlViewModel _vm;
        private readonly bool[] _cylOverride = new bool[4];
        private ForceRealtimeWindow _forceWindow;
        private ForceTransitionWindow _ftExpWindow;

        public MainWindow()
        {
            InitializeComponent();
            _vm = new AdsControlViewModel();
            DataContext = _vm;
            _vm.StateUpdated += Vm_StateUpdated;
        }

        protected override void OnClosed(System.EventArgs e)
        {
            _vm.StateUpdated -= Vm_StateUpdated;
            _forceWindow?.Close();
            _ftExpWindow?.Close();
            _vm.Dispose();
            base.OnClosed(e);
        }

        private void Cyl1_Click(object sender, RoutedEventArgs e) => ToggleCylinder(0);
        private void Cyl2_Click(object sender, RoutedEventArgs e) => ToggleCylinder(1);
        private void Cyl3_Click(object sender, RoutedEventArgs e) => ToggleCylinder(2);
        private void Cyl4_Click(object sender, RoutedEventArgs e) => ToggleCylinder(3);

        private void ToggleCylinder(int index)
        {
            _cylOverride[index] = !_cylOverride[index];
            if (_cylOverride[index])
                _vm.SetCylinderOverride(index);
            else
                _vm.ClearCylinderOverride(index);
        }

        private void ModeCathFwd_Click(object sender, RoutedEventArgs e) => _vm.SetMode(0, 0);
        private void ModeCathRev_Click(object sender, RoutedEventArgs e) => _vm.SetMode(0, 1);
        private void ModeGuideFwd_Click(object sender, RoutedEventArgs e) => _vm.SetMode(1, 0);
        private void ModeGuideRev_Click(object sender, RoutedEventArgs e) => _vm.SetMode(1, 1);

        private void Zero_Click(object sender, RoutedEventArgs e) => _vm.ZeroForceSensor();
        private void FfToggle_Click(object sender, RoutedEventArgs e) => _vm.ToggleForceFeedback();
        private void GravityComp_Click(object sender, RoutedEventArgs e) => _vm.SetGravityCompensation(GravityCompCheckBox.IsChecked == true);
        private void ForceLog_Click(object sender, RoutedEventArgs e) => _vm.ToggleForceLog();
        private void DirectControl_Click(object sender, RoutedEventArgs e) => _vm.SelectDirectControl();

        private void ShowForce_Click(object sender, RoutedEventArgs e)
        {
            ForcePlotError.Text = "";
            if (!_vm.FfEnabled)
            {
                ForcePlotError.Text = "请先开启力反馈。";
                return;
            }
            if (!_vm.CalZeroed)
            {
                ForcePlotError.Text = "请先完成力传感器零点采集。";
                return;
            }

            if (_forceWindow == null)
            {
                _forceWindow = new ForceRealtimeWindow { Owner = this };
                _forceWindow.Closed += (s, args) => _forceWindow = null;
                _forceWindow.Show();
            }
            else
            {
                _forceWindow.Activate();
            }

            _forceWindow.AddState(_vm.LatestState);
        }

        private void Vm_StateUpdated(VisState state)
        {
            _forceWindow?.AddState(state);
            _ftExpWindow?.OnState(state);
        }

        private void ShowFtExp_Click(object sender, RoutedEventArgs e)
        {
            if (_ftExpWindow == null)
            {
                _ftExpWindow = new ForceTransitionWindow(_vm) { Owner = this };
                _ftExpWindow.Closed += (s, args) => _ftExpWindow = null;
                _ftExpWindow.Show();
            }
            else
            {
                _ftExpWindow.Activate();
            }
        }

        private void ExecuteStartup_Click(object sender, RoutedEventArgs e)
        {
            StartupError.Text = "";
            if (!double.TryParse(TbAxis1.Text, out double a1) ||
                !double.TryParse(TbAxis3.Text, out double a3) ||
                !double.TryParse(TbAxis5.Text, out double a5) ||
                !double.TryParse(TbAxis6.Text, out double a6) ||
                !double.TryParse(TbAxis2.Text, out double a2) ||
                !double.TryParse(TbAxis7.Text, out double a7) ||
                !double.TryParse(TbSpeed.Text, out double speed))
            {
                StartupError.Text = "输入格式错误，请输入有效数字。";
                return;
            }

            if (a1 < 5 || a1 > 95) { StartupError.Text = "轴1必须在5-95mm之间。"; return; }
            if (a3 < 10 || a3 > 650) { StartupError.Text = "轴3必须在10-650mm之间。"; return; }
            if (a5 < 10 || a5 > 670) { StartupError.Text = "轴5必须在10-670mm之间。"; return; }
            if (a6 < 10 || a6 > 670) { StartupError.Text = "轴6必须在10-670mm之间。"; return; }
            if (a2 < -360 || a2 > 360) { StartupError.Text = "轴2必须在-360~360度之间。"; return; }
            if (a7 < -360 || a7 > 360) { StartupError.Text = "轴7必须在-360~360度之间。"; return; }
            if (speed < 0.00001 || speed > 0.5) { StartupError.Text = "速度比例必须在0.00001-0.5之间。"; return; }
            if (a6 < a5) { StartupError.Text = "轴6位置必须>=轴5。"; return; }
            if (a5 < a3) { StartupError.Text = "轴5位置必须>=轴3。"; return; }
            if (a3 < a1) { StartupError.Text = "轴3位置必须>=轴1。"; return; }

            _vm.SendStartupParams(a1, a3, a5, a6, a2, a7, speed);
        }
    }
}
