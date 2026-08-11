using System.Windows;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using System.Windows.Input;
using System.Windows.Threading;

namespace AdsControlUI
{
    public partial class MainWindow : Window
    {
        private readonly AdsControlViewModel _vm;
        private ForceRealtimeWindow _forceWindow;
        private CleanForceWindow _cleanForceWindow;
        private CameraPreviewWindow _cameraPreviewWindow;
        private ForceTransitionWindow _ftExpWindow;
		private readonly DispatcherTimer _manualJogKeepaliveTimer;
		private int _activeArmJogAxis;
		private int _activeArmJogDirection;
		private int _activeAxis4JogDirection;

        public MainWindow()
        {
            InitializeComponent();
            _vm = new AdsControlViewModel();
            DataContext = _vm;
            _vm.StateUpdated += Vm_StateUpdated;
			_manualJogKeepaliveTimer = new DispatcherTimer { Interval = System.TimeSpan.FromMilliseconds(100) };
			_manualJogKeepaliveTimer.Tick += ManualJogKeepaliveTimer_Tick;
			_manualJogKeepaliveTimer.Start();
        }

        protected override void OnClosed(System.EventArgs e)
        {
			_manualJogKeepaliveTimer.Stop();
			StopAllManualJogs();
            _vm.StateUpdated -= Vm_StateUpdated;
            _forceWindow?.Close();
            _cleanForceWindow?.Close();
            _cameraPreviewWindow?.Close();
            _ftExpWindow?.Close();
            _vm.Dispose();
            base.OnClosed(e);
        }

		protected override void OnDeactivated(System.EventArgs e)
		{
			StopAllManualJogs();
			base.OnDeactivated(e);
		}

        private void Cyl1_Click(object sender, RoutedEventArgs e) => SetCylinderState(sender, 0);
        private void Cyl2_Click(object sender, RoutedEventArgs e) => SetCylinderState(sender, 1);
        private void Cyl3_Click(object sender, RoutedEventArgs e) => SetCylinderState(sender, 2);
        private void Cyl4_Click(object sender, RoutedEventArgs e) => SetCylinderState(sender, 3);

        private void SetCylinderState(object sender, int index)
        {
            if (sender is ToggleButton button)
                _vm.SetCylinderManualState(index, button.IsChecked == true);
        }

        private void ModeCathFwd_Click(object sender, RoutedEventArgs e) => _vm.SetMode(0, 0);
        private void ModeCathRev_Click(object sender, RoutedEventArgs e) => _vm.SetMode(0, 1);
        private void ModeGuideFwd_Click(object sender, RoutedEventArgs e) => _vm.SetMode(1, 0);
        private void ModeGuideRev_Click(object sender, RoutedEventArgs e) => _vm.SetMode(1, 1);
        private void ModeCooperativeDelivery_Click(object sender, RoutedEventArgs e)
        {
            if (sender is ToggleButton button)
                _vm.SetCooperativeDelivery(button.IsChecked == true);
        }
        private void ModeCooperativeRetraction_Click(object sender, RoutedEventArgs e)
        {
            if (sender is ToggleButton button)
                _vm.SetCooperativeRetraction(button.IsChecked == true);
        }
        private void SpacingRecovery_Click(object sender, RoutedEventArgs e)
        {
            if (sender is ToggleButton button)
                _vm.SetSpacingRecovery(button.IsChecked == true);
        }

		private void ArmManualEnable_Click(object sender, RoutedEventArgs e)
		{
			if (sender is ToggleButton button)
			{
				if (button.IsChecked != true)
					StopAllManualJogs();
				_vm.SetArmManualEnable(button.IsChecked == true);
			}
		}

		private void ArmAxisEnable_Click(object sender, RoutedEventArgs e)
		{
			if (sender is ToggleButton button && TryGetAxisNumber(button.Tag, out int axisNumber))
			{
				if (button.IsChecked != true)
					StopArmAxisJog(axisNumber);
				_vm.SetArmAxisEnable(axisNumber, button.IsChecked == true);
			}
		}

		private void ArmAxisReset_Click(object sender, RoutedEventArgs e)
		{
			if (sender is Button button && TryGetAxisNumber(button.Tag, out int axisNumber))
				_vm.RequestArmAxisReset(axisNumber);
		}

		private void ArmJogButton_PreviewMouseLeftButtonDown(object sender, MouseButtonEventArgs e)
		{
			if (!(sender is Button button) ||
				!TryGetAxisNumber(button.Tag, out int axisNumber) ||
				!TryGetJogDirection(button.CommandParameter, out int direction))
			{
				return;
			}

			if (_activeArmJogAxis != 0 && _activeArmJogAxis != axisNumber)
				_vm.SetArmAxisJog(_activeArmJogAxis, 0);
			_activeArmJogAxis = axisNumber;
			_activeArmJogDirection = direction;
			_vm.SetArmAxisJog(axisNumber, direction);
			button.CaptureMouse();
		}

		private void ArmJogButton_PreviewMouseLeftButtonUp(object sender, MouseButtonEventArgs e)
		{
			StopArmJog(sender);
		}

		private void ArmJogButton_LostMouseCapture(object sender, MouseEventArgs e)
		{
			StopArmJog(sender);
		}

		private void StopArmJog(object sender)
		{
			if (!(sender is Button button) || !TryGetAxisNumber(button.Tag, out int axisNumber))
				return;
			StopArmAxisJog(axisNumber);
			if (button.IsMouseCaptured)
				button.ReleaseMouseCapture();
		}

		private void Axis4JogButton_PreviewMouseLeftButtonDown(object sender, MouseButtonEventArgs e)
		{
			if (!(sender is Button button) ||
				!TryGetJogDirection(button.CommandParameter, out int direction))
			{
				return;
			}

			_activeAxis4JogDirection = direction;
			_vm.SetAxis4ManualJog(direction);
			button.CaptureMouse();
		}

		private void Axis4JogButton_PreviewMouseLeftButtonUp(object sender, MouseButtonEventArgs e)
		{
			StopAxis4Jog(sender);
		}

		private void Axis4JogButton_LostMouseCapture(object sender, MouseEventArgs e)
		{
			StopAxis4Jog(sender);
		}

		private void StopAxis4Jog(object sender)
		{
			if (!(sender is Button button)) return;
			_activeAxis4JogDirection = 0;
			_vm.SetAxis4ManualJog(0);
			if (button.IsMouseCaptured)
				button.ReleaseMouseCapture();
		}

		private void StopArmAxisJog(int axisNumber)
		{
			if (_activeArmJogAxis == axisNumber)
			{
				_activeArmJogAxis = 0;
				_activeArmJogDirection = 0;
			}
			_vm.SetArmAxisJog(axisNumber, 0);
		}

		private void StopAllManualJogs()
		{
			_activeArmJogAxis = 0;
			_activeArmJogDirection = 0;
			_activeAxis4JogDirection = 0;
			_vm.StopManualJogs();
		}

		private void ManualJogKeepaliveTimer_Tick(object sender, System.EventArgs e)
		{
			if (_activeArmJogAxis != 0 && _activeArmJogDirection != 0)
				_vm.SetArmAxisJog(_activeArmJogAxis, _activeArmJogDirection);
			if (_activeAxis4JogDirection != 0)
				_vm.SetAxis4ManualJog(_activeAxis4JogDirection);
		}

		private void ApplyArmJogParameters_Click(object sender, RoutedEventArgs e)
		{
			if (!(sender is Button button) || !(button.DataContext is ArmAxisControlModel axis))
				return;
			if (!axis.TryGetParameters(out double velocity, out double acceleration, out double deceleration, out double jerk))
				return;

			if (_vm.SetArmJogParameters(axis.AxisNumber, velocity, acceleration, deceleration, jerk))
				axis.MarkParametersApplied();
			else
				axis.ParameterErrorText = "参数发送失败，请检查上位机管道连接。";
		}

		private static bool TryGetAxisNumber(object value, out int axisNumber)
		{
			try
			{
				axisNumber = System.Convert.ToInt32(value);
				return axisNumber >= 1 && axisNumber <= 5;
			}
			catch
			{
				axisNumber = 0;
				return false;
			}
		}

		private static bool TryGetJogDirection(object value, out int direction)
		{
			try
			{
				direction = System.Convert.ToInt32(value);
				return direction == -1 || direction == 1;
			}
			catch
			{
				direction = 0;
				return false;
			}
		}

        private void Zero_Click(object sender, RoutedEventArgs e) => _vm.ZeroForceSensor();
        private void FfToggle_Click(object sender, RoutedEventArgs e) => _vm.ToggleForceFeedback();
        private void GravityComp_Click(object sender, RoutedEventArgs e) => _vm.SetGravityCompensation(GravityCompCheckBox.IsChecked == true);
        private void TrackingCompensation_Click(object sender, RoutedEventArgs e)
        {
            if (sender is ToggleButton button)
                _vm.SetTrackingCompensation(button.IsChecked == true);
        }

        private void ApplyTrackingParams_Click(object sender, RoutedEventArgs e)
        {
            TrackingError.Text = "";
            if (_vm.TrackingCompensationEnabled)
            {
                TrackingError.Text = "请先关闭位移补偿，再修改参数。";
                return;
            }

            if (!double.TryParse(TbTrackAxis1Kp.Text, out double a1Kp) ||
                !double.TryParse(TbTrackAxis1Ki.Text, out double a1Ki) ||
                !double.TryParse(TbTrackAxis1Gain.Text, out double a1Gain) ||
                !double.TryParse(TbTrackAxis1Error.Text, out double a1Error) ||
                !double.TryParse(TbTrackAxis6Kp.Text, out double a6Kp) ||
                !double.TryParse(TbTrackAxis6Ki.Text, out double a6Ki) ||
                !double.TryParse(TbTrackAxis6Gain.Text, out double a6Gain) ||
                !double.TryParse(TbTrackAxis6Error.Text, out double a6Error))
            {
                TrackingError.Text = "参数格式错误，请输入有效数字。";
                return;
            }

			bool gainsValid = a1Gain > 1.0 && a1Gain <= 5.0 && a6Gain > 1.0 && a6Gain <= 5.0;
			bool errorsValid = a1Error > 0.0 && a1Error <= 20.0 && a6Error > 0.0 && a6Error <= 20.0;
			bool piValid = a1Kp >= 0.0 && a1Kp <= 1.0 && a1Ki >= 0.0 && a1Ki <= 100.0 &&
						   a6Kp >= 0.0 && a6Kp <= 1.0 && a6Ki >= 0.0 && a6Ki <= 100.0;
			if (!gainsValid || !errorsValid || !piValid)
			{
				TrackingError.Text = "Kp 范围为 0-1，Ki 范围为 0-100，最大增益为 (1, 5]，换手欠账上界为 (0, 20] mm。";
                return;
            }

            _vm.SendTrackingCompensationParams(a1Kp, a1Ki, a1Gain, a1Error, a6Kp, a6Ki, a6Gain, a6Error);
            TrackingError.Text = "参数已发送；Kp=0 时补偿不会产生额外位移。";
        }

        private void ApplyAxis1PostReturnLead_Click(object sender, RoutedEventArgs e)
        {
            Axis1LeadError.Text = "";
            if (!double.TryParse(TbAxis1PostReturnLead.Text, out double leadMm))
            {
                Axis1LeadError.Text = "输入格式错误，请输入有效数字。";
                return;
            }

            if (leadMm < 0.0 || leadMm > 10.0)
            {
                Axis1LeadError.Text = "轴1回退完成后的自动先行量必须在 0 到 10 mm 之间。";
                return;
            }

            _vm.SetAxis1PostReturnLead(leadMm);
            Axis1LeadError.Text = leadMm <= 0.0
                ? "自动先行已关闭。"
                : "自动先行量已发送，将沿递送方向执行。";
        }

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
			_cleanForceWindow?.AddState(state);
			_cameraPreviewWindow?.OnState(state);
            _ftExpWindow?.OnState(state);
        }

		private void StartExperimentRecording_Click(object sender, RoutedEventArgs e)
		{
			if (_vm.CanStartExperimentRecording)
				_vm.StartExperimentRecording(TbExperimentName.Text);
		}

		private void StopExperimentRecording_Click(object sender, RoutedEventArgs e)
		{
			if (_vm.CanStopExperimentRecording)
				_vm.StopExperimentRecording();
		}

		private void ShowCameraPreview_Click(object sender, RoutedEventArgs e)
		{
			if (_cameraPreviewWindow == null)
			{
				_cameraPreviewWindow = new CameraPreviewWindow { Owner = this };
				_cameraPreviewWindow.Closed += (s, args) =>
				{
					_vm.SetCameraPreview(false);
					_cameraPreviewWindow = null;
				};
				_vm.SetCameraPreview(true);
				_cameraPreviewWindow.Show();
			}
			else
			{
				_cameraPreviewWindow.Activate();
			}
			_cameraPreviewWindow?.OnState(_vm.LatestState);
		}

		private void ShowCleanForce_Click(object sender, RoutedEventArgs e)
		{
			if (_cleanForceWindow == null)
			{
				_cleanForceWindow = new CleanForceWindow { Owner = this };
				_cleanForceWindow.Closed += (s, args) =>
				{
					_vm.SetCleanForceMonitor(false);
					_cleanForceWindow = null;
				};
				_vm.SetCleanForceMonitor(true);
				_cleanForceWindow.Show();
			}
			else
			{
				_cleanForceWindow.Activate();
			}
			_cleanForceWindow?.AddState(_vm.LatestState);
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
