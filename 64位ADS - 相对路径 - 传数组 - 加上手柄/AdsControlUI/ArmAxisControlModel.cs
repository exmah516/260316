using System;
using System.ComponentModel;
using System.Globalization;
using System.Runtime.CompilerServices;

namespace AdsControlUI
{
    // 定位臂单轴显示与参数编辑模型。运动请求仍统一由主 ViewModel 经命名管道下发。
    public sealed class ArmAxisControlModel : INotifyPropertyChanged
    {
        private bool _enableRequested;
        private bool _powerDone;
        private bool _powerBusy;
        private bool _powerActive;
        private bool _powerError;
        private uint _powerErrorId;
        private bool _resetDone;
        private bool _resetBusy;
        private bool _resetError;
        private uint _resetErrorId;
        private double _actualPosition;
        private double _actualVelocity;
        private bool _motionBusy;
        private bool _motionDone;
        private bool _motionError;
        private uint _motionErrorId;
        private sbyte _commandDirection;
        private bool _commandConflict;
        private bool _parametersEdited;
        private string _velocityInput = "5";
        private string _accelerationInput = "50";
        private string _decelerationInput = "50";
        private string _jerkInput = "500";
        private string _parameterErrorText = "";

        public ArmAxisControlModel(int axisNumber)
        {
            AxisNumber = axisNumber;
        }

        public int AxisNumber { get; }
        public string AxisLabel => $"定位臂轴 {AxisNumber}";
        public bool EnableRequested => _enableRequested;
        public double ActualPosition => _actualPosition;
        public double ActualVelocity => _actualVelocity;
        public string PositionText => $"{ActualPosition:F3}";
        public string VelocityText => $"{ActualVelocity:F3}";
        public bool HasFault => _powerError || _resetError || _motionError || _commandConflict;

        public string StatusText
        {
            get
            {
                if (_powerError)
                    return $"上电错误 0x{_powerErrorId:X8}";
                if (_resetError)
                    return $"复位错误 0x{_resetErrorId:X8}";
                if (_motionError)
                    return $"运动错误 0x{_motionErrorId:X8}";
                if (_commandConflict)
                    return "正反点动请求冲突";
                if (_resetBusy)
                    return "正在复位";
                if (_motionBusy)
                    return _commandDirection > 0 ? "正向点动中" : _commandDirection < 0 ? "反向点动中" : "正在停止";
                if (_powerDone || _powerActive)
                    return "已上电，等待点动";
                if (_powerBusy)
                    return "正在上电";
                if (_enableRequested)
                    return "等待上电确认";
                if (_resetDone)
                    return "复位完成，未上电";
                return "未上电";
            }
        }

        public string VelocityInput
        {
            get => _velocityInput;
            set => SetParameterText(ref _velocityInput, value);
        }

        public string AccelerationInput
        {
            get => _accelerationInput;
            set => SetParameterText(ref _accelerationInput, value);
        }

        public string DecelerationInput
        {
            get => _decelerationInput;
            set => SetParameterText(ref _decelerationInput, value);
        }

        public string JerkInput
        {
            get => _jerkInput;
            set => SetParameterText(ref _jerkInput, value);
        }

        public string ParameterErrorText
        {
            get => _parameterErrorText;
            set => SetField(ref _parameterErrorText, value ?? "");
        }

        public bool TryGetParameters(out double velocity, out double acceleration, out double deceleration, out double jerk)
        {
            bool velocityValid = TryParseParameter(VelocityInput, out velocity);
            bool accelerationValid = TryParseParameter(AccelerationInput, out acceleration);
            bool decelerationValid = TryParseParameter(DecelerationInput, out deceleration);
            bool jerkValid = TryParseParameter(JerkInput, out jerk);
            bool valid = velocityValid && accelerationValid && decelerationValid && jerkValid;
            if (!valid)
            {
                ParameterErrorText = "参数必须为 0.01 至 10000 的有效数值。";
                return false;
            }

            ParameterErrorText = "";
            return true;
        }

        public void MarkParametersApplied()
        {
            _parametersEdited = true;
            ParameterErrorText = "参数已发送";
        }

        public void UpdateFrom(VisState state, int index)
        {
            SetField(ref _enableRequested, BoolAt(state.arm_enable_req, index), nameof(EnableRequested));
            SetField(ref _powerDone, BoolAt(state.arm_power_done, index));
            SetField(ref _powerBusy, BoolAt(state.arm_power_busy, index));
            SetField(ref _powerActive, BoolAt(state.arm_power_active, index));
            SetField(ref _powerError, BoolAt(state.arm_power_error, index));
            SetField(ref _powerErrorId, UIntAt(state.arm_power_error_id, index));
            SetField(ref _resetDone, BoolAt(state.arm_reset_done, index));
            SetField(ref _resetBusy, BoolAt(state.arm_reset_busy, index));
            SetField(ref _resetError, BoolAt(state.arm_reset_error, index));
            SetField(ref _resetErrorId, UIntAt(state.arm_reset_error_id, index));

            bool positionChanged = SetField(ref _actualPosition, DoubleAt(state.arm_act_pos, index), nameof(ActualPosition));
            bool velocityChanged = SetField(ref _actualVelocity, DoubleAt(state.arm_act_vel, index), nameof(ActualVelocity));
            SetField(ref _motionBusy, BoolAt(state.arm_motion_busy, index));
            SetField(ref _motionDone, BoolAt(state.arm_motion_done, index));
            SetField(ref _motionError, BoolAt(state.arm_motion_error, index));
            SetField(ref _motionErrorId, UIntAt(state.arm_motion_error_id, index));
            SetField(ref _commandDirection, SByteAt(state.arm_cmd_dir, index));
            SetField(ref _commandConflict, BoolAt(state.arm_cmd_conflict, index));

            if (positionChanged) OnPropertyChanged(nameof(PositionText));
            if (velocityChanged) OnPropertyChanged(nameof(VelocityText));
            OnPropertyChanged(nameof(StatusText));
            OnPropertyChanged(nameof(HasFault));

            if (!_parametersEdited)
            {
                UpdateParameterFromPlc(ref _velocityInput, DoubleAt(state.arm_jog_velocity, index), nameof(VelocityInput));
                UpdateParameterFromPlc(ref _accelerationInput, DoubleAt(state.arm_jog_acc, index), nameof(AccelerationInput));
                UpdateParameterFromPlc(ref _decelerationInput, DoubleAt(state.arm_jog_dec, index), nameof(DecelerationInput));
                UpdateParameterFromPlc(ref _jerkInput, DoubleAt(state.arm_jog_jerk, index), nameof(JerkInput));
            }
        }

        private static bool TryParseParameter(string text, out double value)
        {
            return double.TryParse(text, NumberStyles.Float, CultureInfo.CurrentCulture, out value) &&
                !double.IsNaN(value) && !double.IsInfinity(value) &&
                value >= 0.01 && value <= 10000.0;
        }

        private static string FormatParameter(double value) =>
            value.ToString("0.###", CultureInfo.CurrentCulture);

        private void UpdateParameterFromPlc(ref string field, double value, string propertyName)
        {
            if (value < 0.01 || value > 10000.0 || double.IsNaN(value) || double.IsInfinity(value))
                return;
            SetField(ref field, FormatParameter(value), propertyName);
        }

        private void SetParameterText(ref string field, string value, [CallerMemberName] string propertyName = null)
        {
            _parametersEdited = true;
            SetField(ref field, value ?? "", propertyName);
        }

        private static bool BoolAt(bool[] values, int index) =>
            values != null && index >= 0 && index < values.Length && values[index];

        private static uint UIntAt(uint[] values, int index) =>
            values != null && index >= 0 && index < values.Length ? values[index] : 0U;

        private static double DoubleAt(double[] values, int index) =>
            values != null && index >= 0 && index < values.Length ? values[index] : 0.0;

        private static sbyte SByteAt(sbyte[] values, int index) =>
            values != null && index >= 0 && index < values.Length ? values[index] : (sbyte)0;

        private bool SetField<T>(ref T field, T value, string propertyName = null)
        {
            if (Equals(field, value)) return false;
            field = value;
            if (!string.IsNullOrEmpty(propertyName))
                OnPropertyChanged(propertyName);
            return true;
        }

        public event PropertyChangedEventHandler PropertyChanged;

        private void OnPropertyChanged([CallerMemberName] string propertyName = null) =>
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
    }
}
