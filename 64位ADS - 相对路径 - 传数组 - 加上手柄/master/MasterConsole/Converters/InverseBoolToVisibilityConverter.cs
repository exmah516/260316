using System;
using System.Globalization;
using System.Windows;
using System.Windows.Data;

namespace MasterConsole.Converters
{
    /// <summary>布尔取反后转可见性：条件为 true 时隐藏（用于“不可操作时才显示的提示”）。</summary>
    public sealed class InverseBoolToVisibilityConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
            => value is bool b && b ? Visibility.Collapsed : Visibility.Visible;

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
            => throw new NotSupportedException();
    }
}
