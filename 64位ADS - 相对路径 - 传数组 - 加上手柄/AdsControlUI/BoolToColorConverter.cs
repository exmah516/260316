using System;
using System.Globalization;
using System.Windows.Data;
using System.Windows.Media;

namespace AdsControlUI
{
    public class BoolToColorConverter : IValueConverter
    {
        public Color TrueColor { get; set; } = Colors.LimeGreen;
        public Color FalseColor { get; set; } = Colors.Gray;

        public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
        {
            bool b = value is bool bv && bv;
            return new SolidColorBrush(b ? TrueColor : FalseColor);
        }

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
        {
            throw new NotSupportedException();
        }
    }
}
