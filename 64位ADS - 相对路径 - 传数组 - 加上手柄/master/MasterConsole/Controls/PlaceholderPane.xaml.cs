using System.Windows;
using System.Windows.Controls;

namespace MasterConsole.Controls
{
    /// <summary>显示区占位框（影像 / URDF 模型）。</summary>
    public partial class PlaceholderPane : UserControl
    {
        public static readonly DependencyProperty TitleProperty =
            DependencyProperty.Register(nameof(Title), typeof(string), typeof(PlaceholderPane), new PropertyMetadata(""));

        public static readonly DependencyProperty HintProperty =
            DependencyProperty.Register(nameof(Hint), typeof(string), typeof(PlaceholderPane), new PropertyMetadata(""));

        public string Title
        {
            get => (string)GetValue(TitleProperty);
            set => SetValue(TitleProperty, value);
        }

        public string Hint
        {
            get => (string)GetValue(HintProperty);
            set => SetValue(HintProperty, value);
        }

        public PlaceholderPane()
        {
            InitializeComponent();
        }
    }
}
