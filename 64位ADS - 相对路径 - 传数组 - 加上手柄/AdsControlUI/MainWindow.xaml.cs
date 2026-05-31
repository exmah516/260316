using System.Windows;

namespace AdsControlUI
{
    public partial class MainWindow : Window
    {
        private readonly AdsControlViewModel _vm;
        private readonly bool[] _cylOverride = new bool[4];

        public MainWindow()
        {
            InitializeComponent();
            _vm = new AdsControlViewModel();
            DataContext = _vm;
        }

        protected override void OnClosed(System.EventArgs e)
        {
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

        private void ModeCatheter_Click(object sender, RoutedEventArgs e) => _vm.RequestModeSwitch(0);
        private void ModeIndependent_Click(object sender, RoutedEventArgs e) => _vm.RequestModeSwitch(1);
        private void ModeCooperative_Click(object sender, RoutedEventArgs e) => _vm.RequestModeSwitch(2);

        private void Zero_Click(object sender, RoutedEventArgs e) => _vm.ZeroForceSensor();
        private void FfToggle_Click(object sender, RoutedEventArgs e) => _vm.ToggleForceFeedback();
    }
}
