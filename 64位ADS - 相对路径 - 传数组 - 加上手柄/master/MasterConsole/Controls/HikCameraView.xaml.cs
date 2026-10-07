using System;
using System.Drawing;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Threading;
using MasterConsole.Services;
using Forms = System.Windows.Forms;

namespace MasterConsole.Controls
{
    public partial class HikCameraView : UserControl, IDisposable
    {
        private readonly DispatcherTimer _timer;
        private Forms.PictureBox _pictureBox;
        private HikCameraSession _session;
        private bool _started;

        public static readonly DependencyProperty CameraNameProperty = DependencyProperty.Register(nameof(CameraName), typeof(string), typeof(HikCameraView), new PropertyMetadata("海康"));
        public static readonly DependencyProperty StatusTextProperty = DependencyProperty.Register(nameof(StatusText), typeof(string), typeof(HikCameraView), new PropertyMetadata("未连接"));
        public static readonly DependencyProperty DetailTextProperty = DependencyProperty.Register(nameof(DetailText), typeof(string), typeof(HikCameraView), new PropertyMetadata("等待启动"));

        public string CameraName { get => (string)GetValue(CameraNameProperty); set => SetValue(CameraNameProperty, value); }
        public string StatusText { get => (string)GetValue(StatusTextProperty); private set => SetValue(StatusTextProperty, value); }
        public string DetailText { get => (string)GetValue(DetailTextProperty); private set => SetValue(DetailTextProperty, value); }

        public HikCameraView()
        {
            InitializeComponent();
            Loaded += OnLoaded;
            Unloaded += OnUnloaded;
            _timer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(500) };
            _timer.Tick += (s, e) => { _session?.RefreshStatus(); RefreshText(); };
        }

        private async void OnLoaded(object sender, RoutedEventArgs e)
        {
            if (_started) return;
            _started = true;
            _pictureBox = new Forms.PictureBox { Dock = Forms.DockStyle.Fill, BackColor = Color.Black, SizeMode = Forms.PictureBoxSizeMode.Zoom };
            FormsHost.Child = _pictureBox;
            int index = int.TryParse(Convert.ToString(Tag), out int parsed) ? parsed : 1;
            _session = new HikCameraSession(HikCameraSettings.Load(index));
            CameraName = _session.Settings.Name;
            _session.Changed += RefreshText;
            _timer.Start();
            await _session.StartAsync(_pictureBox.Handle);
            RefreshText();
        }

        private void OnUnloaded(object sender, RoutedEventArgs e) => Dispose();

        private void RefreshText()
        {
            if (!Dispatcher.CheckAccess()) { Dispatcher.BeginInvoke(new Action(RefreshText)); return; }
            if (_session == null) return;
            StatusText = _session.Status;
            DetailText = _session.Detail;
        }

        public void Dispose()
        {
            if (!_started) return;
            _started = false;
            _timer.Stop();
            if (_session != null) { _session.Changed -= RefreshText; _session.Dispose(); _session = null; }
            if (_pictureBox != null) { FormsHost.Child = null; _pictureBox.Dispose(); _pictureBox = null; }
        }
    }
}
