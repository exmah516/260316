using System;
using System.IO;
using System.Threading;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media.Imaging;
using System.Windows.Threading;

namespace MasterConsole.Controls
{
    /// <summary>独立 DSA JPEG 影像入口；接入发送端前保持等待状态。</summary>
    public partial class DsaImageView : UserControl, IDisposable
    {
        private readonly DispatcherTimer _timer;
        private readonly object _decodeSync = new object();
        private PendingFrame _pending;
        private DecodedFrame _decoded;
        private bool _decodeRunning;
        private DateTime _lastDisplayedUtc;
        private bool _disposed;

        private sealed class PendingFrame
        {
            internal readonly byte[] Bytes;
            internal readonly DateTime ReceivedUtc;
            internal PendingFrame(byte[] bytes, DateTime receivedUtc) { Bytes = bytes; ReceivedUtc = receivedUtc; }
        }

        private sealed class DecodedFrame
        {
            internal readonly BitmapImage Image;
            internal readonly DateTime ReceivedUtc;
            internal DecodedFrame(BitmapImage image, DateTime receivedUtc) { Image = image; ReceivedUtc = receivedUtc; }
        }

        public DsaImageView()
        {
            InitializeComponent();
            _timer = new DispatcherTimer(DispatcherPriority.Render) { Interval = TimeSpan.FromMilliseconds(50) };
            _timer.Tick += OnTick;
            _timer.Start();
        }

        /// <summary>发布一帧完整 JPEG；复制输入，后台调用者只保留最新帧。</summary>
        public bool PublishJpeg(byte[] jpeg, DateTime receivedUtc)
        {
            if (_disposed || jpeg == null || jpeg.Length < 4 || jpeg[0] != 0xFF || jpeg[1] != 0xD8 ||
                jpeg[jpeg.Length - 2] != 0xFF || jpeg[jpeg.Length - 1] != 0xD9) return false;
            lock (_decodeSync)
            {
                _pending = new PendingFrame((byte[])jpeg.Clone(), receivedUtc.ToUniversalTime());
                if (_decodeRunning) return true;
                _decodeRunning = true;
            }
            System.Threading.Tasks.Task.Run(DecodeLoop);
            return true;
        }

        private void OnTick(object sender, EventArgs e)
        {
            if (_disposed) return;
            var frame = Interlocked.Exchange(ref _decoded, null);
            if (frame != null)
            {
                Image.Source = frame.Image;
                _lastDisplayedUtc = frame.ReceivedUtc;
                StatusText.Text = "在线";
                StatusText.Foreground = System.Windows.Media.Brushes.LightGreen;
                DetailText.Text = "JPEG帧已显示；独立影像链路";
            }
            if (_lastDisplayedUtc != DateTime.MinValue && DateTime.UtcNow - _lastDisplayedUtc > TimeSpan.FromSeconds(2))
            {
                StatusText.Text = "过期";
                StatusText.Foreground = System.Windows.Media.Brushes.Gold;
                DetailText.Text = "2秒未收到新的DSA帧，保留最后画面";
            }
        }

        private void DecodeLoop()
        {
            while (true)
            {
                PendingFrame frame;
                lock (_decodeSync)
                {
                    frame = _pending;
                    _pending = null;
                    if (frame == null) { _decodeRunning = false; return; }
                }
                try
                {
                    using (var stream = new MemoryStream(frame.Bytes, false))
                    {
                        var bitmap = new BitmapImage();
                        bitmap.BeginInit();
                        bitmap.CacheOption = BitmapCacheOption.OnLoad;
                        bitmap.StreamSource = stream;
                        bitmap.EndInit();
                        bitmap.Freeze();
                        Interlocked.Exchange(ref _decoded, new DecodedFrame(bitmap, frame.ReceivedUtc));
                    }
                }
                catch (Exception ex)
                {
                    Dispatcher.BeginInvoke(new Action(() =>
                    {
                        StatusText.Text = "失败";
                        StatusText.Foreground = System.Windows.Media.Brushes.IndianRed;
                        DetailText.Text = "JPEG解码失败：" + ex.GetType().Name;
                    }));
                }
            }
        }

        public void UpdateReceiverStatus(string status, string detail)
        {
            if (!Dispatcher.CheckAccess()) { Dispatcher.BeginInvoke(new Action(() => UpdateReceiverStatus(status, detail))); return; }
            if (_lastDisplayedUtc != DateTime.MinValue && status != "失败") return;
            StatusText.Text = status;
            DetailText.Text = detail;
        }

        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;
            _timer.Stop();
            lock (_decodeSync) _pending = null;
            Interlocked.Exchange(ref _decoded, null);
        }
    }
}
