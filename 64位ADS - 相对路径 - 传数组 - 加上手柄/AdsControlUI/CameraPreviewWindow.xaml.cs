using System;
using System.IO;
using System.IO.MemoryMappedFiles;
using System.Threading;
using System.Windows;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Threading;

namespace AdsControlUI
{
    public partial class CameraPreviewWindow : Window
    {
        private const string MappingName = "Local\\ADS_Action4_Preview_v1";
        private const string EventName = "Local\\ADS_Action4_PreviewEvent_v1";
        private const uint PreviewMagic = 0x56503441;
        private const uint PreviewVersion = 1;
        private const int HeaderSize = 64;
        private const int NativeWidth = 1280;
        private const int NativeHeight = 720;
        private const int NativeStride = NativeWidth * 4;
        private const int FrameBytes = NativeStride * NativeHeight;

        private MemoryMappedFile _mapping;
        private MemoryMappedViewAccessor _accessor;
        private EventWaitHandle _frameEvent;
        private Thread _readerThread;
        private volatile bool _stopRequested;
        private int _dispatchPending;
        private byte[] _pixels;
        private WriteableBitmap _bitmap;
        private long _lastSequence = -1;
        private double _zoom = 1.0;
        private double _fitScale = 1.0;
        private bool _dragging;
        private Point _dragStart;
        private double _dragHorizontalOffset;
        private double _dragVerticalOffset;

        public CameraPreviewWindow()
        {
            InitializeComponent();
            Loaded += CameraPreviewWindow_Loaded;
            Closed += CameraPreviewWindow_Closed;
            DeviceStatusText.Text = "正在打开摄像设备：OsmoAction4";
            ParameterText.Text = "等待实际视频参数";
            RecordingText.Text = "未录像";
            FrameText.Text = "帧 0 · 丢帧 0";
        }

        public void OnState(VisState state)
        {
            DeviceStatusText.Text = CameraStatus(state);
            ParameterText.Text = CameraParameters(state);
            RecordingText.Text = state.camera_recording
                ? $"录像 {FormatElapsed(state.camera_recording_elapsed_us)}"
                : "未录像";
            FrameText.Text = $"帧 {state.camera_frame_count} · 丢帧 {state.camera_dropped_frames}";

            if (state.camera_state == 4 || state.camera_state == 5 || state.camera_state == 6)
            {
                PreviewOverlayText.Text = DeviceStatusText.Text;
                PreviewOverlay.Visibility = Visibility.Visible;
            }
            else if (_lastSequence < 0)
            {
                PreviewOverlayText.Text = DeviceStatusText.Text;
                PreviewOverlay.Visibility = Visibility.Visible;
            }
        }

        private void CameraPreviewWindow_Loaded(object sender, RoutedEventArgs e)
        {
            _stopRequested = false;
            _readerThread = new Thread(ReaderLoop)
            {
                IsBackground = true,
                Name = "Action4PreviewReader"
            };
            _readerThread.Start();
            UpdateImageSize();
        }

        private void CameraPreviewWindow_Closed(object sender, EventArgs e)
        {
            _stopRequested = true;
            try { _frameEvent?.Set(); } catch { }
            _readerThread?.Join(1500);
            _accessor?.Dispose();
            _mapping?.Dispose();
            _frameEvent?.Dispose();
        }

        private void ReaderLoop()
        {
            while (!_stopRequested)
            {
                if (_accessor == null || _frameEvent == null)
                {
                    TryOpenSharedPreview();
                    if (_accessor == null || _frameEvent == null)
                    {
                        Thread.Sleep(250);
                        continue;
                    }
                }

                bool signaled;
                try { signaled = _frameEvent.WaitOne(250); }
                catch { signaled = false; }
                if (!signaled || _stopRequested) continue;
                if (Interlocked.Exchange(ref _dispatchPending, 1) == 0)
                {
                    Dispatcher.BeginInvoke(new Action(() =>
                    {
                        try { ReadLatestFrame(); }
                        finally { Interlocked.Exchange(ref _dispatchPending, 0); }
                    }), DispatcherPriority.Render);
                }
            }
        }

        private void TryOpenSharedPreview()
        {
            MemoryMappedFile mapping = null;
            MemoryMappedViewAccessor accessor = null;
            EventWaitHandle frameEvent = null;
            try
            {
                mapping = MemoryMappedFile.OpenExisting(MappingName, MemoryMappedFileRights.Read);
                accessor = mapping.CreateViewAccessor(0, 0, MemoryMappedFileAccess.Read);
                frameEvent = EventWaitHandle.OpenExisting(EventName);
                _mapping = mapping;
                _accessor = accessor;
                _frameEvent = frameEvent;
                mapping = null;
                accessor = null;
                frameEvent = null;
            }
            catch (FileNotFoundException) { }
            catch (WaitHandleCannotBeOpenedException) { }
            catch (UnauthorizedAccessException) { }
            catch (IOException) { }
            finally
            {
                frameEvent?.Dispose();
                accessor?.Dispose();
                mapping?.Dispose();
            }
        }

        private void ReadLatestFrame()
        {
            if (_accessor == null) return;
            try
            {
                uint magic = _accessor.ReadUInt32(0);
                uint version = _accessor.ReadUInt32(4);
                uint headerSize = _accessor.ReadUInt32(8);
                int width = _accessor.ReadInt32(12);
                int height = _accessor.ReadInt32(16);
                int stride = _accessor.ReadInt32(20);
                int pixelFormat = _accessor.ReadInt32(24);
                if (magic != PreviewMagic || version != PreviewVersion || headerSize != HeaderSize ||
                    width != NativeWidth || height != NativeHeight || stride != NativeStride || pixelFormat != 1)
                    return;

                long sequenceBefore = _accessor.ReadInt64(32);
                int activeIndex = _accessor.ReadInt32(28);
                if (sequenceBefore == _lastSequence || (activeIndex != 0 && activeIndex != 1)) return;
                if (_pixels == null) _pixels = new byte[FrameBytes];
                long frameOffset = HeaderSize + (long)activeIndex * FrameBytes;
                int read = _accessor.ReadArray(frameOffset, _pixels, 0, FrameBytes);
                long sequenceAfter = _accessor.ReadInt64(32);
                int activeAfter = _accessor.ReadInt32(28);
                if (read != FrameBytes || sequenceBefore != sequenceAfter || activeIndex != activeAfter) return;

                if (_bitmap == null)
                {
                    _bitmap = new WriteableBitmap(NativeWidth, NativeHeight, 96, 96, PixelFormats.Bgra32, null);
                    PreviewImage.Source = _bitmap;
                    UpdateImageSize();
                }
                _bitmap.WritePixels(new Int32Rect(0, 0, NativeWidth, NativeHeight), _pixels, NativeStride, 0);
                _lastSequence = sequenceAfter;
                PreviewOverlay.Visibility = Visibility.Collapsed;
            }
            catch (ObjectDisposedException) { }
            catch (IOException) { }
        }

        private void Fit_Click(object sender, RoutedEventArgs e)
        {
            _zoom = 1.0;
            UpdateImageSize();
            PreviewScrollViewer.ScrollToHorizontalOffset(0);
            PreviewScrollViewer.ScrollToVerticalOffset(0);
        }

        private void PreviewScrollViewer_SizeChanged(object sender, SizeChangedEventArgs e) => UpdateImageSize();

        private void UpdateImageSize()
        {
            double viewportWidth = Math.Max(1.0, PreviewScrollViewer.ViewportWidth > 0
                ? PreviewScrollViewer.ViewportWidth
                : PreviewScrollViewer.ActualWidth);
            double viewportHeight = Math.Max(1.0, PreviewScrollViewer.ViewportHeight > 0
                ? PreviewScrollViewer.ViewportHeight
                : PreviewScrollViewer.ActualHeight);
            _fitScale = Math.Min(viewportWidth / NativeWidth, viewportHeight / NativeHeight);
            if (!IsFinite(_fitScale) || _fitScale <= 0) _fitScale = 1.0;
            PreviewImage.Width = NativeWidth * _fitScale * _zoom;
            PreviewImage.Height = NativeHeight * _fitScale * _zoom;
            ZoomText.Text = $"缩放 {_zoom:F2}×";
        }

        private void PreviewScrollViewer_PreviewMouseWheel(object sender, MouseWheelEventArgs e)
        {
            double oldZoom = _zoom;
            double factor = e.Delta > 0 ? 1.2 : 1.0 / 1.2;
            _zoom = Math.Max(1.0, Math.Min(8.0, _zoom * factor));
            if (Math.Abs(_zoom - oldZoom) < 0.0001) return;

            Point mouse = e.GetPosition(PreviewScrollViewer);
            double contentX = PreviewScrollViewer.HorizontalOffset + mouse.X;
            double contentY = PreviewScrollViewer.VerticalOffset + mouse.Y;
            double ratio = _zoom / oldZoom;
            UpdateImageSize();
            Dispatcher.BeginInvoke(new Action(() =>
            {
                PreviewScrollViewer.ScrollToHorizontalOffset(contentX * ratio - mouse.X);
                PreviewScrollViewer.ScrollToVerticalOffset(contentY * ratio - mouse.Y);
            }), DispatcherPriority.Loaded);
            e.Handled = true;
        }

        private void PreviewScrollViewer_PreviewMouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            _dragging = true;
            _dragStart = e.GetPosition(PreviewScrollViewer);
            _dragHorizontalOffset = PreviewScrollViewer.HorizontalOffset;
            _dragVerticalOffset = PreviewScrollViewer.VerticalOffset;
            PreviewScrollViewer.CaptureMouse();
            Mouse.OverrideCursor = Cursors.SizeAll;
            e.Handled = true;
        }

        private void PreviewScrollViewer_PreviewMouseMove(object sender, MouseEventArgs e)
        {
            if (!_dragging || e.LeftButton != MouseButtonState.Pressed) return;
            Point current = e.GetPosition(PreviewScrollViewer);
            PreviewScrollViewer.ScrollToHorizontalOffset(_dragHorizontalOffset - (current.X - _dragStart.X));
            PreviewScrollViewer.ScrollToVerticalOffset(_dragVerticalOffset - (current.Y - _dragStart.Y));
        }

        private void PreviewScrollViewer_PreviewMouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            _dragging = false;
            PreviewScrollViewer.ReleaseMouseCapture();
            Mouse.OverrideCursor = null;
            e.Handled = true;
        }

        private static bool IsFinite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);

        private static string FormatElapsed(ulong elapsedUs) =>
            TimeSpan.FromMilliseconds(elapsedUs / 1000.0).ToString(@"hh\:mm\:ss");

        private static string CameraStatus(VisState state)
        {
            switch (state.camera_state)
            {
                case 1: return "正在打开摄像设备：OsmoAction4";
                case 2: return "摄像设备预览中：OsmoAction4";
                case 3: return "摄像设备录像中：OsmoAction4";
                case 4: return "无法找到摄像设备：OsmoAction4";
                case 5: return "摄像设备已断开：OsmoAction4";
                case 6: return $"摄像设备错误：OsmoAction4（0x{unchecked((uint)state.camera_error_code):X8}）";
                default: return "摄像设备未打开：OsmoAction4";
            }
        }

        private static string CameraParameters(VisState state)
        {
            if (state.camera_width <= 0 || state.camera_height <= 0) return "等待实际视频参数";
            double fps = state.camera_fps_denominator > 0
                ? (double)state.camera_fps_numerator / state.camera_fps_denominator
                : 0.0;
            string input = state.camera_input_format == 1 ? "H.264" :
                (state.camera_input_format == 2 ? "MJPEG" :
                (state.camera_input_format == 3 ? "RGB32" : "未知"));
            return $"{state.camera_width}×{state.camera_height} · {fps:F1} fps · 输入 {input} · 输出 H.264 / 无音频";
        }
    }
}
