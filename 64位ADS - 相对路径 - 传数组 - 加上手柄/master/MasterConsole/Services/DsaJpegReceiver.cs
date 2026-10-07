using System;
using System.Collections.Generic;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace MasterConsole.Services
{
    internal sealed class DsaJpegReceiverSettings
    {
        public int ListenPort0 { get; set; } = 36661;
        public int ListenPort1 { get; set; } = 36663;
        public int CommandPort0 { get; set; } = 36662;
        public int CommandPort1 { get; set; } = 36664;
        public string ScreenCutIp { get; set; } = "";
        public int MaxJpegBytes { get; set; } = 64 * 1024 * 1024;

        internal static DsaJpegReceiverSettings Load()
        {
            var result = new DsaJpegReceiverSettings();
            string path = FindConfig();
            if (path == null) return result;
            foreach (string raw in File.ReadAllLines(path, Encoding.UTF8))
            {
                string line = raw.Trim();
                if (line.Length == 0 || line.StartsWith("#") || line.StartsWith(";")) continue;
                int equals = line.IndexOf('=');
                if (equals <= 0) continue;
                string key = line.Substring(0, equals).Trim();
                string value = line.Substring(equals + 1).Trim();
                if (key.Equals("screenCutIp", StringComparison.OrdinalIgnoreCase)) result.ScreenCutIp = value;
                else if (key.Equals("listenPort0", StringComparison.OrdinalIgnoreCase) && int.TryParse(value, out int listen0) && listen0 > 0 && listen0 <= 65535) result.ListenPort0 = listen0;
                else if (key.Equals("listenPort1", StringComparison.OrdinalIgnoreCase) && int.TryParse(value, out int listen1) && listen1 > 0 && listen1 <= 65535) result.ListenPort1 = listen1;
                else if (key.Equals("commandPort0", StringComparison.OrdinalIgnoreCase) && int.TryParse(value, out int command0) && command0 > 0 && command0 <= 65535) result.CommandPort0 = command0;
                else if (key.Equals("commandPort1", StringComparison.OrdinalIgnoreCase) && int.TryParse(value, out int command1) && command1 > 0 && command1 <= 65535) result.CommandPort1 = command1;
                else if (key.Equals("maxJpegBytes", StringComparison.OrdinalIgnoreCase) && int.TryParse(value, out int max) && max > 0) result.MaxJpegBytes = max;
            }
            return result;
        }

        private static string FindConfig()
        {
            string dir = AppDomain.CurrentDomain.BaseDirectory;
            for (int i = 0; i < 10 && !string.IsNullOrEmpty(dir); i++)
            {
                string path = Path.Combine(dir, "config", "dsa_screencut.ini");
                if (File.Exists(path)) return path;
                dir = Directory.GetParent(dir.TrimEnd(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar))?.FullName;
            }
            return null;
        }
    }

    internal sealed class DsaJpegReceiver : IDisposable
    {
        private const int HeaderBytes = 5;
        private const int DatagramBytes = 8192;
        private readonly DsaJpegReceiverSettings _settings;
        private readonly object _sync = new object();
        private readonly CancellationTokenSource _stop = new CancellationTokenSource();
        private readonly Reassembler[] _reassemblers;
        private UdpClient[] _listeners;
        private Task[] _receiveTasks;
        private bool _started;
        private bool _disposed;

        internal event Action<int, byte[], DateTime> FrameReceived;
        internal event Action<int, string, string> StatusChanged;

        internal DsaJpegReceiver(DsaJpegReceiverSettings settings)
        {
            _settings = settings ?? new DsaJpegReceiverSettings();
            _reassemblers = new[] { new Reassembler(0, _settings.MaxJpegBytes, PublishFrame), new Reassembler(1, _settings.MaxJpegBytes, PublishFrame) };
        }

        internal void Start()
        {
            lock (_sync)
            {
                if (_started || _disposed) return;
                _started = true;
                _listeners = new UdpClient[2];
                _receiveTasks = new Task[2];
                Bind(0, _settings.ListenPort0);
                Bind(1, _settings.ListenPort1);
            }
        }

        private void Bind(int stream, int port)
        {
            try
            {
                _listeners[stream] = new UdpClient(new IPEndPoint(IPAddress.Any, port));
                StatusChanged?.Invoke(stream, "等待", "监听 UDP " + port);
                _receiveTasks[stream] = Task.Run(() => ReceiveLoop(stream, _listeners[stream]));
            }
            catch (Exception ex)
            {
                StatusChanged?.Invoke(stream, "失败", "UDP " + port + " 绑定失败：" + ex.GetType().Name);
            }
        }

        private async Task ReceiveLoop(int stream, UdpClient listener)
        {
            while (!_stop.IsCancellationRequested)
            {
                try
                {
                    UdpReceiveResult result = await listener.ReceiveAsync().ConfigureAwait(false);
                    if (result.Buffer.Length > DatagramBytes && result.Buffer.Length != HeaderBytes)
                    {
                        StatusChanged?.Invoke(stream, "失败", "收到超过8192字节的JPEG数据报");
                        _reassemblers[stream].Reset();
                        continue;
                    }
                    _reassemblers[stream].Accept(result.Buffer);
                }
                catch (ObjectDisposedException) { break; }
                catch (SocketException) { if (!_stop.IsCancellationRequested) StatusChanged?.Invoke(stream, "失败", "UDP接收失败"); break; }
                catch (Exception ex) { if (!_stop.IsCancellationRequested) StatusChanged?.Invoke(stream, "失败", "UDP接收异常：" + ex.GetType().Name); break; }
            }
        }

        private void PublishFrame(int stream, byte[] jpeg)
        {
            StatusChanged?.Invoke(stream, "在线", "已收到完整JPEG，正在解码");
            FrameReceived?.Invoke(stream, jpeg, DateTime.UtcNow);
        }

        internal bool SendSpeedCommand(bool fast)
        {
            if (string.IsNullOrWhiteSpace(_settings.ScreenCutIp)) return false;
            if (!IPAddress.TryParse(_settings.ScreenCutIp, out IPAddress address)) return false;
            bool sent = false;
            sent |= SendCommand(address, _settings.CommandPort0, fast ? new byte[] { 1, 0 } : new byte[] { 0, 0 });
            sent |= SendCommand(address, _settings.CommandPort1, fast ? new byte[] { 0, 1 } : new byte[] { 0, 0 });
            return sent;
        }

        private static bool SendCommand(IPAddress address, int port, byte[] command)
        {
            try
            {
                using (var client = new UdpClient())
                {
                    client.Send(command, command.Length, new IPEndPoint(address, port));
                    return true;
                }
            }
            catch { return false; }
        }

        public void Dispose()
        {
            lock (_sync)
            {
                if (_disposed) return;
                _disposed = true;
                _stop.Cancel();
                if (_listeners != null)
                    foreach (UdpClient listener in _listeners) try { listener?.Close(); } catch { }
                if (_listeners != null)
                    for (int i = 0; i < _listeners.Length; i++)
                        if (_listeners[i] != null) StatusChanged?.Invoke(i, "未连接", "已停止监听");
            }
            _stop.Dispose();
        }

        private sealed class Reassembler
        {
            private readonly int _stream;
            private readonly int _maxBytes;
            private readonly Action<int, byte[]> _complete;
            private byte[] _buffer;
            private int _expected;
            private int _received;
            private DateTime _startedUtc;

            internal Reassembler(int stream, int maxBytes, Action<int, byte[]> complete)
            {
                _stream = stream; _maxBytes = maxBytes; _complete = complete;
            }

            internal void Accept(byte[] datagram)
            {
                if (datagram == null || datagram.Length == 0) return;
                if (_expected == 0)
                {
                    if (datagram.Length != HeaderBytes || datagram[0] != (byte)_stream) return;
                    int length = (datagram[1] << 24) | (datagram[2] << 16) | (datagram[3] << 8) | datagram[4];
                    if (length < 4 || length > _maxBytes) return;
                    _expected = length;
                    _received = 0;
                    _buffer = new byte[length];
                    _startedUtc = DateTime.UtcNow;
                    return;
                }

                if (DateTime.UtcNow - _startedUtc > TimeSpan.FromSeconds(5) || datagram.Length > _expected - _received)
                {
                    Reset();
                    return;
                }
                Buffer.BlockCopy(datagram, 0, _buffer, _received, datagram.Length);
                _received += datagram.Length;
                if (_received != _expected) return;
                byte[] complete = _buffer;
                Reset();
                if (complete[0] == 0xFF && complete[1] == 0xD8 && complete[complete.Length - 2] == 0xFF && complete[complete.Length - 1] == 0xD9)
                    _complete(_stream, complete);
            }

            internal void Reset()
            {
                _buffer = null; _expected = 0; _received = 0; _startedUtc = DateTime.MinValue;
            }
        }
    }
}
