using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace MasterConsole.Services
{
    internal sealed class HikCameraSettings
    {
        public string Name { get; set; }
        public string Ip { get; set; }
        public int Port { get; set; } = 8000;
        public string UserName { get; set; } = "admin";
        public string Password { get; set; } = "";
        public int Channel { get; set; } = 1;
        public uint StreamType { get; set; } = 0;

        public static HikCameraSettings Load(int index)
        {
            var value = new HikCameraSettings {
                Name = "海康 " + index,
                Ip = index == 1 ? "192.168.1.32" : "192.168.1.64"
            };
            string path = FindConfig();
            if (path == null) return value;
            var map = ReadIni(path, "camera" + index);
            string text;
            if (map.TryGetValue("name", out text) && text.Length > 0) value.Name = text;
            if (map.TryGetValue("ip", out text) && text.Length > 0) value.Ip = text;
            if (map.TryGetValue("port", out text) && int.TryParse(text, out int port)) value.Port = port;
            if (map.TryGetValue("username", out text)) value.UserName = text;
            if (map.TryGetValue("password", out text)) value.Password = text;
            if (map.TryGetValue("channel", out text) && int.TryParse(text, out int channel)) value.Channel = channel;
            if (map.TryGetValue("stream", out text) && uint.TryParse(text, out uint stream)) value.StreamType = stream;
            return value;
        }

        private static string FindConfig()
        {
            string dir = AppDomain.CurrentDomain.BaseDirectory;
            for (int i = 0; i < 10 && !string.IsNullOrEmpty(dir); i++)
            {
                string path = Path.Combine(dir, "config", "hik_cameras.ini");
                if (File.Exists(path)) return path;
                dir = Directory.GetParent(dir.TrimEnd(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar))?.FullName;
            }
            return null;
        }

        private static Dictionary<string, string> ReadIni(string path, string section)
        {
            var result = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
            string current = "";
            foreach (string raw in File.ReadAllLines(path, Encoding.UTF8))
            {
                string line = raw.Trim();
                if (line.Length == 0 || line.StartsWith("#") || line.StartsWith(";")) continue;
                if (line.StartsWith("[") && line.EndsWith("]")) { current = line.Substring(1, line.Length - 2).Trim(); continue; }
                if (!string.Equals(current, section, StringComparison.OrdinalIgnoreCase)) continue;
                int equals = line.IndexOf('=');
                if (equals > 0) result[line.Substring(0, equals).Trim()] = line.Substring(equals + 1).Trim();
            }
            return result;
        }
    }

    internal static class HikCameraRuntime
    {
        private static readonly object Sync = new object();
        private static readonly ConcurrentDictionary<int, HikCameraSession> Sessions = new ConcurrentDictionary<int, HikCameraSession>();
        private static HikNetSdk.ExceptionCallback _exceptionCallback;
        private static int _users;
        private static bool _initialized;

        internal static bool TryInitialize(out string error)
        {
            lock (Sync)
            {
                error = null;
                if (_initialized) { _users++; return true; }
                try
                {
                    if (!HikNetSdk.NET_DVR_Init()) { error = "NET_DVR_Init失败，SDK错误码 " + HikNetSdk.NET_DVR_GetLastError(); return false; }
                    _exceptionCallback = OnException;
                    HikNetSdk.NET_DVR_SetExceptionCallBack_V30(0, IntPtr.Zero, _exceptionCallback, IntPtr.Zero);
                    _initialized = true;
                    _users = 1;
                    return true;
                }
                catch (DllNotFoundException ex) { error = "HCNetSDK.dll加载失败：" + ex.Message; return false; }
                catch (BadImageFormatException ex) { error = "海康SDK位数不匹配：" + ex.Message; return false; }
                catch (Exception ex) { error = "海康SDK初始化失败：" + ex.Message; return false; }
            }
        }

        internal static void Add(HikCameraSession session) { Sessions[session.RealHandle] = session; }
        internal static void Remove(HikCameraSession session) { if (session.RealHandle != 0) Sessions.TryRemove(session.RealHandle, out _); }

        internal static void Release()
        {
            lock (Sync)
            {
                if (!_initialized || --_users > 0) return;
                _initialized = false;
                _exceptionCallback = null;
                HikNetSdk.NET_DVR_Cleanup();
            }
        }

        private static void OnException(uint type, int userId, int handle, IntPtr user)
        {
            HikCameraSession session;
            if (handle != 0 && Sessions.TryGetValue(handle, out session)) session.NotifyException(type);
        }
    }

    internal sealed class HikCameraSession : IDisposable
    {
        private readonly object _sync = new object();
        private readonly HikNetSdk.RealDataCallback _dataCallback;
        private readonly CancellationTokenSource _stop = new CancellationTokenSource();
        private int _userId = -1;
        private int _realHandle = -1;
        private bool _disposed;
        private bool _runtimeHeld;
        private Task _operation;
        private long _lastDataTicks;
        private long _previewStartedTicks;

        internal HikCameraSettings Settings { get; }
        internal int RealHandle => _realHandle < 0 ? 0 : _realHandle;
        internal DateTime LastDataUtc => _lastDataTicks == 0 ? DateTime.MinValue : new DateTime(_lastDataTicks, DateTimeKind.Utc);
        internal string Status { get; private set; } = "未连接";
        internal string Detail { get; private set; } = "";
        internal event Action Changed;

        internal HikCameraSession(HikCameraSettings settings)
        {
            Settings = settings;
            _dataCallback = OnData;
        }

        internal async Task StartAsync(IntPtr playWindow)
        {
            SetStatus("连接中", "正在登录设备");
            string error;
            if (!HikCameraRuntime.TryInitialize(out error)) { SetStatus("失败", error); return; }
            _runtimeHeld = true;
            try
            {
                _operation = Task.Run(() => LoginAndPreview(playWindow), _stop.Token);
                await _operation.ConfigureAwait(false);
            }
            catch (OperationCanceledException) { }
            catch (Exception ex) { SetStatus("失败", ex.Message); }
            finally
            {
                if (_disposed && _userId < 0 && _realHandle < 0) ReleaseRuntime();
            }
        }

        private void LoginAndPreview(IntPtr playWindow)
        {
            if (_stop.IsCancellationRequested) return;
            var login = new HikNetSdk.UserLoginInfo {
                DeviceAddress = Bytes(Settings.Ip, 129), UserName = Bytes(Settings.UserName, 64), Password = Bytes(Settings.Password, 64),
                Port = (ushort)Settings.Port, LoginMode = 0, UseTransport = 0, AsyncLogin = false, Reserved = new byte[119]
            };
            var device = new HikNetSdk.DeviceInfoV40 { Device = new HikNetSdk.DeviceInfoV30 { Serial = new byte[48], Reserved = new byte[9] }, Reserved = new byte[243] };
            int user = HikNetSdk.NET_DVR_Login_V40(ref login, ref device);
            if (user < 0) { SetStatus("失败", "登录失败，SDK错误码 " + HikNetSdk.NET_DVR_GetLastError()); ReleaseRuntime(); return; }
            lock (_sync) { if (_disposed) { HikNetSdk.NET_DVR_Logout(user); ReleaseRuntime(); return; } _userId = user; }
            var preview = new HikNetSdk.PreviewInfo {
                Channel = Settings.Channel, StreamType = Settings.StreamType, LinkMode = 1, PlayWnd = playWindow,
                Blocked = false, PassbackRecord = false, StreamId = new byte[32], DisplayBufNum = 1, Reserved = new byte[215]
            };
            int real = HikNetSdk.NET_DVR_RealPlay_V40(user, ref preview, _dataCallback, IntPtr.Zero);
            if (real < 0) { SetStatus("失败", "预览启动失败，SDK错误码 " + HikNetSdk.NET_DVR_GetLastError()); HikNetSdk.NET_DVR_Logout(user); ReleaseRuntime(); lock (_sync) _userId = -1; return; }
            lock (_sync) { if (_disposed) { HikNetSdk.NET_DVR_StopRealPlay(real); HikNetSdk.NET_DVR_Logout(user); ReleaseRuntime(); return; } _realHandle = real; }
            HikCameraRuntime.Add(this);
            Interlocked.Exchange(ref _previewStartedTicks, DateTime.UtcNow.Ticks);
            SetStatus("连接中", "已启动UDP预览，等待媒体帧");
        }

        private static byte[] Bytes(string value, int size)
        {
            var result = new byte[size];
            if (!string.IsNullOrEmpty(value)) Array.Copy(Encoding.Default.GetBytes(value), result, Math.Min(size - 1, Encoding.Default.GetByteCount(value)));
            return result;
        }

        private void OnData(int realHandle, uint dataType, IntPtr buffer, uint bufferSize, IntPtr user)
        {
            if (dataType == HikNetSdk.StreamData && bufferSize > 0)
            {
                Interlocked.Exchange(ref _lastDataTicks, DateTime.UtcNow.Ticks);
                if (Status != "在线") SetStatus("在线", "已收到媒体帧；UDP为SDK请求参数");
            }
        }

        internal void NotifyException(uint type)
        {
            if (type == HikNetSdk.Reconnect || type == HikNetSdk.RealPlayReconnect) SetStatus("连接中", "SDK正在重连");
            else if (type == HikNetSdk.RealPlayNetClose || type == HikNetSdk.RealPlayNoData || type == HikNetSdk.PreviewException) SetStatus("过期", "预览异常，等待SDK恢复");
        }

        internal void RefreshStatus()
        {
            DateTime now = DateTime.UtcNow;
            DateTime previewStarted = _previewStartedTicks == 0 ? DateTime.MinValue : new DateTime(_previewStartedTicks, DateTimeKind.Utc);
            if ((Status == "在线" && LastDataUtc != DateTime.MinValue && now - LastDataUtc > TimeSpan.FromSeconds(5)) ||
                (Status == "连接中" && previewStarted != DateTime.MinValue && now - previewStarted > TimeSpan.FromSeconds(5)))
                SetStatus("过期", "5秒未收到媒体帧");
        }

        private void ReleaseRuntime()
        {
            if (!_runtimeHeld) return;
            _runtimeHeld = false;
            HikCameraRuntime.Release();
        }

        private void SetStatus(string status, string detail)
        {
            Status = status; Detail = detail; Changed?.Invoke();
        }

        public void Dispose()
        {
            lock (_sync)
            {
                if (_disposed) return;
                _disposed = true;
                int real = _realHandle, user = _userId;
                _realHandle = -1; _userId = -1;
                HikCameraRuntime.Remove(this);
                if (real >= 0) HikNetSdk.NET_DVR_StopRealPlay(real);
                if (user >= 0) HikNetSdk.NET_DVR_Logout(user);
                _stop.Cancel();
                if (_operation == null || _operation.IsCompleted) ReleaseRuntime();
                SetStatus("未连接", "已关闭");
            }
            _stop.Dispose();
        }
    }
}
