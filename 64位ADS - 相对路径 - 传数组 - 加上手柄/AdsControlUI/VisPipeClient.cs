using System;
using System.IO;
using System.IO.Pipes;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;

namespace AdsControlUI
{
    public class VisPipeClient : IDisposable
    {
        private const string PipeName = "ADS_Control_Vis";
        // 必须与 C++ VisState 的 static_assert 一致；新旧管道协议不可混用。
		private const int VisStateWireSize = 499;
        private const uint CommandMagic = 0x31434D56;
        private const ushort CommandVersion = 1;
        private const ushort CommandHeaderSize = 24;
        private const int MaxCommandPayloadBytes = 1024;
        private NamedPipeClientStream _pipe;
        private Thread _readThread;
        private volatile bool _stopRequested;
        private readonly object _stateLock = new object();
        private VisState _latestState;
        private bool _hasState;
        private readonly object _writeLock = new object();

        public event Action<VisState> StateReceived;
        public bool IsConnected => _pipe?.IsConnected == true;

        public void Start()
        {
            if (Marshal.SizeOf<VisState>() != VisStateWireSize)
                throw new InvalidOperationException("VisState 管道协议大小不匹配，请同时更新 C++ 与 WPF 程序。");
            _stopRequested = false;
            _readThread = new Thread(ReadLoop) { IsBackground = true, Name = "VisPipeReader" };
            _readThread.Start();
        }

        public void Stop()
        {
            _stopRequested = true;
            try { _pipe?.Close(); } catch { }
            _readThread?.Join(2000);
        }

        public bool TryGetLatestState(out VisState state)
        {
            lock (_stateLock)
            {
                state = _latestState;
                return _hasState;
            }
        }

        public void SendCommand(VisCommandType type, int param1 = 0, int param2 = 0, string payloadUtf8 = null)
        {
            if (_pipe == null || !_pipe.IsConnected) return;
            byte[] payload = string.IsNullOrEmpty(payloadUtf8)
                ? Array.Empty<byte>()
                : Encoding.UTF8.GetBytes(payloadUtf8);
            if (payload.Length > MaxCommandPayloadBytes)
                throw new ArgumentException("命令 UTF-8 负载超过协议上限。", nameof(payloadUtf8));

            byte[] buf;
            using (var stream = new MemoryStream(CommandHeaderSize + payload.Length))
            using (var writer = new BinaryWriter(stream, Encoding.UTF8, true))
            {
                writer.Write(CommandMagic);
                writer.Write(CommandVersion);
                writer.Write(CommandHeaderSize);
                writer.Write((int)type);
                writer.Write(param1);
                writer.Write(param2);
                writer.Write((uint)payload.Length);
                writer.Write(payload);
                writer.Flush();
                buf = stream.ToArray();
            }
            try
            {
                lock (_writeLock)
                {
                    _pipe.Write(buf, 0, buf.Length);
                    _pipe.Flush();
                }
            }
            catch { }
        }

        private void ReadLoop()
        {
            int stateSize = Marshal.SizeOf<VisState>();
            byte[] buf = new byte[stateSize];

            while (!_stopRequested)
            {
                try
                {
                    _pipe = new NamedPipeClientStream(".", PipeName, PipeDirection.InOut, PipeOptions.None);
                    _pipe.Connect(1000);
                    _pipe.ReadMode = PipeTransmissionMode.Message;
                }
                catch
                {
                    Thread.Sleep(500);
                    continue;
                }

                try
                {
                    while (!_stopRequested && _pipe.IsConnected)
                    {
                        int bytesRead = 0;
                        int offset = 0;
                        do
                        {
                            int n = _pipe.Read(buf, offset, buf.Length - offset);
                            if (n == 0) throw new IOException("Pipe closed");
                            offset += n;
                        } while (!_pipe.IsMessageComplete && offset < buf.Length);

                        bytesRead = offset;
                        if (bytesRead == stateSize)
                        {
                            var state = BytesToStruct<VisState>(buf);
                            lock (_stateLock)
                            {
                                _latestState = state;
                                _hasState = true;
                            }
                            StateReceived?.Invoke(state);
                        }
                    }
                }
                catch { }

                try { _pipe?.Dispose(); } catch { }
                _pipe = null;

                if (!_stopRequested) Thread.Sleep(500);
            }
        }

        private static T BytesToStruct<T>(byte[] buf) where T : struct
        {
            int size = Marshal.SizeOf<T>();
            IntPtr ptr = Marshal.AllocHGlobal(size);
            try
            {
                Marshal.Copy(buf, 0, ptr, size);
                return Marshal.PtrToStructure<T>(ptr);
            }
            finally
            {
                Marshal.FreeHGlobal(ptr);
            }
        }

        public void Dispose()
        {
            Stop();
        }
    }
}
