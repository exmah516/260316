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
		private const int VisStateWireSize = 883;
        private const uint CommandMagic = 0x31434D56;
        private const ushort CommandVersion = 1;
        private const ushort CommandHeaderSize = 24;
        private const int MaxCommandPayloadBytes = 1024;
        private volatile NamedPipeClientStream _pipe;
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
            DisconnectPipe(_pipe);
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

        public bool SendCommand(VisCommandType type, int param1 = 0, int param2 = 0, string payloadUtf8 = null)
        {
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
            NamedPipeClientStream pipe;
            lock (_writeLock)
            {
                pipe = _pipe;
                if (pipe == null || !pipe.IsConnected)
                    return false;

                try
                {
                    pipe.Write(buf, 0, buf.Length);
                    pipe.Flush();
                    return true;
                }
                catch { }
            }

            // 写失败后立即使本连接失效，让读线程重建管道，避免 UI 仍把坏连接当作可用。
            DisconnectPipe(pipe);
            return false;
        }

        private void ReadLoop()
        {
            int stateSize = Marshal.SizeOf<VisState>();
            byte[] buf = new byte[stateSize];

            while (!_stopRequested)
            {
                NamedPipeClientStream pipe = null;
                try
                {
                    pipe = new NamedPipeClientStream(".", PipeName, PipeDirection.InOut, PipeOptions.None);
                    pipe.Connect(1000);
                    pipe.ReadMode = PipeTransmissionMode.Message;
                    if (_stopRequested)
                    {
                        try { pipe.Dispose(); } catch { }
                        break;
                    }
                    lock (_writeLock)
                    {
                        _pipe = pipe;
                    }
                }
                catch
                {
                    try { pipe?.Dispose(); } catch { }
                    Thread.Sleep(500);
                    continue;
                }

                try
                {
                    while (!_stopRequested && pipe.IsConnected)
                    {
                        int bytesRead = 0;
                        int offset = 0;
                        do
                        {
                            int n = pipe.Read(buf, offset, buf.Length - offset);
                            if (n == 0) throw new IOException("Pipe closed");
                            offset += n;
                        } while (!pipe.IsMessageComplete && offset < buf.Length);

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

                DisconnectPipe(pipe);

                if (!_stopRequested) Thread.Sleep(500);
            }
        }

        private void DisconnectPipe(NamedPipeClientStream pipe)
        {
            if (pipe == null) return;
            lock (_writeLock)
            {
                if (ReferenceEquals(_pipe, pipe))
                    _pipe = null;
            }
            try { pipe.Dispose(); } catch { }
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
