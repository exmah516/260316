using System;
using System.IO;
using System.IO.Pipes;
using System.Runtime.InteropServices;
using System.Threading;

namespace AdsControlUI
{
    public class VisPipeClient : IDisposable
    {
        private const string PipeName = "ADS_Control_Vis";
        private NamedPipeClientStream _pipe;
        private Thread _readThread;
        private volatile bool _stopRequested;
        private readonly object _stateLock = new object();
        private VisState _latestState;
        private bool _hasState;

        public event Action<VisState> StateReceived;
        public bool IsConnected => _pipe?.IsConnected == true;

        public void Start()
        {
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

        public void SendCommand(VisCommandType type, int param1 = 0, int param2 = 0)
        {
            if (_pipe == null || !_pipe.IsConnected) return;
            var cmd = new VisCommand { type = type, param1 = param1, param2 = param2 };
            byte[] buf = StructToBytes(cmd);
            try
            {
                _pipe.Write(buf, 0, buf.Length);
                _pipe.Flush();
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

        private static byte[] StructToBytes<T>(T obj) where T : struct
        {
            int size = Marshal.SizeOf<T>();
            byte[] buf = new byte[size];
            IntPtr ptr = Marshal.AllocHGlobal(size);
            try
            {
                Marshal.StructureToPtr(obj, ptr, false);
                Marshal.Copy(ptr, buf, 0, size);
            }
            finally
            {
                Marshal.FreeHGlobal(ptr);
            }
            return buf;
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
