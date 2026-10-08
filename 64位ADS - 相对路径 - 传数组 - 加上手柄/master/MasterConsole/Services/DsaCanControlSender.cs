using System;
using System.Net;
using System.Net.Sockets;
using System.Runtime.InteropServices;

namespace MasterConsole.Services
{
    /// <summary>旧项目 DSA CAN → control_cmd → UDP 31002 原始发送路径。</summary>
    public sealed class DsaCanControlSender : IDisposable
    {
        private const int UdpPort = 31002;
        private readonly string _host;
        private readonly object _sync = new object();
        private Ercp.Module.CCan _mCan;
        private UdpClient _udp;

        public DsaCanControlSender(string host) => _host = host ?? throw new ArgumentNullException(nameof(host));

        public void Open()
        {
            lock (_sync)
            {
                if (_mCan != null) return;
                if (Marshal.SizeOf(typeof(CCFrame)) != 20 || Marshal.SizeOf(typeof(control_cmd)) != 88)
                    throw new InvalidOperationException("旧 control_cmd/CCFrame 内存布局不一致。");

                _mCan = new Ercp.Module.CCan();
                int iErr = _mCan.Open();
                _udp = new UdpClient(AddressFamily.InterNetwork);
                _udp.Connect(IPAddress.Parse(_host), UdpPort);
                _ = iErr; // 旧项目保持 Open 返回值不参与后续 Recv/发送判定。
            }
        }

        public void PollAndSend()
        {
            lock (_sync)
            {
                if (_mCan == null || _udp == null) return;

                var data = new control_cmd
                {
                    ccF = new CCFrame { data = new byte[8] }
                };

                int aa = _mCan.Recv(
                    out byte type,
                    out byte length,
                    out UInt32 id,
                    out UInt32 timestamp,
                    out byte[] frameData);

                if (aa == 0)
                {
                    data.ccF.type = type;
                    data.ccF.length = length;
                    data.ccF.id = id;
                    data.ccF.timestamp = timestamp;
                    for (int i = 0; i < frameData.Length && i < data.ccF.data.Length; i++)
                        data.ccF.data[i] = frameData[i];
                }

                data.time = (DateTime.UtcNow.Ticks - DateTime.MinValue.Ticks) / 10000000.0;

                int size = Marshal.SizeOf(data);
                byte[] array = new byte[size];
                IntPtr ptr = Marshal.AllocHGlobal(size);
                try
                {
                    Marshal.StructureToPtr(data, ptr, true);
                    Marshal.Copy(ptr, array, 0, size);
                }
                finally
                {
                    Marshal.FreeHGlobal(ptr);
                }

                try { _udp.Send(array, array.Length); }
                catch (SocketException) { }
                catch (ObjectDisposedException) { }
            }
        }

        public void Dispose()
        {
            lock (_sync)
            {
                try { _udp?.Close(); } catch { }
                try { _mCan?.Dispose(); } catch { }
                _udp = null;
                _mCan = null;
            }
        }
    }
}
