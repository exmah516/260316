using System;
using System.IO;
using System.Linq;
using MasterConsole.Protocol;

namespace MasterConsole.Protocol.Tests
{
    /// <summary>与 tools/gen_protocol_vectors.py 对拍的最小自检程序（无第三方测试框架依赖）。</summary>
    internal static class Program
    {
        private static int _fail;
        private static int _pass;

        private static int Main(string[] args)
        {
            string dir = args.Length > 0 ? args[0] : FindVectorDir();
            Console.WriteLine("测试向量目录：" + dir);

            byte[] key = FrameAuth.FromHex(File.ReadAllText(Path.Combine(dir, "session_key.hex")));
            const uint session = 0xA1B2C3D4;

            TestControl(dir, key, session);
            TestHaptic(dir, key, session);
            TestStatus(dir, key, session);
            TestAuth(dir);
            TestSeqGuard();

            Console.WriteLine($"通过 {_pass} 项，失败 {_fail} 项");
            return _fail == 0 ? 0 : 1;
        }

        private static void TestControl(string dir, byte[] key, uint session)
        {
            var f = new ControlFrame
            {
                HandleA = new HandleSample { Buttons = 0x05, Valid = true, Encoder0 = 1234, Encoder1 = -5678, Joint0 = 0.5f, Joint1 = -0.25f, Vel0 = 1.5f, Vel1 = -2.5f },
                HandleB = new HandleSample { Buttons = 0x02, Valid = true, Encoder0 = -1, Encoder1 = 2, Joint0 = 1.0f, Joint1 = 2.0f, Vel0 = 0f, Vel1 = 0.125f },
                Injector1Dir = 1,
                Injector2Dir = -1,
                Axis4Dir = -1,
            };
            byte[] mine = FrameCodec.EncodeControl(new FrameHeader { Session = session, Seq = 100, TsMs = 123456 }, f, key);
            byte[] want = ReadHex(dir, "control_frame.hex");
            Check("Control 长度 = 79", mine.Length == ProtocolConstants.ControlFrameLen);
            Check("Control 编码与 Python 向量逐字节一致", mine.SequenceEqual(want));

            bool ok = FrameCodec.TryDecodeControl(want, want.Length, session, key, out var h, out var d, out var err);
            Check("Control 解码成功", ok && err == FrameError.None);
            Check("Control 字段还原", ok && h.Seq == 100 && d.HandleA.Encoder1 == -5678 && d.HandleB.Vel1 == 0.125f && d.Injector2Dir == -1 && d.Axis4Dir == -1);

            byte[] bad = (byte[])want.Clone();
            bad[20] ^= 0x01;
            Check("Control 载荷被篡改时 MAC 校验失败",
                !FrameCodec.TryDecodeControl(bad, bad.Length, session, key, out _, out _, out var e2) && e2 == FrameError.BadMac);
            Check("Control 会话号不符被拒绝",
                !FrameCodec.TryDecodeControl(want, want.Length, session + 1, key, out _, out _, out var e3) && e3 == FrameError.BadSession);
            Check("Control 截断帧被拒绝",
                !FrameCodec.TryDecodeControl(want, want.Length - 1, session, key, out _, out _, out var e4) && e4 == FrameError.BadLength);
        }

        private static void TestHaptic(string dir, byte[] key, uint session)
        {
            var f = new HapticFrame
            {
                EchoTsMs = 123450, HoldMs = 3,
                HandleA = new HapticOut { Enable = true, Axis = 1, ForceN = 0.5f, TorqueNm = 0.01f },
                HandleB = new HapticOut { Enable = false },
            };
            byte[] mine = FrameCodec.EncodeHaptic(new FrameHeader { Session = session, Seq = 7, TsMs = 200000 }, f, key);
            byte[] want = ReadHex(dir, "haptic_frame.hex");
            Check("Haptic 长度 = 50", mine.Length == ProtocolConstants.HapticFrameLen);
            Check("Haptic 编码与 Python 向量逐字节一致", mine.SequenceEqual(want));
            bool ok = FrameCodec.TryDecodeHaptic(want, want.Length, session, key, out _, out var d, out _);
            Check("Haptic 解码字段还原", ok && d.EchoTsMs == 123450 && d.HoldMs == 3 && d.HandleA.Enable && !d.HandleB.Enable && d.HandleA.ForceN == 0.5f && d.HandleA.Axis == 1);
        }

        private static void TestStatus(string dir, byte[] key, uint session)
        {
            var f = new StatusFrame
            {
                EchoTsMs = 123450, HoldMs = 3,
                Flags = StatusFlags.ControlActive | StatusFlags.FfEnabled | StatusFlags.CalZeroed | StatusFlags.LeaseHeld | StatusFlags.AdsHealthy,
                Mode = 2, Phase = 1, SelfCheckStatus = 4, AdsState = 3,
                Cylinder1 = 2000, Cylinder2 = 10, Cylinder3 = 2000, Cylinder4 = 10,
                CylinderManualMask = 0x05,
                InjectorActive1 = 0, InjectorActive2 = 1, Axis4Active = 1,
                Force582F = 0.25f, Force582N = 0.01f, Force587F = 0.5f, Force587N = 0.02f, CleanForceN = 0.3f,
                AdsActualHz = 99.5f,
                AxisPos = new float[] { 1, 2, 3, 4, 5, 6, 7 },
                AxisFromLeft = new float[] { 11, 12, 13, 14, 15, 16, 17 },
            };
            byte[] mine = FrameCodec.EncodeStatus(new FrameHeader { Session = session, Seq = 9, TsMs = 300000 }, f, key);
            byte[] want = ReadHex(dir, "status_frame.hex");
            Check("Status 长度 = 142", mine.Length == ProtocolConstants.StatusFrameLen);
            Check("Status 编码与 Python 向量逐字节一致", mine.SequenceEqual(want));
            bool ok = FrameCodec.TryDecodeStatus(want, want.Length, session, key, out _, out var d, out _);
            Check("Status 解码字段还原", ok && d.Has(StatusFlags.LeaseHeld) && !d.Has(StatusFlags.EstopHold)
                && d.Cylinder2 == 10 && d.AxisFromLeft[6] == 17f && d.AdsActualHz == 99.5f && d.Axis4Active == 1);
        }

        private static void TestAuth(string dir)
        {
            var kv = File.ReadAllLines(Path.Combine(dir, "auth.txt"))
                .Where(l => l.Contains("=")).ToDictionary(l => l.Split('=')[0], l => l.Substring(l.IndexOf('=') + 1).Trim());
            byte[] token = System.Text.Encoding.ASCII.GetBytes(kv["token_ascii"]);
            byte[] nc = FrameAuth.FromHex(kv["nonce_client"]);
            byte[] ns = FrameAuth.FromHex(kv["nonce_server"]);
            Check("握手 auth_mac 与 Python 一致", FrameAuth.ToHex(FrameAuth.ComputeAuth(token, nc, ns)) == kv["auth_mac"]);
            Check("会话密钥派生与 Python 一致", FrameAuth.ToHex(FrameAuth.DeriveSessionKey(token, nc, ns)) == kv["session_key"]);
        }

        private static void TestSeqGuard()
        {
            var g = new SeqGuard();
            Check("序号首帧接受", g.Accept(10));
            Check("重复序号拒绝", !g.Accept(10));
            Check("乱序（更小）序号拒绝", !g.Accept(9));
            Check("递增序号接受", g.Accept(11));
            var w = new SeqGuard();
            w.Accept(uint.MaxValue - 1);
            Check("u32 回绕后序号接受", w.Accept(2));
        }

        // ------------------------------------------------------------ 辅助

        private static byte[] ReadHex(string dir, string name)
            => FrameAuth.FromHex(File.ReadAllText(Path.Combine(dir, name)));

        private static void Check(string name, bool ok)
        {
            if (ok) _pass++; else _fail++;
            Console.WriteLine((ok ? "  [通过] " : "  [失败] ") + name);
        }

        private static string FindVectorDir()
        {
            string d = AppDomain.CurrentDomain.BaseDirectory;
            for (int i = 0; i < 8 && d != null; i++)
            {
                string cand = Path.Combine(d, "protocol", "test_vectors");
                if (Directory.Exists(cand)) return cand;
                d = Path.GetDirectoryName(d);
            }
            throw new DirectoryNotFoundException("找不到 protocol/test_vectors，请先运行 tools/gen_protocol_vectors.py，或把目录作为第一个参数传入。");
        }
    }
}
