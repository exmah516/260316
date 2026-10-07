using System;
using System.IO;
using System.Security.Cryptography;

namespace MasterConsole.Protocol
{
    /// <summary>帧解码失败原因。</summary>
    public enum FrameError
    {
        None,
        BadLength,
        BadMagic,
        BadVersion,
        BadType,
        BadSession,
        BadMac,
    }

    /// <summary>
    /// UDP 二进制帧的编解码。逐字段读写（小端），不使用 Marshal，避免对齐与 ABI 差异。
    /// 布局见 protocol/PROTOCOL.md 第 3 节。
    /// </summary>
    public static class FrameCodec
    {
        // ---------------------------------------------------------------- 编码

        public static byte[] EncodeControl(FrameHeader h, ControlFrame f, byte[] key)
        {
            h.Type = ProtocolConstants.TypeControl;
            return Seal(h, ProtocolConstants.ControlPayloadLen, w =>
            {
                WriteHandle(w, f.HandleA);
                WriteHandle(w, f.HandleB);
                w.Write(f.Injector1Dir);
                w.Write(f.Injector2Dir);
                w.Write(f.Axis4Dir);
            }, key);
        }

        public static byte[] EncodeHaptic(FrameHeader h, HapticFrame f, byte[] key)
        {
            h.Type = ProtocolConstants.TypeHaptic;
            return Seal(h, ProtocolConstants.HapticPayloadLen, w =>
            {
                w.Write(f.EchoTsMs);
                w.Write(f.HoldMs);
                WriteHaptic(w, f.HandleA);
                WriteHaptic(w, f.HandleB);
            }, key);
        }

        public static byte[] EncodeStatus(FrameHeader h, StatusFrame f, byte[] key)
        {
            h.Type = ProtocolConstants.TypeStatus;
            return Seal(h, ProtocolConstants.StatusPayloadLen, w =>
            {
                w.Write(f.EchoTsMs);
                w.Write(f.HoldMs);
                w.Write((uint)f.Flags);
                w.Write(f.Mode);
                w.Write(f.Phase);
                w.Write(f.SelfCheckStatus);
                w.Write(f.AdsState);
                w.Write(f.Cylinder1);
                w.Write(f.Cylinder2);
                w.Write(f.Cylinder3);
                w.Write(f.Cylinder4);
                w.Write(f.CylinderManualMask);
                w.Write(f.InjectorActive1);
                w.Write(f.InjectorActive2);
                w.Write(f.Axis4Active);
                w.Write(f.Force582F);
                w.Write(f.Force582N);
                w.Write(f.Force587F);
                w.Write(f.Force587N);
                w.Write(f.CleanForceN);
                w.Write(f.AdsActualHz);
                WriteFloats(w, f.AxisPos, 7);
                WriteFloats(w, f.AxisFromLeft, 7);
            }, key);
        }

        // ---------------------------------------------------------------- 解码

        /// <summary>
        /// 通用解码：按 长度 → magic → version → type → session → MAC 的顺序校验。
        /// <paramref name="expectedSession"/> 为 0 时跳过会话号检查（握手前/测试用）。
        /// 序号（防重放）由调用方用 <see cref="SeqGuard"/> 另行检查。
        /// </summary>
        public static bool TryDecode(byte[] buf, int len, byte expectedType, uint expectedSession,
                                     byte[] key, out FrameHeader header, out byte[] payload, out FrameError error)
        {
            header = default;
            payload = null;

            int expectedLen = FrameLength(expectedType);
            if (expectedLen == 0) { error = FrameError.BadType; return false; }
            if (buf == null || len != expectedLen) { error = FrameError.BadLength; return false; }

            using (var r = new BinaryReader(new MemoryStream(buf, 0, len, false)))
            {
                if (r.ReadUInt16() != ProtocolConstants.Magic) { error = FrameError.BadMagic; return false; }
                if (r.ReadByte() != ProtocolConstants.Version) { error = FrameError.BadVersion; return false; }
                header.Type = r.ReadByte();
                header.Session = r.ReadUInt32();
                header.Seq = r.ReadUInt32();
                header.TsMs = r.ReadUInt32();
            }

            if (header.Type != expectedType) { error = FrameError.BadType; return false; }
            if (expectedSession != 0 && header.Session != expectedSession) { error = FrameError.BadSession; return false; }

            int bodyLen = len - ProtocolConstants.MacLen;
            byte[] mac = ComputeMac(key, buf, bodyLen);
            if (!ConstantTimeEquals(mac, buf, bodyLen, ProtocolConstants.MacLen)) { error = FrameError.BadMac; return false; }

            payload = new byte[bodyLen - ProtocolConstants.HeaderLen];
            Buffer.BlockCopy(buf, ProtocolConstants.HeaderLen, payload, 0, payload.Length);
            error = FrameError.None;
            return true;
        }

        public static bool TryDecodeControl(byte[] buf, int len, uint session, byte[] key,
                                            out FrameHeader h, out ControlFrame f, out FrameError err)
        {
            f = default;
            if (!TryDecode(buf, len, ProtocolConstants.TypeControl, session, key, out h, out var p, out err)) return false;
            using (var r = new BinaryReader(new MemoryStream(p, false)))
            {
                f.HandleA = ReadHandle(r);
                f.HandleB = ReadHandle(r);
                f.Injector1Dir = r.ReadSByte();
                f.Injector2Dir = r.ReadSByte();
                f.Axis4Dir = r.ReadSByte();
            }
            return true;
        }

        public static bool TryDecodeHaptic(byte[] buf, int len, uint session, byte[] key,
                                           out FrameHeader h, out HapticFrame f, out FrameError err)
        {
            f = default;
            if (!TryDecode(buf, len, ProtocolConstants.TypeHaptic, session, key, out h, out var p, out err)) return false;
            using (var r = new BinaryReader(new MemoryStream(p, false)))
            {
                f.EchoTsMs = r.ReadUInt32();
                f.HoldMs = r.ReadUInt16();
                f.HandleA = ReadHaptic(r);
                f.HandleB = ReadHaptic(r);
            }
            return true;
        }

        public static bool TryDecodeStatus(byte[] buf, int len, uint session, byte[] key,
                                           out FrameHeader h, out StatusFrame f, out FrameError err)
        {
            f = default;
            if (!TryDecode(buf, len, ProtocolConstants.TypeStatus, session, key, out h, out var p, out err)) return false;
            using (var r = new BinaryReader(new MemoryStream(p, false)))
            {
                f.EchoTsMs = r.ReadUInt32();
                f.HoldMs = r.ReadUInt16();
                f.Flags = (StatusFlags)r.ReadUInt32();
                f.Mode = r.ReadInt32();
                f.Phase = r.ReadInt32();
                f.SelfCheckStatus = r.ReadInt32();
                f.AdsState = r.ReadInt32();
                f.Cylinder1 = r.ReadUInt16();
                f.Cylinder2 = r.ReadUInt16();
                f.Cylinder3 = r.ReadUInt16();
                f.Cylinder4 = r.ReadUInt16();
                f.CylinderManualMask = r.ReadByte();
                f.InjectorActive1 = r.ReadSByte();
                f.InjectorActive2 = r.ReadSByte();
                f.Axis4Active = r.ReadSByte();
                f.Force582F = r.ReadSingle();
                f.Force582N = r.ReadSingle();
                f.Force587F = r.ReadSingle();
                f.Force587N = r.ReadSingle();
                f.CleanForceN = r.ReadSingle();
                f.AdsActualHz = r.ReadSingle();
                f.AxisPos = ReadFloats(r, 7);
                f.AxisFromLeft = ReadFloats(r, 7);
            }
            return true;
        }

        public static int FrameLength(byte type)
        {
            switch (type)
            {
                case ProtocolConstants.TypeControl: return ProtocolConstants.ControlFrameLen;
                case ProtocolConstants.TypeHaptic: return ProtocolConstants.HapticFrameLen;
                case ProtocolConstants.TypeStatus: return ProtocolConstants.StatusFrameLen;
                default: return 0;
            }
        }

        // ---------------------------------------------------------------- 内部实现

        private static byte[] Seal(FrameHeader h, int payloadLen, Action<BinaryWriter> writePayload, byte[] key)
        {
            int bodyLen = ProtocolConstants.HeaderLen + payloadLen;
            var ms = new MemoryStream(bodyLen + ProtocolConstants.MacLen);
            var w = new BinaryWriter(ms);
            w.Write(ProtocolConstants.Magic);
            w.Write(ProtocolConstants.Version);
            w.Write(h.Type);
            w.Write(h.Session);
            w.Write(h.Seq);
            w.Write(h.TsMs);
            writePayload(w);
            w.Flush();
            if (ms.Length != bodyLen)
                throw new InvalidOperationException($"帧体长度错误：期望 {bodyLen}，实际 {ms.Length}");

            byte[] buf = new byte[bodyLen + ProtocolConstants.MacLen];
            Buffer.BlockCopy(ms.GetBuffer(), 0, buf, 0, bodyLen);
            byte[] mac = ComputeMac(key, buf, bodyLen);
            Buffer.BlockCopy(mac, 0, buf, bodyLen, ProtocolConstants.MacLen);
            return buf;
        }

        private static byte[] ComputeMac(byte[] key, byte[] data, int count)
        {
            using (var h = new HMACSHA256(key))
            {
                byte[] full = h.ComputeHash(data, 0, count);
                byte[] mac = new byte[ProtocolConstants.MacLen];
                Buffer.BlockCopy(full, 0, mac, 0, ProtocolConstants.MacLen);
                return mac;
            }
        }

        private static bool ConstantTimeEquals(byte[] expected, byte[] buf, int offset, int count)
        {
            int diff = 0;
            for (int i = 0; i < count; i++) diff |= expected[i] ^ buf[offset + i];
            return diff == 0;
        }

        private static void WriteHandle(BinaryWriter w, HandleSample s)
        {
            w.Write(s.Buttons);
            w.Write((byte)(s.Valid ? 1 : 0));
            w.Write(s.Encoder0);
            w.Write(s.Encoder1);
            w.Write(s.Joint0);
            w.Write(s.Joint1);
            w.Write(s.Vel0);
            w.Write(s.Vel1);
        }

        private static HandleSample ReadHandle(BinaryReader r)
        {
            var s = new HandleSample();
            s.Buttons = r.ReadByte();
            s.Valid = r.ReadByte() != 0;
            s.Encoder0 = r.ReadInt32();
            s.Encoder1 = r.ReadInt32();
            s.Joint0 = r.ReadSingle();
            s.Joint1 = r.ReadSingle();
            s.Vel0 = r.ReadSingle();
            s.Vel1 = r.ReadSingle();
            return s;
        }

        private static void WriteHaptic(BinaryWriter w, HapticOut o)
        {
            w.Write((byte)(o.Enable ? 1 : 0));
            w.Write(o.Axis);
            w.Write(o.ForceN);
            w.Write(o.TorqueNm);
        }

        private static HapticOut ReadHaptic(BinaryReader r)
        {
            var o = new HapticOut();
            o.Enable = r.ReadByte() != 0;
            o.Axis = r.ReadSByte();
            o.ForceN = r.ReadSingle();
            o.TorqueNm = r.ReadSingle();
            return o;
        }

        private static void WriteFloats(BinaryWriter w, float[] v, int count)
        {
            for (int i = 0; i < count; i++) w.Write(v != null && i < v.Length ? v[i] : 0f);
        }

        private static float[] ReadFloats(BinaryReader r, int count)
        {
            var v = new float[count];
            for (int i = 0; i < count; i++) v[i] = r.ReadSingle();
            return v;
        }
    }

    /// <summary>
    /// 序号防重放：只接受严格递增（u32 回绕安全）的序号。
    /// 新会话需重新创建实例。
    /// </summary>
    public sealed class SeqGuard
    {
        private uint _last;
        private bool _hasLast;
        public ulong Dropped { get; private set; }

        public bool Accept(uint seq)
        {
            if (!_hasLast || (int)(seq - _last) > 0)
            {
                _last = seq;
                _hasLast = true;
                return true;
            }
            Dropped++;
            return false;
        }
    }
}
