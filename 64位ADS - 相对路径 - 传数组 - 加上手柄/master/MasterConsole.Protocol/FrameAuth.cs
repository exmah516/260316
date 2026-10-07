using System;
using System.Collections.Generic;
using System.Security.Cryptography;
using System.Text;
using System.Web.Script.Serialization;

namespace MasterConsole.Protocol
{
    /// <summary>
    /// 握手认证：token 不在链路上传输，双方用 nonce 做 HMAC 挑战，并派生 UDP 会话密钥。
    /// 见 protocol/PROTOCOL.md 第 4.1 节。
    /// </summary>
    public static class FrameAuth
    {
        public static byte[] NewNonce()
        {
            byte[] n = new byte[16];
            using (var rng = RandomNumberGenerator.Create()) rng.GetBytes(n);
            return n;
        }

        /// <summary>auth = HMAC-SHA256(token, "auth" ‖ nonce_c ‖ nonce_s)，整段 32 字节。</summary>
        public static byte[] ComputeAuth(byte[] token, byte[] nonceC, byte[] nonceS)
            => Hmac(token, "auth", nonceC, nonceS);

        /// <summary>session_key = HMAC-SHA256(token, "key" ‖ nonce_c ‖ nonce_s) 的前 16 字节。</summary>
        public static byte[] DeriveSessionKey(byte[] token, byte[] nonceC, byte[] nonceS)
        {
            byte[] full = Hmac(token, "key", nonceC, nonceS);
            byte[] key = new byte[16];
            Buffer.BlockCopy(full, 0, key, 0, 16);
            return key;
        }

        public static string ToHex(byte[] data)
        {
            var sb = new StringBuilder(data.Length * 2);
            foreach (byte b in data) sb.Append(b.ToString("x2"));
            return sb.ToString();
        }

        public static byte[] FromHex(string hex)
        {
            hex = (hex ?? "").Trim();
            if (hex.Length % 2 != 0) throw new FormatException("hex 长度必须为偶数");
            byte[] data = new byte[hex.Length / 2];
            for (int i = 0; i < data.Length; i++)
                data[i] = Convert.ToByte(hex.Substring(i * 2, 2), 16);
            return data;
        }

        private static byte[] Hmac(byte[] token, string label, byte[] nonceC, byte[] nonceS)
        {
            byte[] lab = Encoding.ASCII.GetBytes(label);
            byte[] msg = new byte[lab.Length + nonceC.Length + nonceS.Length];
            Buffer.BlockCopy(lab, 0, msg, 0, lab.Length);
            Buffer.BlockCopy(nonceC, 0, msg, lab.Length, nonceC.Length);
            Buffer.BlockCopy(nonceS, 0, msg, lab.Length + nonceC.Length, nonceS.Length);
            using (var h = new HMACSHA256(token)) return h.ComputeHash(msg);
        }
    }

    /// <summary>TCP 命令通道的 JSON 消息构造（长度前缀帧由网络层处理）。</summary>
    public static class CommandMessages
    {
        private static readonly JavaScriptSerializer Json = new JavaScriptSerializer();

        public static string Hello(byte[] nonceC)
            => Json.Serialize(new Dictionary<string, object>
            {
                ["t"] = "hello", ["proto"] = (int)ProtocolConstants.Version,
                ["client"] = "MasterConsole", ["nonce_c"] = FrameAuth.ToHex(nonceC),
            });

        public static string Auth(byte[] mac)
            => Json.Serialize(new Dictionary<string, object> { ["t"] = "auth", ["mac"] = FrameAuth.ToHex(mac) });

        public static string Acquire(int id) => Simple("acquire", id);
        public static string Release(int id) => Simple("release", id);
        public static string Ping(int id) => Simple("ping", id);

        /// <summary>进入器械准备位置：导管搓捻机构位置 + Y 阀及导丝机构位置（mm）。</summary>
        public static string PreparePosition(int id, double catheterMm, double wireMm)
            => Cmd(id, "prepare_position", ("catheter_mm", catheterMm), ("wire_mm", wireMm));

        /// <summary>力反馈开关。开启前的零点采集由从端自动完成。</summary>
        public static string ForceFeedback(int id, bool enable)
            => Cmd(id, "force_feedback", ("enable", enable));

        /// <summary>电缸 1–4：engaged=true 为打开（手动覆盖），false 为恢复原状态。</summary>
        public static string Cylinder(int id, int index, bool engaged)
            => Cmd(id, "cylinder", ("index", index), ("engaged", engaged));

        /// <summary>开始控制：已到达准备位置后，在当前位置直接进入手柄控制。</summary>
        public static string StartControl(int id)
            => Cmd(id, "start_control");

        public static string RefreshHandles(int id, int successMask)
            => Cmd(id, "refresh_handles", ("success_mask", successMask));

        /// <summary>Y 阀：closed=true 为关闭，false 为打开（取消关闭）。</summary>
        public static string YValve(int id, bool closed)
            => Cmd(id, "yvalve", ("closed", closed));

        public static Dictionary<string, object> Parse(string json)
            => Json.Deserialize<Dictionary<string, object>>(json);

        private static string Simple(string type, int id)
            => Json.Serialize(new Dictionary<string, object> { ["t"] = type, ["id"] = id });

        private static string Cmd(int id, string name, params (string, object)[] args)
        {
            var d = new Dictionary<string, object> { ["t"] = "cmd", ["id"] = id, ["name"] = name };
            foreach (var (k, v) in args) d[k] = v;
            return Json.Serialize(d);
        }
    }
}
