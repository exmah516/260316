// 文件职责说明：见 remote_gateway.h。
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <bcrypt.h>

#include "remote_gateway.h"
#include "remote_protocol.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>

#pragma comment(lib, "Ws2_32.lib")
#pragma comment(lib, "bcrypt.lib")

namespace rp = remote_protocol;

namespace
{
	constexpr ULONGLONG kControlFrameWarnMs = 200;    // 控制帧超时：点动归零
	constexpr ULONGLONG kControlFrameReleaseMs = 2000; // 控制帧持续超时：释放控制权
	constexpr ULONGLONG kHoldRenewMs = 100;           // 点动租约续期周期（租约 500 ms）
	constexpr ULONGLONG kStateFreshMs = 1000;         // 状态快照有效期
	constexpr ULONGLONG kTcpIdleMs = 20000;           // TCP 空闲断开
	constexpr size_t kMaxJsonBytes = 65536;
	constexpr size_t kMaxQueuedCommands = 64;
	constexpr size_t kMaxPending = 8;

	// ------------------------------------------------------------------ 十六进制
	std::string hex_encode(const unsigned char* d, size_t n)
	{
		static const char* digits = "0123456789abcdef";
		std::string s;
		s.reserve(n * 2);
		for (size_t i = 0; i < n; ++i)
		{
			s.push_back(digits[d[i] >> 4]);
			s.push_back(digits[d[i] & 0x0F]);
		}
		return s;
	}

	int hex_val(char c)
	{
		if (c >= '0' && c <= '9') return c - '0';
		if (c >= 'a' && c <= 'f') return c - 'a' + 10;
		if (c >= 'A' && c <= 'F') return c - 'A' + 10;
		return -1;
	}

	bool hex_decode(const std::string& s, unsigned char* out, size_t expected_len)
	{
		if (s.size() != expected_len * 2) return false;
		for (size_t i = 0; i < expected_len; ++i)
		{
			const int hi = hex_val(s[i * 2]);
			const int lo = hex_val(s[i * 2 + 1]);
			if (hi < 0 || lo < 0) return false;
			out[i] = static_cast<unsigned char>((hi << 4) | lo);
		}
		return true;
	}

	bool ct_equal(const unsigned char* a, const unsigned char* b, size_t n)
	{
		unsigned char diff = 0;
		for (size_t i = 0; i < n; ++i) diff |= static_cast<unsigned char>(a[i] ^ b[i]);
		return diff == 0;
	}

	// ------------------------------------------------------------------ 扁平 JSON
	std::string json_escape(const std::string& s)
	{
		std::string o;
		o.reserve(s.size() + 8);
		for (unsigned char c : s)
		{
			switch (c)
			{
			case '"': o += "\\\""; break;
			case '\\': o += "\\\\"; break;
			case '\n': o += "\\n"; break;
			case '\r': o += "\\r"; break;
			case '\t': o += "\\t"; break;
			default:
				if (c < 0x20)
				{
					char buf[8];
					std::snprintf(buf, sizeof(buf), "\\u%04x", c);
					o += buf;
				}
				else
				{
					o.push_back(static_cast<char>(c));
				}
			}
		}
		return o;
	}

	void skip_ws(const std::string& s, size_t& i)
	{
		while (i < s.size() && (s[i] == ' ' || s[i] == '\t' || s[i] == '\r' || s[i] == '\n')) ++i;
	}

	void append_utf8(std::string& out, unsigned code)
	{
		if (code < 0x80) out.push_back(static_cast<char>(code));
		else if (code < 0x800)
		{
			out.push_back(static_cast<char>(0xC0 | (code >> 6)));
			out.push_back(static_cast<char>(0x80 | (code & 0x3F)));
		}
		else
		{
			out.push_back(static_cast<char>(0xE0 | (code >> 12)));
			out.push_back(static_cast<char>(0x80 | ((code >> 6) & 0x3F)));
			out.push_back(static_cast<char>(0x80 | (code & 0x3F)));
		}
	}

	bool parse_string(const std::string& s, size_t& i, std::string& out)
	{
		if (i >= s.size() || s[i] != '"') return false;
		++i;
		out.clear();
		while (i < s.size())
		{
			const char c = s[i++];
			if (c == '"') return true;
			if (c != '\\')
			{
				out.push_back(c);
				continue;
			}
			if (i >= s.size()) return false;
			const char e = s[i++];
			switch (e)
			{
			case '"': out.push_back('"'); break;
			case '\\': out.push_back('\\'); break;
			case '/': out.push_back('/'); break;
			case 'b': out.push_back('\b'); break;
			case 'f': out.push_back('\f'); break;
			case 'n': out.push_back('\n'); break;
			case 'r': out.push_back('\r'); break;
			case 't': out.push_back('\t'); break;
			case 'u':
			{
				if (i + 4 > s.size()) return false;
				unsigned code = 0;
				for (int k = 0; k < 4; ++k)
				{
					const int v = hex_val(s[i++]);
					if (v < 0) return false;
					code = (code << 4) | static_cast<unsigned>(v);
				}
				if (code >= 0xD800 && code <= 0xDFFF) code = '?';
				append_utf8(out, code);
				break;
			}
			default: return false;
			}
		}
		return false;
	}

	bool parse_value(const std::string& s, size_t& i, RemoteJsonValue& v)
	{
		skip_ws(s, i);
		if (i >= s.size()) return false;
		const char c = s[i];
		if (c == '"')
		{
			v.type = RemoteJsonValue::String;
			return parse_string(s, i, v.s);
		}
		if (s.compare(i, 4, "true") == 0)
		{
			v.type = RemoteJsonValue::Bool; v.b = true; i += 4; return true;
		}
		if (s.compare(i, 5, "false") == 0)
		{
			v.type = RemoteJsonValue::Bool; v.b = false; i += 5; return true;
		}
		if (s.compare(i, 4, "null") == 0)
		{
			v.type = RemoteJsonValue::Null; i += 4; return true;
		}
		if (c == '{' || c == '[') return false; // 协议只使用扁平对象
		const char* begin = s.c_str() + i;
		char* end = nullptr;
		const double d = std::strtod(begin, &end);
		if (end == begin) return false;
		v.type = RemoteJsonValue::Number;
		v.n = d;
		i += static_cast<size_t>(end - begin);
		return true;
	}

	bool parse_flat_json(const std::string& s, RemoteJsonObject& out)
	{
		out.clear();
		size_t i = 0;
		skip_ws(s, i);
		if (i >= s.size() || s[i] != '{') return false;
		++i;
		skip_ws(s, i);
		if (i < s.size() && s[i] == '}') return true;
		while (i < s.size())
		{
			skip_ws(s, i);
			std::string key;
			if (!parse_string(s, i, key)) return false;
			skip_ws(s, i);
			if (i >= s.size() || s[i] != ':') return false;
			++i;
			RemoteJsonValue v;
			if (!parse_value(s, i, v)) return false;
			out[key] = v;
			skip_ws(s, i);
			if (i >= s.size()) return false;
			if (s[i] == ',') { ++i; continue; }
			if (s[i] == '}') return true;
			return false;
		}
		return false;
	}

	bool get_num(const RemoteJsonObject& o, const char* k, double& v)
	{
		auto it = o.find(k);
		if (it == o.end() || it->second.type != RemoteJsonValue::Number) return false;
		v = it->second.n;
		return std::isfinite(v);
	}

	bool get_bool(const RemoteJsonObject& o, const char* k, bool& v)
	{
		auto it = o.find(k);
		if (it == o.end() || it->second.type != RemoteJsonValue::Bool) return false;
		v = it->second.b;
		return true;
	}

	bool get_str(const RemoteJsonObject& o, const char* k, std::string& v)
	{
		auto it = o.find(k);
		if (it == o.end() || it->second.type != RemoteJsonValue::String) return false;
		v = it->second.s;
		return true;
	}

	// ------------------------------------------------------------------ 套接字
	bool send_all(SOCKET s, const char* data, int total)
	{
		int sent = 0;
		while (sent < total)
		{
			const int n = send(s, data + sent, total - sent, 0);
			if (n == SOCKET_ERROR || n == 0) return false;
			sent += n;
		}
		return true;
	}

	bool send_json(SOCKET s, const std::string& json)
	{
		std::string frame;
		const std::uint32_t len = static_cast<std::uint32_t>(json.size());
		frame.resize(4 + json.size());
		std::memcpy(&frame[0], &len, 4); // 小端
		std::memcpy(&frame[4], json.data(), json.size());
		return send_all(s, frame.data(), static_cast<int>(frame.size()));
	}

	bool send_ack(SOCKET s, int id, const char* state, const std::string& reason = std::string())
	{
		std::ostringstream o;
		o << "{\"t\":\"ack\",\"id\":" << id << ",\"state\":\"" << state << "\"";
		if (!reason.empty()) o << ",\"reason\":\"" << json_escape(reason) << "\"";
		o << "}";
		return send_json(s, o.str());
	}

	float fin(double v)
	{
		return std::isfinite(v) ? static_cast<float>(v) : 0.0f;
	}

	int clamp_dir(int v)
	{
		return v < -1 ? -1 : (v > 1 ? 1 : v);
	}
}

// ====================================================================== 生命周期

RemoteGateway::~RemoteGateway()
{
	stop();
}

bool RemoteGateway::start(const RemoteGatewayConfig& cfg)
{
	if (running_.load()) return true;
	cfg_ = cfg;

	// 密钥文件：不存在则不启用，避免无认证的网络入口。
	{
		std::ifstream f(cfg_.token_path, std::ios::binary);
		if (!f)
		{
			std::cout << "远程网关：未启用（找不到密钥文件 " << cfg_.token_path << "）。" << std::endl;
			return false;
		}
		std::string raw((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
		size_t b = 0, e = raw.size();
		while (b < e && std::isspace(static_cast<unsigned char>(raw[b]))) ++b;
		while (e > b && std::isspace(static_cast<unsigned char>(raw[e - 1]))) --e;
		if (e - b < 16)
		{
			std::cout << "远程网关：未启用（密钥至少 16 个字符）。" << std::endl;
			return false;
		}
		token_.assign(raw.begin() + b, raw.begin() + e);
	}

	WSADATA wsa{};
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
	{
		std::cout << "远程网关：WSAStartup 失败。" << std::endl;
		return false;
	}
	wsa_started_ = true;

	BCRYPT_ALG_HANDLE alg = nullptr;
	if (!BCRYPT_SUCCESS(BCryptOpenAlgorithmProvider(&alg, BCRYPT_SHA256_ALGORITHM, nullptr, BCRYPT_ALG_HANDLE_HMAC_FLAG)))
	{
		std::cout << "远程网关：无法初始化 HMAC-SHA256。" << std::endl;
		WSACleanup();
		wsa_started_ = false;
		return false;
	}
	bcrypt_alg_ = alg;

	stop_.store(false);
	try
	{
		tcp_thread_ = std::thread(&RemoteGateway::tcp_loop, this);
		udp_thread_ = std::thread(&RemoteGateway::udp_loop, this);
	}
	catch (...)
	{
		stop_.store(true);
		if (tcp_thread_.joinable()) tcp_thread_.join();
		BCryptCloseAlgorithmProvider(static_cast<BCRYPT_ALG_HANDLE>(bcrypt_alg_), 0);
		bcrypt_alg_ = nullptr;
		WSACleanup();
		wsa_started_ = false;
		return false;
	}
	running_.store(true);
	std::cout << "远程网关：已启动（TCP " << cfg_.tcp_port << " / UDP " << cfg_.udp_port << "）。" << std::endl;
	return true;
}

void RemoteGateway::stop()
{
	if (!running_.load() && !tcp_thread_.joinable() && !udp_thread_.joinable()) return;
	stop_.store(true);
	if (tcp_thread_.joinable()) tcp_thread_.join();
	if (udp_thread_.joinable()) udp_thread_.join();
	{
		std::lock_guard<std::mutex> lock(m_);
		if (session_active_) end_session_locked("网关停止");
	}
	if (bcrypt_alg_)
	{
		BCryptCloseAlgorithmProvider(static_cast<BCRYPT_ALG_HANDLE>(bcrypt_alg_), 0);
		bcrypt_alg_ = nullptr;
	}
	if (wsa_started_)
	{
		WSACleanup();
		wsa_started_ = false;
	}
	running_.store(false);
}

void RemoteGateway::publish_state(const VisState& state, const RemoteExtraState& extra)
{
	std::lock_guard<std::mutex> lock(m_);
	state_ = state;
	extra_ = extra;
	have_state_ = true;
	++state_seq_;
	state_tick_ms_ = GetTickCount64();
}

bool RemoteGateway::poll_command(VisCommand& cmd)
{
	std::lock_guard<std::mutex> lock(cmd_m_);
	if (cmd_q_.empty()) return false;
	cmd = std::move(cmd_q_.front());
	cmd_q_.pop_front();
	return true;
}

// ====================================================================== 内部工具

RemoteGateway::StateCopy RemoteGateway::copy_state()
{
	StateCopy c;
	std::lock_guard<std::mutex> lock(m_);
	c.valid = have_state_ && (GetTickCount64() - state_tick_ms_) < kStateFreshMs;
	c.lease = lease_held_;
	c.vs = state_;
	c.extra = extra_;
	c.seq = state_seq_;
	return c;
}

void RemoteGateway::push_cmd(VisCommandType type, int p1, int p2)
{
	std::lock_guard<std::mutex> lock(cmd_m_);
	if (cmd_q_.size() >= kMaxQueuedCommands)
	{
		std::cout << "远程网关：命令队列已满，丢弃一条命令。" << std::endl;
		return;
	}
	VisCommand c;
	c.type = type;
	c.param1 = p1;
	c.param2 = p2;
	cmd_q_.push_back(std::move(c));
}

void RemoteGateway::add_event(const char* level, const std::string& text)
{
	std::cout << "远程网关：" << text << std::endl;
	std::lock_guard<std::mutex> lock(ev_m_);
	if (events_.size() >= 32) events_.pop_front();
	std::ostringstream o;
	o << "{\"t\":\"event\",\"level\":\"" << level << "\",\"text\":\"" << json_escape(text) << "\"}";
	events_.push_back(o.str());
}

void RemoteGateway::apply_holds_locked(const int dirs[3], ULONGLONG now)
{
	for (int i = 0; i < 3; ++i)
	{
		const int d = dirs[i];
		const bool changed = d != hold_dir_[i];
		const bool renew = !changed && d != 0 && (now - hold_last_push_ms_[i]) >= kHoldRenewMs;
		if (!changed && !renew) continue;
		if (i < 2) push_cmd(VisCommandType::SetInjectorManualJog, i + 1, d);
		else push_cmd(VisCommandType::SetAxis4ManualJog, d, 0);
		hold_dir_[i] = d;
		hold_last_push_ms_[i] = now;
	}
}

void RemoteGateway::release_lease_locked(const char* reason)
{
	const int zeros[3] = { 0, 0, 0 };
	apply_holds_locked(zeros, GetTickCount64());
	if (lease_held_) std::cout << "远程网关：已释放控制权（" << reason << "）。" << std::endl;
	lease_held_ = false;
}

void RemoteGateway::end_session_locked(const char* reason)
{
	release_lease_locked(reason);
	session_active_ = false;
	peer_valid_ = false;
	seq_has_ = false;
	std::cout << "远程网关：会话结束（" << reason << "）。" << std::endl;
}

bool RemoteGateway::hmac_sha256(const unsigned char* key, size_t key_len,
	const unsigned char* data, size_t data_len, unsigned char out[32])
{
	if (!bcrypt_alg_) return false;
	BCRYPT_HASH_HANDLE h = nullptr;
	NTSTATUS st = BCryptCreateHash(static_cast<BCRYPT_ALG_HANDLE>(bcrypt_alg_), &h, nullptr, 0,
		const_cast<PUCHAR>(key), static_cast<ULONG>(key_len), 0);
	if (!BCRYPT_SUCCESS(st)) return false;
	st = BCryptHashData(h, const_cast<PUCHAR>(data), static_cast<ULONG>(data_len), 0);
	if (BCRYPT_SUCCESS(st)) st = BCryptFinishHash(h, out, 32, 0);
	BCryptDestroyHash(h);
	return BCRYPT_SUCCESS(st);
}

bool RemoteGateway::random_bytes(unsigned char* out, size_t len)
{
	return BCRYPT_SUCCESS(BCryptGenRandom(nullptr, out, static_cast<ULONG>(len), BCRYPT_USE_SYSTEM_PREFERRED_RNG));
}

// ====================================================================== TCP 命令通道

void RemoteGateway::tcp_loop()
{
	SOCKET ls = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (ls == INVALID_SOCKET)
	{
		std::cout << "远程网关：TCP 套接字创建失败。" << std::endl;
		return;
	}
	sockaddr_in addr{};
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_ANY);
	addr.sin_port = htons(cfg_.tcp_port);
	if (bind(ls, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == SOCKET_ERROR ||
		listen(ls, 2) == SOCKET_ERROR)
	{
		std::cout << "远程网关：TCP 端口 " << cfg_.tcp_port << " 绑定/监听失败。" << std::endl;
		closesocket(ls);
		return;
	}
	while (!stop_.load())
	{
		fd_set rs;
		FD_ZERO(&rs);
		FD_SET(ls, &rs);
		timeval tv{ 0, 200000 };
		const int r = select(0, &rs, nullptr, nullptr, &tv);
		if (r <= 0) continue;
		SOCKET c = accept(ls, nullptr, nullptr);
		if (c == INVALID_SOCKET) continue;
		BOOL nodelay = TRUE;
		setsockopt(c, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&nodelay), sizeof(nodelay));
		handle_client(static_cast<std::uintptr_t>(c));
		closesocket(c);
	}
	closesocket(ls);
}

void RemoteGateway::handle_client(std::uintptr_t sock_handle)
{
	const SOCKET sock = static_cast<SOCKET>(sock_handle);
	std::cout << "远程网关：主端 TCP 已连接。" << std::endl;

	AuthState auth = AuthState::Hello;
	unsigned char nonce_c[16] = {};
	unsigned char nonce_s[16] = {};
	std::vector<Pending> pending;
	std::vector<unsigned char> buf;
	ULONGLONG last_rx = GetTickCount64();
	bool authed_here = false;
	bool alive = true;

	while (alive && !stop_.load())
	{
		if (auth == AuthState::Authed)
		{
			flush_events(sock);
			process_pending(sock, pending);
			{
				std::lock_guard<std::mutex> lock(m_);
				if (!session_active_) break; // 已被网关结束
			}
		}

		fd_set rs;
		FD_ZERO(&rs);
		FD_SET(sock, &rs);
		timeval tv{ 0, 50000 };
		const int r = select(0, &rs, nullptr, nullptr, &tv);
		if (r < 0) break;
		if (r > 0)
		{
			unsigned char tmp[2048];
			const int n = recv(sock, reinterpret_cast<char*>(tmp), sizeof(tmp), 0);
			if (n <= 0) break;
			buf.insert(buf.end(), tmp, tmp + n);
			last_rx = GetTickCount64();
			while (alive && buf.size() >= 4)
			{
				std::uint32_t len = 0;
				std::memcpy(&len, buf.data(), 4);
				if (len == 0 || len > kMaxJsonBytes) { alive = false; break; }
				if (buf.size() < 4 + static_cast<size_t>(len)) break;
				const std::string text(buf.begin() + 4, buf.begin() + 4 + len);
				buf.erase(buf.begin(), buf.begin() + 4 + len);
				if (!handle_message(sock_handle, text, auth, nonce_c, nonce_s, pending)) alive = false;
				if (auth == AuthState::Authed) authed_here = true;
			}
		}
		if (GetTickCount64() - last_rx > kTcpIdleMs)
		{
			std::cout << "远程网关：TCP 空闲超时，断开。" << std::endl;
			break;
		}
	}

	ff_zeroing_.store(false);
	{
		std::lock_guard<std::mutex> lock(m_);
		if (authed_here && session_active_) end_session_locked("TCP 连接断开");
	}
	std::cout << "远程网关：主端 TCP 已断开。" << std::endl;
}

void RemoteGateway::flush_events(std::uintptr_t sock_handle)
{
	const SOCKET sock = static_cast<SOCKET>(sock_handle);
	for (;;)
	{
		std::string ev;
		{
			std::lock_guard<std::mutex> lock(ev_m_);
			if (events_.empty()) return;
			ev = std::move(events_.front());
			events_.pop_front();
		}
		if (!send_json(sock, ev)) return;
	}
}

bool RemoteGateway::handle_message(std::uintptr_t sock_handle, const std::string& text, AuthState& auth,
	unsigned char nonce_c[16], unsigned char nonce_s[16], std::vector<Pending>& pending)
{
	const SOCKET sock = static_cast<SOCKET>(sock_handle);
	RemoteJsonObject o;
	if (!parse_flat_json(text, o)) return false;
	std::string type;
	if (!get_str(o, "t", type)) return false;

	double id_num = 0;
	get_num(o, "id", id_num);
	const int id = static_cast<int>(id_num);

	if (type == "hello")
	{
		double proto = 0;
		std::string nc;
		if (auth != AuthState::Hello || !get_num(o, "proto", proto) || static_cast<int>(proto) != rp::kVersion ||
			!get_str(o, "nonce_c", nc) || !hex_decode(nc, nonce_c, 16))
		{
			send_json(sock, "{\"t\":\"error\",\"reason\":\"握手参数无效\"}");
			return false;
		}
		if (!random_bytes(nonce_s, 16)) return false;
		auth = AuthState::Challenge;
		return send_json(sock, "{\"t\":\"challenge\",\"nonce_s\":\"" + hex_encode(nonce_s, 16) + "\"}");
	}

	if (type == "auth")
	{
		std::string mac_hex;
		unsigned char mac[32] = {};
		if (auth != AuthState::Challenge || !get_str(o, "mac", mac_hex) || !hex_decode(mac_hex, mac, 32))
		{
			send_json(sock, "{\"t\":\"error\",\"reason\":\"认证参数无效\"}");
			return false;
		}
		unsigned char msg[4 + 32] = { 'a', 'u', 't', 'h' };
		std::memcpy(msg + 4, nonce_c, 16);
		std::memcpy(msg + 20, nonce_s, 16);
		unsigned char expect[32] = {};
		if (!hmac_sha256(token_.data(), token_.size(), msg, sizeof(msg), expect) || !ct_equal(expect, mac, 32))
		{
			std::cout << "远程网关：认证失败。" << std::endl;
			Sleep(300); // 简单限速
			send_json(sock, "{\"t\":\"error\",\"reason\":\"认证失败\"}");
			return false;
		}
		unsigned char kmsg[3 + 32] = { 'k', 'e', 'y' };
		std::memcpy(kmsg + 3, nonce_c, 16);
		std::memcpy(kmsg + 19, nonce_s, 16);
		unsigned char full[32] = {};
		if (!hmac_sha256(token_.data(), token_.size(), kmsg, 3 + 32, full)) return false;
		std::uint32_t sid = 0;
		do { if (!random_bytes(reinterpret_cast<unsigned char*>(&sid), 4)) return false; } while (sid == 0);
		{
			std::lock_guard<std::mutex> lock(m_);
			if (session_active_) end_session_locked("被新会话替换");
			session_active_ = true;
			session_id_ = sid;
			std::memcpy(session_key_, full, 16);
			lease_held_ = false;
			seq_has_ = false;
			peer_valid_ = false;
			tx_seq_ = 0;
			frames_ok_ = frames_dropped_ = 0;
			for (int i = 0; i < 3; ++i) { hold_dir_[i] = 0; hold_last_push_ms_[i] = 0; }
		}
		auth = AuthState::Authed;
		std::cout << "远程网关：认证通过，会话 0x" << std::hex << sid << std::dec << "。" << std::endl;
		std::ostringstream ack;
		ack << "{\"t\":\"hello_ack\",\"session\":" << sid << ",\"udp_port\":" << cfg_.udp_port << "}";
		return send_json(sock, ack.str());
	}

	if (auth != AuthState::Authed) return false;

	if (type == "ping")
	{
		std::ostringstream p;
		p << "{\"t\":\"pong\",\"id\":" << id << "}";
		return send_json(sock, p.str());
	}
	if (type == "acquire")
	{
		{
			std::lock_guard<std::mutex> lock(m_);
			lease_held_ = true;
			last_frame_ms_ = GetTickCount64(); // 给控制帧 2 s 的起步宽限
		}
		std::cout << "远程网关：主端获得控制权。" << std::endl;
		return send_ack(sock, id, "done");
	}
	if (type == "release")
	{
		{
			std::lock_guard<std::mutex> lock(m_);
			release_lease_locked("主端主动释放");
		}
		pending.clear();
		ff_zeroing_.store(false);
		return send_ack(sock, id, "done");
	}
	if (type == "cmd")
	{
		std::string name;
		if (!get_str(o, "name", name)) return send_ack(sock, id, "rejected", "缺少命令名");
		handle_command(sock_handle, id, name, o, pending);
		return true;
	}
	return send_ack(sock, id, "rejected", "未知消息类型");
}

void RemoteGateway::handle_command(std::uintptr_t sock_handle, int id, const std::string& name,
	const RemoteJsonObject& f, std::vector<Pending>& pending)
{
	const SOCKET sock = static_cast<SOCKET>(sock_handle);
	const StateCopy st = copy_state();
	if (!st.lease)
	{
		send_ack(sock, id, "rejected", "未持有控制权");
		return;
	}
	if (!st.valid)
	{
		send_ack(sock, id, "rejected", "从端状态未更新，请稍后重试");
		return;
	}
	if (pending.size() >= kMaxPending)
	{
		send_ack(sock, id, "rejected", "尚有命令未完成");
		return;
	}
	const ULONGLONG now = GetTickCount64();
	Pending p;
	p.id = id;
	p.state_seq0 = st.seq;

	if (name == "prepare_position")
	{
		double c = 0, w = 0;
		if (!get_num(f, "catheter_mm", c) || !get_num(f, "wire_mm", w))
		{
			send_ack(sock, id, "rejected", "参数缺失");
			return;
		}
		if (c < 5.0 || c > 95.0) { send_ack(sock, id, "rejected", "导管搓捻机构位置范围为 5–95 mm"); return; }
		if (w < 10.0 || w > 639.0) { send_ack(sock, id, "rejected", "Y阀及导丝机构位置范围为 10–639 mm"); return; }
		if (c > w) { send_ack(sock, id, "rejected", "导管搓捻机构位置不得大于 Y阀及导丝机构位置"); return; }
		if (st.vs.ads_state != 2) { send_ack(sock, id, "rejected", "从端 ADS 通信未就绪"); return; }
		if (st.vs.self_check_done) { send_ack(sock, id, "rejected", "已处于器械准备位置，无需重复进入"); return; }
		if (st.vs.selfcheck_status != 1 && st.vs.selfcheck_status != 3)
		{
			send_ack(sock, id, "rejected", "PLC 当前不接受自检请求（运行中或尚未初始化）");
			return;
		}
		const int p1 = static_cast<int>(std::llround(c * 100.0));
		const int p3 = static_cast<int>(std::llround(w * 100.0));
		const int p5 = p3 + 500;                                   // 轴5 = 轴3 + 5 mm
		const int p6 = p5 + static_cast<int>(std::llround(cfg_.startup_g_mm * 100.0)); // 轴6 = 轴5 + g
		push_cmd(VisCommandType::SetSelfCheckAxisPos, 1, p1);
		push_cmd(VisCommandType::SetSelfCheckAxisPos, 3, p3);
		push_cmd(VisCommandType::SetSelfCheckAxisPos, 5, p5);
		push_cmd(VisCommandType::SetSelfCheckAxisPos, 6, p6);
		push_cmd(VisCommandType::StartSelfCheck);
		p.kind = PendingKind::Prepare;
		p.deadline_ms = now + 300000;
		pending.push_back(p);
		send_ack(sock, id, "accepted");
		return;
	}

	if (name == "force_feedback")
	{
		bool enable = false;
		if (!get_bool(f, "enable", enable)) { send_ack(sock, id, "rejected", "参数缺失"); return; }
		for (size_t i = 0; i < pending.size(); ++i)
		{
			if (pending[i].kind == PendingKind::FfZero || pending[i].kind == PendingKind::FfEnable ||
				pending[i].kind == PendingKind::FfDisable)
			{
				send_ack(sock, id, "rejected", "力反馈切换尚未完成");
				return;
			}
		}
		if (enable)
		{
			if (st.vs.ff_enabled) { send_ack(sock, id, "done"); return; }
			p.kind = PendingKind::FfZero; // 开启前先自动零点采集
			p.zero_ok0 = st.extra.force_zero_ok_count;
			p.zero_fail0 = st.extra.force_zero_fail_count;
			p.deadline_ms = now + 5000;
			push_cmd(VisCommandType::ZeroForceSensor);
			ff_zeroing_.store(true);
		}
		else
		{
			if (!st.vs.ff_enabled) { send_ack(sock, id, "done"); return; }
			p.kind = PendingKind::FfDisable;
			p.deadline_ms = now + 2000;
			push_cmd(VisCommandType::ToggleForceFeedback);
		}
		pending.push_back(p);
		send_ack(sock, id, "accepted");
		return;
	}

	if (name == "cylinder")
	{
		double idx_num = 0;
		bool engaged = false;
		if (!get_num(f, "index", idx_num) || !get_bool(f, "engaged", engaged))
		{
			send_ack(sock, id, "rejected", "参数缺失");
			return;
		}
		const int index = static_cast<int>(idx_num) - 1;
		if (index < 0 || index > 3) { send_ack(sock, id, "rejected", "电缸编号必须为 1–4"); return; }
		const bool now_engaged = (st.vs.cylinder_manual_mask & (1u << index)) != 0;
		p.index = index;
		p.deadline_ms = now + 1500;
		if (engaged)
		{
			if (now_engaged) { send_ack(sock, id, "done"); return; }
			if (!st.vs.cylinder_manual_allowed)
			{
				send_ack(sock, id, "rejected", "当前未就绪或自动流程正在接管，电缸不可手动操作");
				return;
			}
			// 电缸 1/3 打开值 2000，电缸 2/4 打开值 10。
			const int open_value = (index == 0 || index == 2) ? 2000 : 10;
			push_cmd(VisCommandType::SetCylinderManualPosition, index, open_value);
			p.kind = PendingKind::CylinderOn;
		}
		else
		{
			if (!now_engaged) { send_ack(sock, id, "done"); return; }
			push_cmd(VisCommandType::ResetCylinderManual, index);
			p.kind = PendingKind::CylinderOff;
		}
		pending.push_back(p);
		send_ack(sock, id, "accepted");
		return;
	}

	if (name == "yvalve")
	{
		bool closed = false;
		if (!get_bool(f, "closed", closed)) { send_ack(sock, id, "rejected", "参数缺失"); return; }
		if (st.extra.y_valve_open == !closed) { send_ack(sock, id, "done"); return; }
		push_cmd(VisCommandType::SetYValveOpen, closed ? 0 : 1);
		p.kind = PendingKind::YValve;
		p.flag = closed;
		p.deadline_ms = now + 1500;
		pending.push_back(p);
		send_ack(sock, id, "accepted");
		return;
	}

	send_ack(sock, id, "rejected", "未知命令：" + name);
}

void RemoteGateway::process_pending(std::uintptr_t sock_handle, std::vector<Pending>& pending)
{
	if (pending.empty()) return;
	const SOCKET sock = static_cast<SOCKET>(sock_handle);
	const StateCopy st = copy_state();
	const ULONGLONG now = GetTickCount64();

	for (size_t i = 0; i < pending.size();)
	{
		Pending& p = pending[i];
		// 主循环按 15 Hz 发布快照；快照序号至少前进 2，才能保证命令已被主循环处理并反映到状态里。
		const bool settled = st.seq >= p.state_seq0 + 2;
		const bool timed_out = now >= p.deadline_ms;
		bool finished = false;
		bool ok = false;
		std::string reason;

		switch (p.kind)
		{
		case PendingKind::Prepare:
			if (st.vs.self_check_done) { finished = ok = true; }
			else if (settled && st.vs.selfcheck_status == 4) { finished = true; reason = "从端拒绝命令或 ADS 写入失败"; }
			else if (timed_out) { finished = true; reason = "进入准备位置超时"; }
			break;

		case PendingKind::FfZero:
			if (st.extra.force_zero_fail_count != p.zero_fail0)
			{
				finished = true;
				reason = "力传感器零点采集失败（详见从端日志）";
				ff_zeroing_.store(false);
			}
			else if (st.extra.force_zero_ok_count != p.zero_ok0)
			{
				// 零点已采集：此时才真正翻转力反馈（ToggleForceFeedback 是翻转命令，只在当前关闭时发送）。
				if (!st.vs.ff_enabled) push_cmd(VisCommandType::ToggleForceFeedback);
				p.kind = PendingKind::FfEnable;
				p.state_seq0 = st.seq;
				p.deadline_ms = now + 2000;
			}
			else if (timed_out)
			{
				finished = true;
				reason = "力传感器零点采集超时";
				ff_zeroing_.store(false);
			}
			break;

		case PendingKind::FfEnable:
			if (st.vs.ff_enabled) { finished = ok = true; ff_zeroing_.store(false); }
			else if (settled || timed_out)
			{
				finished = true;
				reason = "力反馈未能开启";
				ff_zeroing_.store(false);
			}
			break;

		case PendingKind::FfDisable:
			if (!st.vs.ff_enabled) { finished = ok = true; }
			else if (settled || timed_out) { finished = true; reason = "力反馈未能关闭"; }
			break;

		case PendingKind::CylinderOn:
		{
			const bool engaged = (st.vs.cylinder_manual_mask & (1u << p.index)) != 0;
			if (engaged) { finished = ok = true; }
			else if (settled || timed_out) { finished = true; reason = "电缸未响应（当前未就绪或自动流程正在接管）"; }
			break;
		}

		case PendingKind::CylinderOff:
		{
			const bool engaged = (st.vs.cylinder_manual_mask & (1u << p.index)) != 0;
			if (!engaged) { finished = ok = true; }
			else if (settled || timed_out) { finished = true; reason = "电缸未能恢复原状态"; }
			break;
		}

		case PendingKind::YValve:
			if (st.extra.y_valve_open == !p.flag) { finished = ok = true; }
			else if (settled || timed_out) { finished = true; reason = "Y 阀状态未改变"; }
			break;
		}

		if (finished)
		{
			send_ack(sock, p.id, ok ? "done" : "rejected", reason);
			pending.erase(pending.begin() + i);
		}
		else
		{
			++i;
		}
	}
}

// ====================================================================== UDP 控制/状态通道

void RemoteGateway::udp_loop()
{
	SOCKET us = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (us == INVALID_SOCKET)
	{
		std::cout << "远程网关：UDP 套接字创建失败。" << std::endl;
		return;
	}
	sockaddr_in addr{};
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_ANY);
	addr.sin_port = htons(cfg_.udp_port);
	if (bind(us, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == SOCKET_ERROR)
	{
		std::cout << "远程网关：UDP 端口 " << cfg_.udp_port << " 绑定失败。" << std::endl;
		closesocket(us);
		return;
	}
	while (!stop_.load())
	{
		fd_set rs;
		FD_ZERO(&rs);
		FD_SET(us, &rs);
		timeval tv{ 0, 5000 };
		const int r = select(0, &rs, nullptr, nullptr, &tv);
		if (r > 0)
		{
			unsigned char buf[512];
			sockaddr_in from{};
			int from_len = sizeof(from);
			const int n = recvfrom(us, reinterpret_cast<char*>(buf), sizeof(buf), 0,
				reinterpret_cast<sockaddr*>(&from), &from_len);
			if (n > 0)
			{
				handle_udp(buf, n, from.sin_addr.s_addr, from.sin_port);
			}
		}
		tick_udp(static_cast<std::uintptr_t>(us));
	}
	closesocket(us);
}

void RemoteGateway::handle_udp(const unsigned char* data, int len, std::uint32_t from_ip, std::uint16_t from_port)
{
	if (len != static_cast<int>(rp::kControlFrameLen)) return;
	rp::FrameHeader h{};
	std::memcpy(&h, data, sizeof(h));
	if (h.magic != rp::kMagic || h.version != rp::kVersion || h.type != rp::kTypeControl) return;

	std::lock_guard<std::mutex> lock(m_);
	if (!session_active_ || h.session != session_id_) return;

	unsigned char mac[32] = {};
	const size_t body = static_cast<size_t>(len) - rp::kMacLen;
	if (!hmac_sha256(session_key_, 16, data, body, mac) || !ct_equal(mac, data + body, rp::kMacLen))
	{
		++frames_dropped_;
		return;
	}
	if (seq_has_ && static_cast<std::int32_t>(h.seq - seq_last_) <= 0)
	{
		++frames_dropped_; // 重放或乱序
		return;
	}
	seq_has_ = true;
	seq_last_ = h.seq;
	++frames_ok_;

	const ULONGLONG now = GetTickCount64();
	peer_valid_ = true;
	peer_ip_ = from_ip;
	peer_port_ = from_port;
	last_frame_ms_ = now;
	last_frame_ts_ = h.ts_ms;

	rp::ControlPayload p{};
	std::memcpy(&p, data + rp::kHeaderLen, sizeof(p));
	// 手柄采样（p.handle）本版本尚未接入运动控制，只校验不使用。

	int dirs[3] = { 0, 0, 0 };
	if (lease_held_)
	{
		dirs[0] = clamp_dir(p.injector_dir[0]);
		dirs[1] = clamp_dir(p.injector_dir[1]);
		dirs[2] = clamp_dir(p.axis4_dir);
	}
	apply_holds_locked(dirs, now);
}

void RemoteGateway::tick_udp(std::uintptr_t udp_handle)
{
	const SOCKET us = static_cast<SOCKET>(udp_handle);
	unsigned char out[rp::kStatusFrameLen];
	sockaddr_in to{};
	bool do_send = false;
	{
		std::lock_guard<std::mutex> lock(m_);
		if (!session_active_) return;
		const ULONGLONG now = GetTickCount64();

		if (lease_held_)
		{
			const ULONGLONG silent = now - last_frame_ms_;
			if (silent > kControlFrameWarnMs)
			{
				// 控制帧超时：点动一律归零（只在方向变化时推一次命令）。
				const int zeros[3] = { 0, 0, 0 };
				const bool any_hold = hold_dir_[0] != 0 || hold_dir_[1] != 0 || hold_dir_[2] != 0;
				apply_holds_locked(zeros, now);
				if (any_hold) add_event("warn", "控制帧超时，已将注射器与轴4点动归零。");
			}
			if (silent > kControlFrameReleaseMs)
			{
				release_lease_locked("控制帧持续超时");
				add_event("warn", "控制帧持续超时，已释放控制权，请重新申请。");
			}
		}

		if (peer_valid_ && have_state_ && state_seq_ != last_sent_seq_)
		{
			last_sent_seq_ = state_seq_;
			rp::StatusPayload sp{};
			const VisState& v = state_;
			const std::uint64_t hold = now - last_frame_ms_;
			sp.echo_ts_ms = last_frame_ts_;
			sp.hold_ms = static_cast<std::uint16_t>(hold > 65535 ? 65535 : hold);
			std::uint32_t flags = 0;
			if (v.control_active) flags |= rp::kFlagControlActive;
			if (v.estop_hold) flags |= rp::kFlagEstopHold;
			if (v.self_check_done) flags |= rp::kFlagSelfCheckDone;
			if (v.ff_enabled) flags |= rp::kFlagFfEnabled;
			if (v.cal_zeroed) flags |= rp::kFlagCalZeroed;
			if (v.host_comm_timeout) flags |= rp::kFlagHostCommTimeout;
			if (!extra_.y_valve_open) flags |= rp::kFlagYValveClosed;
			if (lease_held_) flags |= rp::kFlagLeaseHeld;
			if (v.startup_completed) flags |= rp::kFlagStartupCompleted;
			if (v.ads_state == 2) flags |= rp::kFlagAdsHealthy;
			if (ff_zeroing_.load()) flags |= rp::kFlagFfZeroing;
			sp.flags = flags;
			sp.mode = v.guidewire_mode;
			sp.phase = v.startup_phase;
			sp.selfcheck_status = v.selfcheck_status;
			sp.ads_state = v.ads_state;
			for (int i = 0; i < 4; ++i) sp.cylinder_cmd[i] = v.cylinder_cmd[i];
			sp.cylinder_manual_mask = v.cylinder_manual_mask;
			sp.injector_active[0] = extra_.injector_dir[0];
			sp.injector_active[1] = extra_.injector_dir[1];
			sp.axis4_active = extra_.axis4_dir;
			sp.force[0] = fin(v.force_582_f);
			sp.force[1] = fin(v.force_582_n);
			sp.force[2] = fin(v.force_587_f);
			sp.force[3] = fin(v.force_587_n);
			sp.force[4] = fin(v.clean_force_n);
			sp.ads_actual_hz = fin(v.ads_actual_hz);
			for (int i = 0; i < 7; ++i)
			{
				sp.axis_pos[i] = fin(v.axis_pos[i]);
				sp.axis_from_left[i] = fin(v.axis_pos_from_left[i]);
			}

			rp::FrameHeader h{};
			h.magic = rp::kMagic;
			h.version = rp::kVersion;
			h.type = rp::kTypeStatus;
			h.session = session_id_;
			h.seq = ++tx_seq_;
			h.ts_ms = static_cast<std::uint32_t>(GetTickCount());
			std::memcpy(out, &h, rp::kHeaderLen);
			std::memcpy(out + rp::kHeaderLen, &sp, sizeof(sp));
			unsigned char mac[32] = {};
			const size_t body = rp::kHeaderLen + sizeof(sp);
			if (hmac_sha256(session_key_, 16, out, body, mac))
			{
				std::memcpy(out + body, mac, rp::kMacLen);
				to.sin_family = AF_INET;
				to.sin_addr.s_addr = peer_ip_;
				to.sin_port = peer_port_;
				do_send = true;
			}
		}
	}
	if (do_send)
	{
		sendto(us, reinterpret_cast<const char*>(out), static_cast<int>(rp::kStatusFrameLen), 0,
			reinterpret_cast<const sockaddr*>(&to), sizeof(to));
	}
}
