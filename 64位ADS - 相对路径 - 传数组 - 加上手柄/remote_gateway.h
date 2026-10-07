// 文件职责说明：
// 从端远程网关。让主端（MasterConsole）经 TCP/UDP 远程操作本机 ADS.exe：
// 1) TCP 命令通道（长度前缀 JSON）：HMAC 挑战握手、控制权租约、命令校验与回执。
// 2) UDP 控制通道：校验 MAC/会话/序号后，把注射器与轴4的“按住”状态转成带租约的点动命令；
//    帧超时自动归零。同一通道回传 15 Hz 状态帧。
// 3) 命令一律翻译成既有 VisCommand 推入网关队列，由 main.cpp 的命令循环与本地调试台
//    共用同一套处理逻辑，不绕过任何既有保护。
// 协议权威定义见 protocol/PROTOCOL.md；帧结构见 protocol/cpp/remote_protocol.h。
// 注意：手柄采样（ControlFrame.handle）本版本只做校验与统计，尚未接入运动控制。
#pragma once

#include "vis_server.h"

#include <atomic>
#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// 协议里的 JSON 消息都是扁平对象（无嵌套），只需要这四种取值。
struct RemoteJsonValue
{
	enum Type { Null, Bool, Number, String };
	Type type = Null;
	bool b = false;
	double n = 0.0;
	std::string s;
};
using RemoteJsonObject = std::map<std::string, RemoteJsonValue>;

struct RemoteGatewayConfig
{
	// 预共享密钥文件（纯文本，至少 16 个字符）。文件不存在时网关不启用。
	std::string token_path = "remote.token";
	std::uint16_t tcp_port = 32000;
	std::uint16_t udp_port = 32001;
	// 准备位置命令需要由导管/Y阀位置推算轴5、轴6目标，必须与 ControlConfig.startup_g_mm 一致。
	double startup_g_mm = 13.0;
};

// main.cpp 在发布 VisState 的同时提供给网关的补充状态（VisState 本身不含这些量）。
struct RemoteExtraState
{
	std::int8_t injector_dir[2] = {};
	std::int8_t axis4_dir = 0;
	bool y_valve_open = false;
	// 力传感器零点采集成功/失败累计次数，用于判断“开启力反馈前自动零点采集”是否完成。
	std::uint32_t force_zero_ok_count = 0;
	std::uint32_t force_zero_fail_count = 0;
	// 从端主循环的“为什么不能动”诊断量，用于把拒绝原因说清楚。
	bool initial_sync_done = false;
	bool handle_soft_hold = false;
	bool ads_soft_hold = false;
	bool connection_hold = false;
};

class RemoteGateway
{
public:
	RemoteGateway() = default;
	~RemoteGateway();
	RemoteGateway(const RemoteGateway&) = delete;
	RemoteGateway& operator=(const RemoteGateway&) = delete;

	bool start(const RemoteGatewayConfig& cfg);
	void stop();
	bool is_running() const { return running_.load(); }
	// main.cpp 在确定手柄来源后设置：手柄来自主端时，相关拒绝原因会提示“未收到主端手柄数据”。
	void set_handles_remote(bool on) { handles_remote_.store(on); }

	// 主循环约 15 Hz 调用：提供最新状态快照。
	void publish_state(const VisState& state, const RemoteExtraState& extra);
	// 主循环每拍调用：取出一条远程翻译出的命令。
	bool poll_command(VisCommand& cmd);

private:
	enum class PendingKind
	{
		Prepare,
		FfZero,
		FfEnable,
		FfDisable,
		CylinderOn,
		CylinderOff,
		YValve,
		StartControl,
	};

	struct Pending
	{
		PendingKind kind = PendingKind::Prepare;
		int id = 0;
		ULONGLONG deadline_ms = 0;
		std::uint32_t state_seq0 = 0;
		int index = 0;       // 电缸下标 0..3
		bool flag = false;   // YValve: 目标是否为“关闭”
		std::uint32_t zero_ok0 = 0;
		std::uint32_t zero_fail0 = 0;
	};

	enum class AuthState { Hello, Challenge, Authed };

	struct StateCopy
	{
		bool valid = false;
		bool lease = false;
		VisState vs{};
		RemoteExtraState extra{};
		std::uint32_t seq = 0;
	};

	void tcp_loop();
	void udp_loop();
	void handle_client(std::uintptr_t sock);
	bool handle_message(std::uintptr_t sock, const std::string& text, AuthState& auth,
		unsigned char nonce_c[16], unsigned char nonce_s[16], std::vector<Pending>& pending);
	void handle_command(std::uintptr_t sock, int id, const std::string& name,
		const RemoteJsonObject& fields, std::vector<Pending>& pending);
	void process_pending(std::uintptr_t sock, std::vector<Pending>& pending);
	void flush_events(std::uintptr_t sock);

	// 返回 true 表示收到并接受了一个合法控制帧（调用方据此回一个触觉帧）。
	bool handle_udp(const unsigned char* data, int len, std::uint32_t from_ip, std::uint16_t from_port);
	void send_haptic(std::uintptr_t udp_sock);
	void tick_udp(std::uintptr_t udp_sock);

	StateCopy copy_state();
	void push_cmd(VisCommandType type, int p1 = 0, int p2 = 0);
	void add_event(const char* level, const std::string& text);
	// 以下 *_locked 函数要求调用者已持有 m_。
	void apply_holds_locked(const int dirs[3], ULONGLONG now);
	void release_lease_locked(const char* reason);
	void end_session_locked(const char* reason);

	bool hmac_sha256(const unsigned char* key, size_t key_len,
		const unsigned char* data, size_t data_len, unsigned char out[32]);
	bool random_bytes(unsigned char* out, size_t len);

	RemoteGatewayConfig cfg_;
	std::vector<unsigned char> token_;
	std::atomic<bool> running_{ false };
	std::atomic<bool> stop_{ false };
	std::atomic<bool> ff_zeroing_{ false };
	std::atomic<bool> handles_remote_{ false };
	std::thread tcp_thread_;
	std::thread udp_thread_;
	void* bcrypt_alg_ = nullptr;
	bool wsa_started_ = false;

	// 受 m_ 保护：最新状态与会话。
	std::mutex m_;
	VisState state_{};
	RemoteExtraState extra_{};
	bool have_state_ = false;
	std::uint32_t state_seq_ = 0;
	ULONGLONG state_tick_ms_ = 0;
	std::uint32_t last_sent_seq_ = 0;

	bool session_active_ = false;
	std::uint32_t session_id_ = 0;
	unsigned char session_key_[16] = {};
	bool lease_held_ = false;
	bool seq_has_ = false;
	std::uint32_t seq_last_ = 0;
	std::uint32_t tx_seq_ = 0;
	bool peer_valid_ = false;
	std::uint32_t peer_ip_ = 0;      // 网络字节序
	std::uint16_t peer_port_ = 0;    // 网络字节序
	ULONGLONG last_frame_ms_ = 0;
	std::uint32_t last_frame_ts_ = 0;
	std::uint64_t frames_ok_ = 0;
	std::uint64_t frames_dropped_ = 0;
	int hold_dir_[3] = {};           // 0=注射器1，1=注射器2，2=轴4
	ULONGLONG hold_last_push_ms_[3] = {};

	// 命令队列（供 main.cpp 取出）与事件队列（发给主端日志）。
	std::mutex cmd_m_;
	std::deque<VisCommand> cmd_q_;
	std::mutex ev_m_;
	std::deque<std::string> events_;
};
