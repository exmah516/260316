// remote_protocol.h —— 主从远程协议 v1 的 C++ 定义（从端使用）。
// 权威定义见 protocol/PROTOCOL.md；字段顺序、长度必须与之完全一致。
// 注意：本文件尚未被 ADS.vcxproj 引用，接入远程网关时再加入工程。
#pragma once
#include <cstddef>
#include <cstdint>

namespace remote_protocol {

constexpr uint16_t kMagic = 0x4956;
constexpr uint8_t  kVersion = 1;
constexpr uint8_t  kTypeControl = 1;
constexpr uint8_t  kTypeHaptic = 2;
constexpr uint8_t  kTypeStatus = 3;

constexpr size_t kHeaderLen = 16;
constexpr size_t kMacLen = 8;
constexpr size_t kControlFrameLen = 79;
constexpr size_t kHapticFrameLen = 50;
constexpr size_t kStatusFrameLen = 142;

constexpr uint16_t kTcpPort = 32000;
constexpr uint16_t kUdpPort = 32001;

// 状态标志位（StatusFrame.flags）
enum StatusFlag : uint32_t {
	kFlagControlActive = 1u << 0,
	kFlagEstopHold = 1u << 1,
	kFlagSelfCheckDone = 1u << 2,
	kFlagFfEnabled = 1u << 3,
	kFlagCalZeroed = 1u << 4,
	kFlagHostCommTimeout = 1u << 5,
	kFlagYValveClosed = 1u << 6,
	kFlagLeaseHeld = 1u << 7,
	kFlagStartupCompleted = 1u << 8,
	kFlagAdsHealthy = 1u << 9,
	kFlagFfZeroing = 1u << 10,
};

#pragma pack(push, 1)
struct FrameHeader {
	uint16_t magic;
	uint8_t  version;
	uint8_t  type;
	uint32_t session;
	uint32_t seq;
	uint32_t ts_ms;
};
struct HandleSample {
	uint8_t buttons;
	uint8_t valid;
	int32_t encoders[2];
	float   joints[2];
	float   vels[2];
};
struct ControlPayload {
	HandleSample handle[2];
	int8_t injector_dir[2];
	int8_t axis4_dir;          // 轴4点动方向：-1 后退，0 停，+1 前进
};
struct HapticOut {
	uint8_t enable;
	int8_t  axis;          // 力作用的 SDK 轴（0..2），对应从端 axial_force_axis
	float   force_n;
	float   torque_nm;
};
struct HapticPayload {
	uint32_t echo_ts_ms;
	uint16_t hold_ms;
	HapticOut handle[2];
};
struct StatusPayload {
	uint32_t echo_ts_ms;
	uint16_t hold_ms;
	uint32_t flags;
	int32_t  mode;
	int32_t  phase;
	int32_t  selfcheck_status;
	int32_t  ads_state;
	uint16_t cylinder_cmd[4];
	uint8_t  cylinder_manual_mask;
	int8_t   injector_active[2];
	int8_t   axis4_active;
	float    force[5];       // 582_f, 582_n, 587_f, 587_n, clean_force_n
	float    ads_actual_hz;
	float    axis_pos[7];
	float    axis_from_left[7];
};
#pragma pack(pop)

static_assert(sizeof(FrameHeader) == kHeaderLen, "帧头长度错误");
static_assert(sizeof(HandleSample) == 26, "HandleSample 长度错误");
static_assert(sizeof(ControlPayload) == 55, "ControlPayload 长度错误");
static_assert(sizeof(HapticOut) == 10, "HapticOut 长度错误");
static_assert(sizeof(HapticPayload) == 26, "HapticPayload 长度错误");
static_assert(sizeof(StatusPayload) == 118, "StatusPayload 长度错误");
static_assert(kHeaderLen + sizeof(ControlPayload) + kMacLen == kControlFrameLen, "ControlFrame 长度错误");
static_assert(kHeaderLen + sizeof(HapticPayload) + kMacLen == kHapticFrameLen, "HapticFrame 长度错误");
static_assert(kHeaderLen + sizeof(StatusPayload) + kMacLen == kStatusFrameLen, "StatusFrame 长度错误");

}  // namespace remote_protocol
