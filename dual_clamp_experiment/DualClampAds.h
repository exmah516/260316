#pragma once

#include "DualClampTypes.h"

#include "ADSComm1.h"

#include <string>
#include <vector>
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

struct SelfCheckState
{
	bool valid = false;
	bool done = false;
	std::int32_t status = 0;
	std::int16_t gen_state = 0;
	bool start_pending = false;
	bool host_timeout = true;
	std::string error;
};

class DualClampAds
{
public:
	DualClampAds();
	~DualClampAds();

	bool open();
	void close();
	bool is_open() const;
	std::string last_error() const;

	bool read_live(DualClampLiveFrame& frame);
	bool request_self_check();
	SelfCheckState self_check() const;
	bool write_experiment_config(const DualClampConfig& config, bool setup_request, bool start_request);
	bool request_abort();
	bool clear_sample_buffer();
	bool read_sample_count(std::uint32_t& count, bool& overflow);
	bool read_all_samples(std::uint32_t count, std::vector<DualClampSample>& samples);

private:
	bool begin_host_session();
	static bool read_self_check(CADSComm& comm, SelfCheckState& state);
	void host_loop();
	std::uint32_t heartbeat_sequence_ = 0;
	CADSComm comm_;
	CADSComm host_comm_;
	std::atomic<bool> host_running_{false};
	std::atomic<bool> session_ready_{false};
	std::thread host_thread_;
	mutable std::mutex host_mutex_;
	SelfCheckState self_check_{};
	std::chrono::steady_clock::time_point host_updated_{};
	std::string last_error_;
};
