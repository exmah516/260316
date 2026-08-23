#pragma once

#include "DualClampTypes.h"

#include "ADSComm1.h"

#include <string>
#include <vector>

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
	bool write_experiment_config(const DualClampConfig& config, bool setup_request, bool start_request);
	bool request_abort();
	bool clear_sample_buffer();
	bool read_sample_count(std::uint32_t& count, bool& overflow);
	bool read_all_samples(std::uint32_t count, std::vector<DualClampSample>& samples);

private:
	CADSComm comm_;
};
