#pragma once

#include "ProgrammedDeliveryTypes.h"

#include "ADSComm1.h"

#include <string>
#include <vector>

class ProgrammedDeliveryAds
{
public:
	ProgrammedDeliveryAds();
	~ProgrammedDeliveryAds();

	bool open();
	void close();
	bool is_open() const;
	std::string last_error() const;

	bool select_mode(ProgrammedDeliveryMode mode);
	bool read_live(ProgrammedDeliveryLiveFrame& frame);
	bool write_config(const ProgrammedDeliveryConfig& config, bool setup_request);
	bool request_start();
	bool request_abort();
	bool clear_sample_buffer();
	bool read_sample_count(std::uint32_t& count, bool& overflow);
	bool read_all_samples(ProgrammedDeliveryMode mode, std::uint32_t count, std::vector<ProgrammedDeliverySample>& samples);

private:
	CADSComm comm_;
};
