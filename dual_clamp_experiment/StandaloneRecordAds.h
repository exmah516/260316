#pragma once

#include "StandaloneRecordTypes.h"

#include "ADSComm1.h"

#include <array>
#include <cstdint>
#include <string>

class StandaloneRecordAds
{
public:
    StandaloneRecordAds();
    ~StandaloneRecordAds();

    bool open();
    void close();
    bool is_open() const;
    std::string last_error() const;

    bool read_live(StandaloneRecordLiveFrame& frame);
    bool set_record_enable(bool enable, std::uint32_t event_sequence);
    bool set_manual_cylinder(bool enable, const std::array<std::uint16_t, 4>& values);

private:
    CADSComm comm_;
};

