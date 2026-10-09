#pragma once
#include "ADSComm1.h"
#include <string>
#include <vector>

inline bool read_ads_array(CADSComm& comm, const char* symbol, unsigned long offset,
    unsigned long count, unsigned long element_size, void* output)
{
    if (count == 0) return true;
    if (offset == 0) return comm.ADSRead(symbol, count * element_size, output);
    std::vector<std::string> names(count);
    std::vector<const char*> symbols(count);
    std::vector<unsigned long> lengths(count, element_size);
    std::vector<void*> outputs(count);
    for (unsigned long index = 0; index < count; ++index) {
        names[index] = std::string(symbol) + "[" + std::to_string(offset + index) + "]";
        symbols[index] = names[index].c_str();
        outputs[index] = static_cast<unsigned char*>(output) + index * element_size;
    }
    return comm.ADSReadSum(symbols.data(), lengths.data(), outputs.data(), count);
}
