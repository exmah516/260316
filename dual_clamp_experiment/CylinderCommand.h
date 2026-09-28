#pragma once

#include <cstdint>

namespace cylindercommand
{
	// The node board treats WORD values 0..4 as an invalid power-up command.
	inline constexpr std::uint16_t kSafeMinimumWord = 5;

	inline constexpr std::uint16_t normalize(std::uint16_t word)
	{
		return word < kSafeMinimumWord ? kSafeMinimumWord : word;
	}
}
