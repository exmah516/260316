#pragma once
#include <algorithm>
#include <array>
#include <cmath>

namespace externalhandle {
inline bool map(const std::array<double, 2>& joints,
    const std::array<double, 2>& anchor, const std::array<double, 7>& axis_anchor,
    const std::array<double, 7>& init_pos, double prepare_abs, double trigger_abs,
    std::array<double, 7>& refer)
{
    if (!std::isfinite(prepare_abs) || !std::isfinite(trigger_abs) || prepare_abs <= trigger_abs)
        return false;
    for (double value : joints) if (!std::isfinite(value)) return false;
    for (double value : anchor) if (!std::isfinite(value)) return false;
    for (double value : axis_anchor) if (!std::isfinite(value)) return false;
    for (double value : init_pos) if (!std::isfinite(value)) return false;
    const double axis1 = std::clamp(axis_anchor[0] - 750.0 * (joints[0] - anchor[0]),
        trigger_abs, prepare_abs);
    const double axis2 = axis_anchor[1] - (180.0 / 3.14159265358979323846) * (joints[1] - anchor[1]);
    for (std::size_t axis = 0; axis < refer.size(); ++axis)
        refer[axis] = axis_anchor[axis] - init_pos[axis];
    refer[0] = axis1 - init_pos[0];
    refer[1] = axis2 - init_pos[1];
    refer[4] = axis_anchor[4] - init_pos[4];
    refer[5] = axis_anchor[5] + axis1 - axis_anchor[0] - init_pos[5];
    refer[6] = axis2 - init_pos[6];
    return std::all_of(refer.begin(), refer.end(), [](double value) { return std::isfinite(value); });
}
}
