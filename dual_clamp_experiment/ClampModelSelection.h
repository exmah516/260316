#pragma once
#include "ClampDisturbanceParameters.h"

namespace clampmodel {
// Explicit user-requested cross-mechanism preview; not catheter identification.
inline constexpr bool kCatheterBorrowGuidewire = true;
inline const Parameters& selected_parameters(bool guidewire) {
    return (guidewire || kCatheterBorrowGuidewire) ? kGuidewire : kCatheter;
}
inline const char* parameter_source(bool guidewire) {
    return (guidewire || kCatheterBorrowGuidewire) ? "guidewire" : "catheter";
}
} // namespace clampmodel
