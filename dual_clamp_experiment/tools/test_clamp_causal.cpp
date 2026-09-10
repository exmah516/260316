#include "../ClampCurveBuffer.h"
#include <chrono>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

void require(bool b, const char* message) {
    if (!b) throw std::runtime_error(message);
}
int main(int argc, char** argv) {
    try {
        require(argc == 2, "pass parity fixture CSV");
        std::ifstream file(argv[1]);
        require(bool(file), "cannot read fixture");
        std::string line;
        std::getline(file, line);
        clampmodel::Predictor predictor;
        std::vector<double> timings;
        double max_error = 0;
        int count = 0;
        while (std::getline(file, line)) {
            std::istringstream row(line);
            std::string item;
            std::vector<double> v;
            while (std::getline(row, item, ',')) v.push_back(std::stod(item));
            require(v.size() == 13, "fixture column mismatch");
            const auto& params = v[11] == 1 ? clampmodel::kCatheter : clampmodel::kGuidewire;
            const auto begin = std::chrono::steady_clock::now();
            auto result = predictor.update({v[0], v[1], v[2], v[3], v[4], int(v[5]), int(v[6])}, params);
            timings.push_back(std::chrono::duration<double, std::micro>(
                std::chrono::steady_clock::now() - begin).count());
            require(result.valid == (v[12] != 0), "validity mismatch");
            for (int j = 0; j < 2; ++j)
                max_error = std::max(max_error, std::abs(result.disturbance[j] - v[9+j]));
            if (v[5] < 5 || v[5] > 7)
                require(result.disturbance[0] == 0 && result.disturbance[1] == 0, "normal phase corrected");
            ++count;
        }
        require(count > 100, "fixture too short");
        require(max_error < 1e-8, "Python/C++ parity failed");
        // Independently exercise both output channels even if one mode lacks real data.
        clampmodel::Parameters synthetic;
        synthetic.available = true;
        for (double& s : synthetic.scale) s = 1;
        synthetic.beta[0][0] = 1; synthetic.beta[0][1] = -2;
        predictor.reset();
        for (int i = 0; i < 20; ++i)
            predictor.update({i*.001, 0, 0, 0, 0, 5, 1}, synthetic);
        auto result = predictor.update({.020, 0, 0, 0, 0, 5, 1}, synthetic);
        require(result.valid && result.disturbance[0] == 1 && result.disturbance[1] == -2, "two channels");
        require(!predictor.update({.020, 99, 0, 0, 0, 5, 1}, synthetic).valid, "duplicate sample");
        require(!predictor.update({.100, 0, 0, 0, 0, 5, 1}, synthetic).valid, "gap reset");
        require(!predictor.update({.101, std::numeric_limits<double>::quiet_NaN(), 0, 0, 0, 5, 1}, synthetic).valid, "NaN");
        predictor.reset();
        require(!predictor.update({0, 0, 0, 0, 0, 5, 1}, synthetic).valid, "reset warmup");
        for (int i = 1; i < 20; ++i) predictor.update({i*.001, 0, 0, 0, 0, 4, 1}, synthetic);
        result = predictor.update({.020, 0, 0, 0, 0, 4, 1}, synthetic);
        require(result.valid && result.disturbance[0] == 0 && result.disturbance[1] == 0, "normal phase identity");
        require(!predictor.update({.021, 0, 0, 0, 0, 5, 1}, {}).valid, "unavailable model");
        clampmodel::CurveBuffer buffer;
        for (int i = 0; i < 15000; ++i) buffer.push({0, i*.001, 1, 2, 1, 2, true, 4});
        auto response = buffer.response(0, 1, 2, true, true, true, 0, 511);
        require(response.find("|511|1|") != std::string::npos, "cursor overrun");
        require(std::count(response.begin(), response.end(), ';') == 1023, "bounded batch");
        buffer.reset();
        require(buffer.response(0, 1, 1, false, true, true, 0, 0).rfind("PROGRAM_CURVES|2|1|0|", 0) == 0,
            "mode/zero epoch reset");
        std::sort(timings.begin(), timings.end());
        std::cout << std::setprecision(12) << "{\"samples\":" << count << ",\"max_parity_error_N\":"
            << max_error << ",\"median_us\":" << timings[timings.size()/2]
            << ",\"p95_us\":" << timings[timings.size()*95/100] << ",\"max_us\":" << timings.back()
            << ",\"edge_tests\":\"passed\"}\n";
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n'; return 1;
    }
}
