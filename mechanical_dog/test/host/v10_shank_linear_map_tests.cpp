#include <cassert>
#include <cmath>
#include <initializer_list>
#include <limits>

#include "Ctrl/V10ShankLinearMap.h"

namespace {
bool near(float a, float b, float tolerance = 0.001f) {
    return std::fabs(a - b) <= tolerance;
}
}

int main() {
    using namespace dog;
    constexpr V10ShankMapPoint points[] = {{0, 0}, {30, 9}, {50, 23}};
    const V10ShankLinearMap map(points, 3);
    assert(map.valid() && near(map.maxU(), 50) && near(map.maxQ(), 23));

    float q = 123.0f;
    for (const auto& sample : {V10ShankMapPoint{0, 0}, {30, 9},
                               {40, 16}, {50, 23}, {37, 13.9f}}) {
        assert(map.qFromU(sample.u_deg, q) && near(q, sample.q_deg));
        float u = 124.0f;
        assert(map.uFromQ(q, u) && near(u, sample.u_deg));
    }

    // Both segment boundaries are continuous and round-trip from either side.
    for (int tenths = 0; tenths <= 500; ++tenths) {
        const float input = tenths / 10.0f;
        float result = -1.0f;
        assert(map.qFromU(input, result));
        float round_trip = -1.0f;
        assert(map.uFromQ(result, round_trip) && near(round_trip, input));
    }

    // Rejections do not clamp, extrapolate, or mutate the output argument.
    q = 123.0f;
    assert(!map.qFromU(-0.1f, q));
    assert(!map.qFromU(50.1f, q));
    assert(!map.qFromU(std::numeric_limits<float>::quiet_NaN(), q));
    assert(near(q, 123.0f));
    float u = 124.0f;
    assert(!map.uFromQ(-0.1f, u));
    assert(!map.uFromQ(23.1f, u));
    assert(!map.uFromQ(std::numeric_limits<float>::infinity(), u));
    assert(near(u, 124.0f));

    constexpr V10ShankMapPoint no_origin[] = {{1, 0}, {30, 9}};
    constexpr V10ShankMapPoint flat_u[] = {{0, 0}, {30, 9}, {30, 23}};
    constexpr V10ShankMapPoint falling_q[] = {{0, 0}, {30, 9}, {50, 8}};
    const V10ShankMapPoint non_finite[] = {
        {0, 0}, {30, std::numeric_limits<float>::quiet_NaN()}};
    assert(!V10ShankLinearMap(nullptr, 3).valid());
    assert(!V10ShankLinearMap(points, 1).valid());
    assert(!V10ShankLinearMap(no_origin, 2).valid());
    assert(!V10ShankLinearMap(flat_u, 3).valid());
    assert(!V10ShankLinearMap(falling_q, 3).valid());
    assert(!V10ShankLinearMap(non_finite, 2).valid());
    q = 125.0f;
    u = 126.0f;
    const V10ShankLinearMap invalid_map(falling_q, 3);
    assert(!invalid_map.qFromU(10, q) && near(q, 125.0f));
    assert(!invalid_map.uFromQ(5, u) && near(u, 126.0f));
}
