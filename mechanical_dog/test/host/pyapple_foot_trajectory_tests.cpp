#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

#include "Ctrl/PyAppleFootTrajectory.h"

namespace {

bool close(float a,float b,float tolerance=0.00001f) {
    return std::fabs(a-b)<=tolerance;
}

}  // namespace

int main() {
    // Frozen directly from padog.py:315-351 after normalising t/Tf and using
    // x0=z0=xv0=0. No hardware-bearing padog module is imported by this test.
    const float u[5]={0.0f,0.25f,0.5f,0.75f,1.0f};
    const float swing_x[5]={0.0f,0.0f,0.5f,1.0f,1.0f};
    const float swing_z[5]={0.0f,0.5f,1.0f,0.75f,0.0f};
    const float support_x[5]={1.0f,0.75f,0.5f,0.25f,0.0f};
    for(int i=0;i<5;++i){
        dog::PyAppleFootPoint swing{},support{};
        assert(dog::pyAppleSwingFoot(u[i],swing));
        assert(dog::pyAppleSupportFoot(u[i],support));
        assert(close(swing.x,swing_x[i])&&close(swing.z,swing_z[i]));
        assert(close(support.x,support_x[i])&&close(support.z,0.0f));
    }
    dog::PyAppleFootPoint before{},at{},after{};
    assert(dog::pyAppleSwingFoot(0.75f-0.00001f,before));
    assert(dog::pyAppleSwingFoot(0.75f,at));
    assert(dog::pyAppleSwingFoot(0.75f+0.00001f,after));
    assert(close(at.x,1.0f)&&close(after.x,1.0f)&&close(before.x,1.0f,0.0001f));
    dog::PyAppleFootPoint touchdown{};
    assert(dog::pyAppleSwingFoot(0.999f,touchdown));
    assert(touchdown.z/0.001f>3.9f); // Original touchdown vertical speed is not zero.
    dog::PyAppleFootPoint amplitude{};
    assert(dog::pyAppleSwingFoot(0.5f,amplitude));
    assert(close(16.0f*(amplitude.x-0.5f),0.0f));
    assert(close(5.0f*amplitude.z,5.0f));
    dog::PyAppleFootPoint sentinel{7.0f,8.0f};
    assert(!dog::pyAppleSwingFoot(-0.01f,sentinel)&&close(sentinel.x,7.0f));
    assert(!dog::pyAppleSupportFoot(std::numeric_limits<float>::quiet_NaN(),sentinel));
    std::cout << "Py-Apple foot trajectory tests passed\n";
    return 0;
}

