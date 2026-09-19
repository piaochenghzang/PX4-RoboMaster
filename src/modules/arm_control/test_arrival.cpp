#include "ArrivalCheck.hpp"
#include <cassert>
#include <limits>
int main() {
    ArrivalCheck a;
    assert(!a.update(100000, true, .001f, .01f, .01f));
    assert(!a.update(599999, true, .001f, .01f, .01f));
    assert(a.update(600000, true, .001f, .01f, .01f));
    assert(!a.update(700000, false, 0, 0, 0));
    assert(!a.update(800000, true, .01f, 0, 0));
    assert(!a.update(900000, true, 0, .1f, 0));
    assert(!a.update(1000000, true, 0, 0, 1));
    assert(!a.update(1100000, true, std::numeric_limits<float>::quiet_NaN(), 0, 0));
    assert(!a.update(1200000, true, 0, 0, 0));
    a.reset();
    assert(!a.update(1800000, true, 0, 0, 0));
    assert(!a.update(1000, true, 0, 0, 0));
    assert(a.update(501000, true, 0, 0, 0));
}
