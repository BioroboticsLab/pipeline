#pragma once

#include <cmath>
#include <limits>
#include <type_traits>
#include <algorithm>

#include <boost/math/quaternion.hpp>
#include <boost/math/constants/constants.hpp>

namespace Util {

template<class T>
typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type
    almost_equal(T x, T y, int ulp)
{
    // the machine epsilon has to be scaled to the magnitude of the values used
    // and multiplied by the desired precision in ULPs (units in the last place)
    return std::abs(x-y) < std::numeric_limits<T>::epsilon() * std::abs(x+y) * ulp
    // unless the result is subnormal
           || std::abs(x-y) < std::numeric_limits<T>::min();
}

boost::math::quaternion<double> getQuatForRotation(const boost::math::quaternion<double>& axis,
                                                   const double theta);
boost::math::quaternion<double> quaternionRotation(const boost::math::quaternion<double>& q,
                                                   const boost::math::quaternion<double>& v);
boost::math::quaternion<double> getIntrinsicRotationQuatZYX(const double rz,
                                                            const double ry,
                                                            const double rx);
boost::math::quaternion<double> getIntrinsicRotationQuatZXZ(const double rz1,
                                                            const double rx2,
                                                            const double rz3);
std::array<double, 3> getEulerAngles(const boost::math::quaternion<double>& q);
std::array<double, 4> quaternionToArray(const boost::math::quaternion<double>& q);

}
