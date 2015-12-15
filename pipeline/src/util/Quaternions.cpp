#include "../../util/Quaternions.h"

using quat = boost::math::quaternion<double>;

namespace Util {

quat getQuatForRotation(const quat &axis, const double theta) {
    return quat(std::cos(theta / 2.),
                std::sin(theta / 2.) * axis.R_component_2(),
                std::sin(theta / 2.) * axis.R_component_3(),
                std::sin(theta / 2.) * axis.R_component_4());
}

quat quaternionRotation(const quat &q, const quat &v) {
    return (q * v) * boost::math::conj(q);
}

quat getIntrinsicRotationQuatZYX(const double rz, const double ry, const double rx) {
    static const quat xaxis(0., 1., 0., 0.);
    static const quat yaxis(0., 0., 1., 0.);
    static const quat zaxis(0., 0., 0., 1.);

    const quat zquat = getQuatForRotation(zaxis, rz);
    const quat yaxis_r = quaternionRotation(zquat, yaxis);

    const quat yquat = getQuatForRotation(yaxis_r, ry);
    const quat xaxis_r = quaternionRotation(yquat, quaternionRotation(zquat, xaxis));

    const quat xquat = getQuatForRotation(xaxis_r, rx);

    return (xquat * yquat) * zquat;
}

quat getIntrinsicRotationQuatZXZ(const double rz1, const double rx2, const double rz3) {
    static const quat xaxis(0., 1., 0., 0.);
    static const quat zaxis(0., 0., 0., 1.);

    const quat zquat1 = getQuatForRotation(zaxis, rz1);
    const quat xaxis_r = quaternionRotation(zquat1, xaxis);

    const quat xquat = getQuatForRotation(xaxis_r, rx2);
    const quat zaxis_r = quaternionRotation(xquat, zaxis);

    const quat zquat2 = getQuatForRotation(zaxis_r, rz3);

    return (zquat2 * xquat) * zquat1;
}

std::array<double, 3> getEulerAngles(const quat &q) {
    static const double pi = boost::math::constants::pi<double>();
    const double temp = 2 * (q.R_component_4() * q.R_component_2() -
                             q.R_component_1() * q.R_component_3());
    if (almost_equal(temp, 1., 2)) {
        const double yangle = -pi / 2;
        const double angle = atan2((q.R_component_2() * q.R_component_3()) -
                                   (q.R_component_1() * q.R_component_4()),
                                   (q.R_component_2() * q.R_component_4()) +
                                   (q.R_component_1() * q.R_component_3()));
        const double xangle = pi;
        const double zangle = angle - xangle;
        return {xangle, yangle, zangle};
    } else if (almost_equal(temp, -1., 2)) {
        const double yangle = pi / 2;
        const double angle = atan2((q.R_component_2() * q.R_component_3()) -
                                   (q.R_component_1() * q.R_component_4()),
                                   (q.R_component_2() * q.R_component_4()) +
                                   (q.R_component_1() * q.R_component_3()));
        const double xangle = pi;
        const double zangle = xangle - angle;
        return {xangle, yangle, zangle};
    } else {
        const double xangle = atan2(2 * (q.R_component_1() * q.R_component_2() +
                                         q.R_component_3() * q.R_component_4()),
                                    1 - 2 * (q.R_component_2() * q.R_component_2() +
                                             q.R_component_3() * q.R_component_3()));
        const double yangle = asin(-temp);
        const double zangle = atan2(2 * (q.R_component_1() * q.R_component_4() +
                                         q.R_component_2() * q.R_component_3()),
                                    1 - 2 * (q.R_component_3() * q.R_component_3() +
                                             q.R_component_4() * q.R_component_4()));

        return {xangle, yangle, zangle};
    }
}

std::array<double, 4> quatToArray(const quat &q) {
    return {q.R_component_1(), q.R_component_2(), q.R_component_3(), q.R_component_4()};
}

}
