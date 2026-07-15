#pragma once

#include "common_types.h"

namespace interface::hal {

class ImuHAL {
public:
    virtual ~ImuHAL() = default;
    virtual bool Start() = 0;
    virtual void Stop() = 0;
    virtual types::Vec3f GetRpy() = 0;
    virtual types::Vec3f GetAcc() = 0;
    virtual types::Vec3f GetOmega() = 0;
    virtual bool IsDataValid() = 0;
};

}  // namespace interface::hal
