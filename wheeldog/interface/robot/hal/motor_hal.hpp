#pragma once

namespace interface::hal {

class MotorHAL {
public:
    enum class Mode { Position, Velocity, Torque };

    virtual ~MotorHAL() = default;
    virtual bool Enable() = 0;
    virtual bool Disable() = 0;
    virtual bool SetMode(Mode mode) = 0;
    virtual bool SetTargetPos(float rad) = 0;
    virtual bool SetTargetVel(float rad_per_s) = 0;
    virtual bool SetTargetTorque(float nm) = 0;
    virtual float GetActualPos() = 0;
    virtual float GetActualVel() = 0;
    virtual float GetActualTorque() = 0;
    virtual bool IsHealthy() = 0;
};

}  // namespace interface::hal
