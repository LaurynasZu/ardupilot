#pragma once
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_InertialNav/AP_InertialNav.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <stdint.h>
#include "PosVelEKF.h"
#include <AP_Buffer/AP_Buffer.h>
// declare backend classes
class AC_PrecLand_Backend;
class AC_PrecLand_Companion;
class AC_PrecLand_IRLock;
class AC_PrecLand_SITL_Gazebo;
class AC_PrecLand_SITL;
class AC_PrecLand_RTK;

class AC_PrecLand
{
    // declare backends as friends
    friend class AC_PrecLand_Backend;
    friend class AC_PrecLand_Companion;
    friend class AC_PrecLand_IRLock;
    friend class AC_PrecLand_SITL_Gazebo;
    friend class AC_PrecLand_SITL;
    friend class AC_PrecLand_RTK;

public:

    // precision landing behaviours (held in PRECLAND_ENABLED parameter)
    enum PrecLandBehaviour {
        PRECLAND_BEHAVIOUR_DISABLED,
        PRECLAND_BEHAVIOR_ALWAYSLAND,
        PRECLAND_BEHAVIOR_CAUTIOUS
    };
    // types of precision landing (used for PRECLAND_TYPE parameter)
    enum PrecLandType {
        PRECLAND_TYPE_NONE = 0,
        PRECLAND_TYPE_COMPANION,
        PRECLAND_TYPE_IRLOCK,
        PRECLAND_TYPE_SITL_GAZEBO,
        PRECLAND_TYPE_SITL,
        PRECLAND_TYPE_RTK
    };

    // constructor
    AC_PrecLand(const AP_AHRS& ahrs, const AP_InertialNav& inav);
    // perform any required initialisation of landing controllers
    void init();
    // returns true if precision landing is healthy
    bool healthy() const { return _backend_state.healthy; }
    // returns true if precision landing is enabled (used only for logging)
    bool enabled() const { return _enabled.get(); }
    // returns time of last update
    uint32_t last_update_ms() const { return _last_update_ms; }
    // give chance to driver to get updates from sensor, should be called at 400hz
    void update(float rangefinder_alt_cm, bool rangefinder_alt_valid);
    // returns target position relative to the EKF origin
    bool get_target_position_cm(Vector2f& ret);