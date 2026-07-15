#ifndef CUSTOM_TYPES_H_
#define CUSTOM_TYPES_H_

#include "common_types.h"
#include <cstdlib>

namespace types{
    enum RobotType{
        Mydog,
    };

    enum RobotMotionState{
        WaitingForStand   = 0,
        StandingUp        = 1,
        JointDamping      = 2,

        RLControlMode     = 6,
        PACEChirpCollect  = 7,
    };

    enum StateName{
        kInvalid           = -1,
        kIdle              = 0,
        kStandUp           = 1,
        kJointDamping      = 2,

        kRLControl         = 6,
        kPACEChirpCollect  = 7,
        kSitDown           = 8,
    };
    

    inline std::string GetAbsPath(){
        const char* program_root = std::getenv("PROGRAM_ROOT_PATH");
        if(program_root && program_root[0] != '\0'){
            return std::string(program_root);
        }
        char buffer[PATH_MAX];
        if(getcwd(buffer, sizeof(buffer)) != NULL){
            return std::string(buffer);
        }
        return "";
    }
};

#endif
