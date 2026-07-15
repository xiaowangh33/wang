
#include "state_machine.hpp"


#include "quardLeg_defines.h"
#include "robo_core.h"
#include "robo_rpc.h"


using namespace ROBO_CORE;

using namespace types;

MotionStateFeedback StateBase::msfb_ = MotionStateFeedback();

json handler_func_cmd(const json &args, const json &kwargs)
{
	std::pair<int, json> result;
	bool ret = get_args_cmd_pre_run(args, kwargs, result);
	if (!ret)
	{
		throw std::runtime_error("CtrlCmdRpc: invalid arguments");
	}
	int cmd = result.first;
	json cmd_data = result.second;

	return json{{"CtrlCmd", cmd}};
};


int main(int argc, char **argv){
    if (argc < 4){
        std::cerr << "rl_deploy must be started by robo_core_main with 3 arguments\n";
        return 2;
    }

    try{
        ROBO_CORE::init_core_client(argv[1], argv[2], argv[3], handler_func_cmd);

        sbf_robot_state =
            ROBO_CORE::open_sharemem_yaml("${PROGRAM_ROOT_PATH}/Sources/robot_state_shm.yaml");
        if(!sbf_robot_state || !sbf_robot_state->pdata){
            throw std::runtime_error("failed to open robot_state shared memory");
        }

        robot_data_all = static_cast<ShmRobotDataAll *>(sbf_robot_state->pdata);
        init_RobotDataAll();

        StateMachine state_machine(RobotType::Mydog);
        state_machine.Run();
    }catch(const std::exception& e){
        std::cerr << "rl_deploy fatal error: " << e.what() << std::endl;
        ROBO_CORE::end_core();
        return 1;
    }

    ROBO_CORE::end_core();
    return 0;
}
