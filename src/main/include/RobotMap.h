#pragma once

class RobotMap
{
public:
	static constexpr uint32_t LEFT_FWD_DRIVE_CTRL{ 1 };
	static constexpr uint32_t LEFT_FWD_STEER_CTRL{ 5 };
	static constexpr uint32_t LEFT_FWD_STEER_ENCODER{ 0 };

	static constexpr uint32_t LEFT_AFT_DRIVE_CTRL{ 0 };
	static constexpr uint32_t LEFT_AFT_STEER_CTRL{ 4 };
	static constexpr uint32_t LEFT_AFT_STEER_ENCODER{ 2 };

	static constexpr uint32_t RIGHT_FWD_DRIVE_CTRL{ 2 };
	static constexpr uint32_t RIGHT_FWD_STEER_CTRL{ 7 };
	static constexpr uint32_t RIGHT_FWD_STEER_ENCODER{ 1 };

	static constexpr uint32_t RIGHT_AFT_DRIVE_CTRL{ 3 };
	static constexpr uint32_t RIGHT_AFT_STEER_CTRL{ 6 };
	static constexpr uint32_t RIGHT_AFT_STEER_ENCODER{ 3 };
	
	static constexpr uint32_t DRIVER_CTRL_PORT{ 0 };
	static constexpr uint32_t OPERATOR_CTRL_PORT{ 1 };
};