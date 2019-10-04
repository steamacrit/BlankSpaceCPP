#pragma once

class RobotMap
{
public:
	static constexpr uint32_t LEFT_FWD_DRIVE_MOTOR{ 1 };
	static constexpr uint32_t LEFT_FWD_STEER_MOTOR{ 5 };
	static constexpr uint32_t LEFT_FWD_STEER_ENCODER{ 0 };

	static constexpr uint32_t LEFT_AFT_DRIVE_MOTOR{ 0 };
	static constexpr uint32_t LEFT_AFT_STEER_MOTOR{ 4 };
	static constexpr uint32_t LEFT_AFT_STEER_ENCODER{ 2 };

	static constexpr uint32_t RIGHT_FWD_DRIVE_MOTOR{ 2 };
	static constexpr uint32_t RIGHT_FWD_STEER_MOTOR{ 7 };
	static constexpr uint32_t RIGHT_FWD_STEER_ENCODER{ 1 };

	static constexpr uint32_t RIGHT_AFT_DRIVE_MOTOR{ 3 };
	static constexpr uint32_t RIGHT_AFT_STEER_MOTOR{ 6 };
	static constexpr uint32_t RIGHT_AFT_STEER_ENCODER{ 3 };
	
	static constexpr uint32_t DRIVER_CTRL_PORT{ 0 };
	static constexpr uint32_t OPERATOR_CTRL_PORT{ 1 };
};