#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/components/wait_for_fmu.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Core>

using namespace std::chrono_literals; // NOLINT

static const std::string kName = "Executor";
class MyModeExecutor : public px4_ros2::ModeExecutorBase // [1]
{
public:
	MyModeExecutor(rclcpp::Node &node, px4_ros2::ModeBase &owned_mode) // [2]
		: ModeExecutorBase(node, px4_ros2::ModeExecutorBase::Settings{}, owned_mode),
		  _node(node)
	{
	}

	enum class State // [3]
	{
		Reset,
		TakingOff,
		MyMode,
		RTL,
		WaitUntilDisarmed,
	};

	void onActivate() override
	{
		runState(State::TakingOff, px4_ros2::Result::Success); // [4]
	}

	void onDeactivate(DeactivateReason reason) override {}

	void runState(State state, px4_ros2::Result previous_result)
	{
		if (previous_result != px4_ros2::Result::Success)
		{
			RCLCPP_ERROR(_node.get_logger(), "State %i: previous state failed: %s", (int)state,
						 resultToString(previous_result));
			return;
		}

		switch (state)
		{ // [5]
		case State::Reset:
			break;

		case State::TakingOff:
			takeoff([this](px4_ros2::Result result)
					{ runState(State::MyMode, result); });
			break;

		case State::MyMode: // [6]
			scheduleMode(
				ownedMode().id(), [this](px4_ros2::Result result)
				{ runState(State::RTL, result); });
			break;

		case State::RTL:
			rtl([this](px4_ros2::Result result)
				{ runState(State::WaitUntilDisarmed, result); });
			break;

		case State::WaitUntilDisarmed:
			waitUntilDisarmed([this](px4_ros2::Result result)
							  { RCLCPP_INFO(_node.get_logger(), "All states complete (%s)", resultToString(result)); });
			break;
		}
	}

private:
	rclcpp::Node &_node;
};
