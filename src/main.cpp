// #include directly copies the content of the specified file into the place where the #include directive is used.
// when including standard C++ libraries, you use angle brackets. When including your own files, you use quotes.
// https://en.cppreference.com/w/cpp/preprocessor/include
#include "main.h"
#include "pros/motors.hpp"
#include "rev/rev.hh"
#include "globals.hh"
#include "SerialRX.hh"
#include "SerialTX.hh"
#include "HybridOdometry.hh"
#include "rev/api/alg/reckless/turn_segment.hh"
// #include "campbell_turn_copy.hh"

#include <chrono>
#include <string>

// this makes it so you can just put the name of the item rather then putting rev:: before everything
using namespace rev;

using std::to_string;
using namespace std;

#define DRIVER1 0

void print_position();
void print_color_sensor(int* bbs, int* rbs);
void update_balls_seen(int* bbs, int* rbs, double* lv_r, double* lv_b);
void receive_data(std::string sender, std::string message);
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	pros::lcd::initialize();


	odom = std::make_shared<rev::HybridOdometry>(
		fwd,				  // The forward sensor
		lat,				  // The rightward sensor
		imu,				  // Inertial sensor
		WHEEL_DIAMETER,		  // Diameter of forward wheel
		WHEEL_DIAMETER,		  // Diameter of sideways wheel
		FORWARD_WHEEL_OFFSET, // How far to the right of the center of the robot the forward wheel is
		LATERAL_WHEEL_OFFSET  // How far to the rear of the robot the lateral wheel is from the center
	);

	reckless = std::make_shared<Reckless>(chassis, odom);

	pros::delay(2000);

	odom->reset_position();
	odom_runner = std::make_shared<rev::AsyncRunner>(odom);
	reckless_runner = std::make_shared<rev::AsyncRunner>(reckless);
	mogoClamp.set_value(RELEASE);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous()
{
	// **This is for initialization. Do not put reckless code in this area**
	/*odom = std::make_shared<rev::HybridOdometry>(
		fwd,				  // The forward sensor
		lat,				  // The rightward sensor
		imu,				  // Inertial sensor
		WHEEL_DIAMETER,		  // Diameter of forward wheel
		WHEEL_DIAMETER,		  // Diameter of sideways wheel
		FORWARD_WHEEL_OFFSET, // How far to the right of the center of the robot the forward wheel is
		LATERAL_WHEEL_OFFSET  // How far to the rear of the robot the lateral wheel is from the center
	);

	reckless = std::make_shared<Reckless>(chassis, odom);

	pros::delay(2000);*/

	const PilonsSegmentParams FAST{
		std::make_shared<ConstantMotion>(FAST_POWER),
		std::make_shared<PilonsCorrection>(FAST_CORRECTION_COEFF, FAST_CORRECTION_DIST),
		std::make_shared<SimpleStop>(FAST_HARSH_THRESHOLD, FAST_COAST_THRESHOLD, FAST_COAST_POWER)};

	const PilonsSegmentParams MEDIUM{
		std::make_shared<ConstantMotion>(MID_POWER),
		std::make_shared<PilonsCorrection>(MID_CORRECTION_COEFF, MID_CORRECTION_DIST),
		std::make_shared<SimpleStop>(MID_HARSH_THRESHOLD, MID_COAST_THRESHOLD, MID_COAST_POWER)};

	const PilonsSegmentParams SLOW{
		std::make_shared<ConstantMotion>(SLOW_POWER),
		std::make_shared<PilonsCorrection>(SLOW_CORRECTION_COEFF, SLOW_CORRECTION_DIST),
		std::make_shared<SimpleStop>(SLOW_HARSH_THRESHOLD, SLOW_COAST_THRESHOLD, SLOW_COAST_POWER)};

	odom->reset_position();

	/* intake_arm.move_voltage(-12000);  */

	reckless->go(
		RecklessPath()
		.with_segment(PilonsSegment(
		std::make_shared<ConstantMotion>(MID_POWER),
		std::make_shared<PilonsCorrection>(MID_CORRECTION_COEFF, MID_CORRECTION_DIST),
		std::make_shared<SimpleStop>(MID_HARSH_THRESHOLD, 0.25_s, MID_COAST_POWER),		
		{48_in, -2_in, 0_deg}, 0_in)
		)
	);  
	run_until(200000);
	pros::delay(2000);

	reckless->go(RecklessPath()
	.with_segment(RecklessTurnSegment(0.7, 0.3,
		-90_deg, HARSH_COEFF, COAST_COEFF, 0.1_s)));
	run_until(1000);

	pros::delay(20000);
	reckless->go(
		RecklessPath()
		.with_segment(PilonsSegment(
		std::make_shared<ConstantMotion>(MID_POWER),
		std::make_shared<PilonsCorrection>(MID_CORRECTION_COEFF, MID_CORRECTION_DIST),
		std::make_shared<SimpleStop>(MID_HARSH_THRESHOLD, 0.25_s, MID_COAST_POWER),		
		{48_in, -24_in, 0_deg}, 0_in)
		)
	);
	run_until(2000);

	//odom_runner = std::make_shared<rev::AsyncRunner>(odom);
	//reckless_runner = std::make_shared<rev::AsyncRunner>(reckless);

	//auto serialRX = std::make_shared<SerialRX>(receive_data);
	//auto serialRXRunner = std::make_shared<rev::AsyncRunner>(serialRX);

	
}

const unsigned int NUM_FLAGS = 10;
bool flags[NUM_FLAGS] = {false};

bool update_positions = false;
bool received_first_pos = false;
rev::Pose initial_pose;
auto last_time = 0;
int drop_count = 0;
void receive_data(std::string sender, std::string message)
{
	if (sender == "npos")
	{ // if the position is an position update, the update the position in the odometry system
		if (!update_positions)
			return;

		std::stringstream ss(message);
		float x, y, theta;
		char type;
		int elapsed_time;

		ss >> type >> x >> y >> theta >> elapsed_time; // since SLAM only outputs x, y, and theta, we can assume that the data
													   // received is x, y, and theta.

		// we should probably check the time when it actually loaded into the odometry state,
		// since there could be more time before it is actually processed by the odometry system
		if (last_time != 0 && pros::millis() - 50 > last_time + elapsed_time)
		{
			last_time = 0;
			drop_count++;
			return;
		}

		last_time += elapsed_time;

		rev::Pose pos = {x * meter, y * meter, theta * radian};

		if (!received_first_pos)
		{
			auto current_pos = odom->get_state().pos;
			if ((current_pos.x * current_pos.x + current_pos.y * current_pos.y).convert(inch * inch) > 2 || abs(current_pos.theta) > 5 * radian)
			{
				initial_pose = current_pos;
			}
			initial_pose = pos;
			received_first_pos = true;
		}
		else
		{
			if (abs(pos.theta - initial_pose.theta - odom->get_state().pos.theta) > 5_deg)
			{
				drop_count++;
			}

			auto c = std::cos(pos.theta.convert(radian));
			auto s = std::sin(pos.theta.convert(radian));

			auto offsetX = pos.x - initial_pose.x;
			auto offsetY = pos.y - initial_pose.y;

			pos = {c * offsetX - s * offsetY, s * offsetX + c * offsetY, pos.theta - initial_pose.theta};
			odom->set_position(pos);
		}
	}
	else if (sender == "flag")
	{
		if (message.length() < 2)
			return;
		char flag = message[0];			 // first character is the name of the flag
		bool status = message[1] == '1'; // second character is 1 or 0 to set or reset the flag

		if (flag - 'A' < 0 || flag - 'A' >= NUM_FLAGS)
			return;
		flags[flag - 'A'] = status;
	}
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol()
{
	//autonomous(); // remove before comp
	pros::Controller controller(pros::E_CONTROLLER_MASTER);

	auto serialRX = std::make_shared<SerialRX>(receive_data);
	auto serialRXRunner = std::make_shared<rev::AsyncRunner>(serialRX);

	pros::Controller master(pros::E_CONTROLLER_MASTER); // used to get inputs from the users's controller

	update_positions = true;

	auto last_time = std::chrono::high_resolution_clock::now();
	int starting;

	int blue_balls_seen = 0;
	int red_balls_seen = 0;
	double past_red_value = 0;
	double past_blue_value = 0;


	color_sensor_tester.set_led_pwm(90);
	// mogo clamp controls
	bool l1Down = false;
	bool mogoClampDown = false;

	while (true)
	{
		
		starting = -controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);

		auto time = std::chrono::high_resolution_clock::now();

		pros::delay(20);

		update_balls_seen(&blue_balls_seen, &red_balls_seen, &past_red_value, &past_blue_value);
		print_color_sensor(&blue_balls_seen, &red_balls_seen);
		/* pros::lcd::set_text(0, std::to_string(odom->get_state().pos.x.convert(inch)));
		pros::lcd::set_text(1, std::to_string(odom->get_state().pos.y.convert(inch)));
		pros::lcd::set_text(2, std::to_string(odom->get_state().pos.theta.convert(degree)));
		pros::lcd::set_text(3, std::to_string(drop_count));
		pros::lcd::set_text(4, std::to_string(beam_break.get_value())); */
		

		last_time = time;

		// arcade drive
		int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) * exp(-abs(starting) * 0.001);
		if (abs(starting) < 50)
		{
			starting = 0;
		}
		if (abs(turn) < 50)
		{
			turn = 0;
		}

		left_motor_group.move_voltage((int)((double)(starting + turn) / 150 * 12000));
		right_motor_group.move_voltage((int)((double)(starting - turn) / 150 * 12000));

		// intake arm controls

		if (master.get_digital(DIGITAL_L2))
		{
			intake_arm.move_voltage(12000); // raise arm to reach wall stake
		}
		else if (master.get_digital(DIGITAL_L1))
		{
			intake_arm.move_voltage(-12000); // lower arm
		}
		else
		{
			intake_arm.move_voltage(0); // default
		}

		// roller controls

		if (rollers.get_actual_velocity() > 0 && conveyor.get_actual_velocity() == 0)
		{
			if (beam_break.get_value() == 1)
			{ // if the rollers are moving and succesfully intake a ring then start the conveyor
				// conveyor.move_voltage(12000);
			}
			else
			{
				// conveyor.move_voltage(0);
			}
		}

		if (master.get_digital(DIGITAL_X))
		{
			rollers.move_voltage(12000); // intake
		}
		else if (master.get_digital(DIGITAL_Y))
		{
			rollers.move_voltage(-12000); // outtake
		}
		else
		{
			rollers.move_voltage(0); // default
		}

		// conveyor controls

		//conveyor.set_brake_mode(MOTOR_BRAKE_BRAKE); // sets conveyor to brake harsh so rings of the wrong color fly off when the intake brakes

		if (master.get_digital(DIGITAL_R2))
		{
			// if ((color_sensor.get_hue() >= 340 || color_sensor.get_hue() <= 20))
			// {					 // if the color sensor senses a red ring spit it out
			// 	pros::delay(30); // this value can be finetuned if the ring scores on the goal after being slowed down this value likely needs to be increased, if you don't notice a slow at all then the value should be decreased
			// 	conveyor.brake();
			// }
			// else
			// {
			// 	conveyor.move_voltage(-12000); // towards mogo
			// }

			conveyor.move_voltage(-12000);
		}
		else if (master.get_digital(DIGITAL_R1))
		{
			conveyor.move_voltage(12000); // away from mogo
		}
		else
		{
			conveyor.move_voltage(0);
		}

		if (master.get_digital(DIGITAL_A))
		{ // toggle clamp control
			if (!l1Down)
			{									// if the button is pressed
				l1Down = true;					// set boolean value to save that it is pressed
				mogoClampDown = !mogoClampDown; // set boolean value to save that intake is down/up
				if (!mogoClampDown)
				{ // toggle clamp
					mogoClamp.set_value(1);
				}
				else
				{
					mogoClamp.set_value(0);
					controller.rumble(".");

				}
			}
		}
		else
		{
			l1Down = false;
		}

		if (master.get_digital(DIGITAL_UP))
		{
			slides.move_velocity(12000);
			//winch.move_velocity(-12000);

		}
		else if(master.get_digital(DIGITAL_DOWN)){
			slides.move_velocity(-12000);
			//winch.move_velocity(12000);
		}
		else if(master.get_digital(DIGITAL_LEFT)){
			winch.move_velocity(-12000);
		}
		else if(master.get_digital(DIGITAL_RIGHT)){
			winch.move_velocity(12000);
		}
		else{
			slides.move_velocity(0);
			winch.move_velocity(0);
		}
	}
	pros::delay(20);
}

void print_position()
{
	std::string position = "Position: ";
	position += std::to_string(odom->get_state().pos.x.convert(inch));
	position += ", ";
	position += std::to_string(odom->get_state().pos.y.convert(inch));
	position += ", ";
	position += to_string(odom->get_state().pos.theta.convert(degree));

	/* pros::lcd::set_text(0, position); */
	/* pros::lcd::set_text(0, "Printing to LCD: Raw Values");
	pros::lcd::set_text(1, to_string(fwd.get_position()));
	pros::lcd::set_text(2, to_string(lat.get_position()));
	pros::lcd::set_text(3, to_string(imu.get_heading()));
	pros::lcd::set_text(4, to_string(imu.get_pitch()));
	pros::lcd::set_text(5, to_string(imu.get_yaw())); */

	pros::lcd::set_text(0, "Printing to LCD: Odom Values");
	pros::lcd::set_text(1, to_string(odom->get_state().pos.x.convert(inch)));
	pros::lcd::set_text(2, to_string(odom->get_state().pos.y.convert(inch)));
	pros::lcd::set_text(3, to_string(odom->get_state().pos.theta.convert(degree)));
}

void print_color_sensor(int* bbs, int* rbs){
	pros::lcd::set_text(0, "Printing to LCD: Color Sensor Values");
	pros::lcd::set_text(1, to_string(color_sensor_tester.get_hue()));
	pros::lcd::set_text(2, to_string(color_sensor_tester.get_rgb().red));
	pros::lcd::set_text(3, to_string(color_sensor_tester.get_rgb().green));
	pros::lcd::set_text(4, to_string(color_sensor_tester.get_rgb().blue));
	pros::lcd::set_text(5, to_string(*bbs));
	pros::lcd::set_text(6, to_string(*rbs));
	
}

void update_balls_seen(int* bbs, int* rbs, double* lv_r, double* lv_b){
	if(color_sensor_tester.get_rgb().blue - *lv_b > 50){
		(*bbs)++;
	}
	if(color_sensor_tester.get_rgb().red - *lv_r > 50){
		(*rbs)++;
	}
	*lv_r = color_sensor_tester.get_rgb().red;
	*lv_b = color_sensor_tester.get_rgb().blue;
	pros::delay(20);
}