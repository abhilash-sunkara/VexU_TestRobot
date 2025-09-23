#include <iostream>
#include <chrono>

#include "rev/rev.hh"
#include "SerialTX.hh"
#include "globals.hh"


void SerialTX::step() {
    using namespace rev;
    using namespace std::chrono;
    auto state = odom->get_state();
    std::cout << "<pos>e " << state.pos.x.convert(inch) << " " << state.pos.y.convert(inch) << " " << state.pos.theta.convert(radian) << " " << state.vel.xv.convert(meter/second) << " " << state.vel.yv.convert(meter/second) << " " << state.vel.angular.convert(radian/second) << " " << imu.get_accel().x << " " << imu.get_accel().y <<   " "  << duration_cast<microseconds>(steady_clock::now() - last_time).count() << "\n";
//	if (buffer[0] == 'e') ss >> location.x >> location.y >> location.theta >> location.vx >> location.vy >> location.vtheta >> location.ax >> location.ay;

    last_time = steady_clock::now();
}