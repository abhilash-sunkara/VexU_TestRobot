#ifndef SERIAL_TX_HH
#define SERIAL_TX_HH

#include "rev/rev.hh"

class SerialTX : public rev::AsyncRunnable {
private:
    std::shared_ptr<rev::Odometry> odom;
    std::chrono::steady_clock::time_point last_time;
public:
    SerialTX(std::shared_ptr<rev::Odometry> odom) : odom(odom) {  }
    void step() override;
};

#endif