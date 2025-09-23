#ifndef SERIAL_RECEIVER_HH
#define SERIAL_RECEVIER_HH

#include "rev/api/async/async_runner.hh"

#include <bits/stdc++.h>
#include <string>
#include <Tuple>

using namespace rev;

class SerialRX : public AsyncRunnable {
private:
    std::function<void(std::string, std::string)> receiver_callback;
    std::string read_buffer;

public:
    SerialRX(std::function<void(std::string, std::string)> receiver_callback);

    bool parse(std::string data, std::tuple<std::string, std::string>& parsed_message);
    void step() override;
};

#endif