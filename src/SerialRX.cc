#include "SerialRX.hh"
#include <sys/unistd.h>
#include <iostream>
#include <string>

#include "pros/apix.h"

SerialRX::SerialRX(std::function<void(std::string, std::string)> receiver_callback) : AsyncRunnable() {
    this->receiver_callback = receiver_callback;
}

void SerialRX::step() {
    std::getline(std::cin, read_buffer);

    std::tuple<std::string, std::string> parsed_message;
    if (parse(read_buffer, parsed_message)) receiver_callback(std::get<0>(parsed_message), std::get<1>(parsed_message));
}

bool SerialRX::parse(std::string data, std::tuple<std::string, std::string>& parsed_message) {
    if (data.length() == 0) {
        return false;
    }

    const char* key_start = nullptr; // <*ey>value
    const char* key_end   = nullptr;   // <key*value

    for (unsigned int i = 0; i < data.length(); i++) {
        if (data[i] == '<') {
            if (key_start != nullptr) {
                return false;
            }
            key_start = &data[i+1];
        }
        if (data[i] == '>') {
            if (key_start == nullptr || key_end != nullptr) {
                return false;
            }
            key_end = &data[i];
        }
    }

    if (key_start == nullptr || key_end == nullptr) {
        return false;
    }

    std::string key(key_start, key_end - key_start);

    std::string* key_copy = new std::string(key);
    std::string* value_copy = new std::string(data.substr(key_end - &data[0] + 1));

    parsed_message = std::make_tuple(key, data.substr(key_end - &data[0] + 1));

    return true;
}