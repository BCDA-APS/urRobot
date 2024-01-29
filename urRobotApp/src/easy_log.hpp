#ifndef _EASY_LOG_HPP_
#define _EASY_LOG_HPP_

#include <iostream>
#include <map>
#include <sstream>
#include <string>

namespace easy_log {

enum Color { RED, GREEN, BLUE, YELLOW, RESET };

std::string stylize(std::string s, Color color) {
    std::map<Color, std::string> color_map = {
        {RED, "\x1b[31m"},    {GREEN, "\x1B[32m"}, {BLUE, "\x1B[34m"},
        {YELLOW, "\x1B[33m"}, {RESET, "\x1b[0m"},
    };

    std::stringstream ss;
    ss << color_map[color];
    ss << s;
    ss << color_map[Color::RESET];
    return ss.str();
}

void log_success(std::string s) {
    std::string string_out = stylize(s, GREEN);
    std::cout << string_out << std::endl;
}

void log_info(std::string s) {
    std::cout << s << std::endl;
}

void log_error(std::string s) {
    std::string string_out = stylize(s, RED);
    std::cout << string_out << std::endl;
}

void log_warn(std::string s) {
    std::string string_out = stylize(s, YELLOW);
    std::cout << string_out << std::endl;
}

} // namespace easy_log

#endif
