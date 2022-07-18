#ifndef UTIL_TIME_H
#define UTIL_TIME_H

#include<chrono>
#include<string>

using namespace std::chrono;
using std::string;

class UtilTime
{
public:
    void set_start_time();
    void set_end_time();
    void show_time_span(string oper);
private:
    steady_clock::time_point start_time;
    steady_clock::time_point end_time;
};
#endif