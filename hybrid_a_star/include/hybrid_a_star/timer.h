

#ifndef HYBRID_A_STAR_TIMER_H
#define HYBRID_A_STAR_TIMER_H

#include <string>
#include <iostream>
#include <chrono>

class Timer {
public:
    Timer() {
        Begin();
    }

    void Begin() {
        start_ = std::chrono::system_clock::now();
    }

    void End(const std::string &task_name) {
        end_ = std::chrono::system_clock::now();
        std::chrono::duration<double,std::milli> use_time = end_ - start_;

        std::cout.precision(3);
        std::cout << task_name << " use time(ms): " << use_time.count()  << std::endl;
    }

    double End() {
        end_ = std::chrono::system_clock::now();
        std::chrono::duration<double,std::milli> use_time = end_ - start_;
        return use_time.count() ;
    }

private:
    std::chrono::time_point<std::chrono::system_clock> start_, end_;
};


#endif //HYBRID_A_STAR_TIMER_H
