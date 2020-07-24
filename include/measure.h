#pragma once

#include <iostream>
#include <string>
#include "../src/objectdetection.h"

class Measure
{
private:
    std::clock_t start, end_time;
    bool running = false;
    vector<tuple<string, float>> raw_stats;
    std::string stats;
public:
    Measure()
    {
        
    }
    void time_meas() {
        double cpuTimeUsed;
        if (!running) {
            running = true;
            start = clock();
        }
        else {
            end_time = clock();
            running = false;
            cpuTimeUsed = ((double)(end_time - start)) / CLOCKS_PER_SEC;
            std::cout << "Time taken: " << (double)cpuTimeUsed << std::endl;
        }
    }
    void time_meas(string action) {
        double cpuTimeUsed;
        if (!running) {
            running = true;
            start = clock();
        }
        else {
            end_time = clock();
            running = false;
            cpuTimeUsed = ((double)end_time - (double)start) / CLOCKS_PER_SEC;
            std::cout << "Time taken for: " + action + " " << (double)cpuTimeUsed << std::endl;
        }
        raw_stats.push_back(make_tuple(action, cpuTimeUsed));
    }

    void create_writable_stats() {
        std::string data = "";
        for (int i = 0; i < raw_stats.size(); ++i) {
            data += get<0>(raw_stats[i]) + ',' + std::to_string(get<1>(raw_stats[i]));
            data += "\n";
        }
        stats = data;
    }

    std::string get_stats() {
        create_writable_stats();
        return stats;
    }

    void clear_stats() {
        raw_stats.clear();
        stats.clear();
    }
   
};
