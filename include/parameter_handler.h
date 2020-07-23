#pragma once

#include <iostream>
#include <string>
#include "../src/objectdetection.h"

class ParameterHandler
{
private:
    std::vector<tuple<string, bool>> execution_params;
public:
    ParameterHandler()
    {
        init_parameter_handler();
    }

    void init_parameter_handler() {
        init_exec_params();

    }

    void init_exec_params() {
        execution_params.push_back(make_tuple("cloudcreation", false));
        execution_params.push_back(make_tuple("merging", false));
        execution_params.push_back(make_tuple("detection", false));
        execution_params.push_back(make_tuple("processing", false));
    }

     void set_execution_param(string id) {
        for (int i = 0; i < execution_params.size(); ++i) {
            if (id.compare(get<0>(execution_params[i])) == 0) {
                get<1>(execution_params[i]) = true;
            }
        }
    }

    bool get_exec_param_state(string id) {
        for (int i = 0; i < execution_params.size(); ++i) {
            if (id.compare(get<0>(execution_params[i])) == 0) {
                return (get<1>(execution_params[i]));
            }
        }
        return false;
    }
};
