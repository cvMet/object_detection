// objectdetection.h : Include file for standard system include files,
// or project specific include files.

#pragma once

#include <iostream>

void time_meas(void);
void set_dataset(std:: string);
void set_object(std::string);
std::string get_input(void);
bool get_path(void);
void set_execution_param(std::string);
bool toggle_filter(std::string);
bool toggle_cloudgen_stats(void);
// TODO: Reference additional headers your program requires here.
