// objectdetection.h : Include file for standard system include files,
// or project specific include files.

#pragma once

#include <iostream>
//General
void time_meas(void);
std::string get_input(void);
bool get_path(void);
//void set_execution_param(std::string);
//Cloudgen

//Detection
constexpr auto RANSAC_MIN_MATCHES = (3);
//Learning
void set_query_learning(void);
//Processing

