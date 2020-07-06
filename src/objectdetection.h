// objectdetection.h : Include file for standard system include files,
// or project specific include files.

#pragma once

#include <iostream>
//General
void time_meas(void);
std::string get_input(void);
bool get_path(void);
void set_execution_param(std::string);
//Cloudgen
void set_dataset(std:: string);
void set_object(std::string);
bool toggle_filter(std::string);
bool toggle_cloudgen_stats(void);
//Detection
void add_detector_threshold(float);
void add_detector_nn(int);
void set_preprocessor(string);
bool toggle_detection_stats(void);
bool toggle_visualization(void);
void set_detection_threshold(int);
bool toggle_detection_logging(void);