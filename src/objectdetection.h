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
void set_dataset(std:: string);
void set_object(std::string);
bool toggle_filter(std::string);
bool toggle_cloudgen_stats(void);
//Detection
constexpr auto RANSAC_MIN_MATCHES = (3);
void add_detector_threshold(float);
void add_detector_nn(int);
void set_ne_scalefactor(int);
void set_preprocessor(string);
void set_matcher_distance_threshold(float);
void set_detection_threshold(int);
bool toggle_detection_stats(void);
bool toggle_visualization(void);
bool toggle_detection_logging(void);
bool toggle_match_retrieval(void);
bool toggle_ransac(void);
//Learning
void set_query_learning(void);
//Processing
void set_background_removal_threshold(float);
bool toggle_background_removal(void);

