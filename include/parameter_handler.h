#pragma once

#include <iostream>
#include <string>
#include "../src/objectdetection.h"

class ParameterHandler
{
private:
    std::vector<tuple<string, bool>> execution_params;
    std::vector<tuple<string, bool>> filter;
    std::vector<float> keypointdetector_threshold = { 0.7f };
    std::vector<int> keypointdetector_nof_neighbors = { 5 };
    bool background_removal = false;
    bool detection_stats = false;
    bool match_retrieval = false;
    bool cloudgen_stats = false;
    bool detection_log = false;
    bool pose_estimation = false;
    bool filtering = false;
    bool transformation_matrices_based_estimation = false;
    bool iss_frame_based_estimation = false;
    bool ransac = true;
    bool visu = false;
    int ne_scalefactor = 5;
    float matcher_distance_threshold = 0.95f;
    float background_removal_threshold = 0.005f;
    int detection_threshold = 0;
    std::string object = "mutter_sequence_easy";
    std::string dataset = "object_threshold_eval";
    std::string preprocessor_mode = "3d_filtered";
public:
    ParameterHandler()
    {
        init_parameter_handler();
    }

    void init_parameter_handler() {
        init_exec_params();
        init_filter();

    }

    void init_exec_params() {
        execution_params.push_back(make_tuple("cloudcreation", false));
        execution_params.push_back(make_tuple("merging", false));
        execution_params.push_back(make_tuple("detection", false));
        execution_params.push_back(make_tuple("processing", false));
    }

    void init_filter() {
        filter.push_back(make_tuple("median", false));
        filter.push_back(make_tuple("roi", false));
        filter.push_back(make_tuple("sor", false));
    }

    bool toggle_filter(string id) {
        for (int i = 0; i < filter.size(); ++i) {
            if (id.compare(get<0>(filter[i])) == 0) {
                get<1>(filter[i]) = !(get<1>(filter[i]));
                return (get<1>(filter[i]));
            }
        }
        return false;
    }

    bool toggle_filtering_state() {
        filtering = !filtering;
        return filtering;
    }

    bool toggle_cloudgen_stats() {
        cloudgen_stats = !cloudgen_stats;
        return cloudgen_stats;
    }

    bool toggle_ransac() {
        ransac = !ransac;
        return ransac;
    }

    bool toggle_detection_stats() {
        detection_stats = !detection_stats;
        return detection_stats;
    }

    bool toggle_detection_logging() {
        detection_log = !detection_log;
        return detection_log;
    }

    bool toggle_match_retrieval() {
        match_retrieval = !match_retrieval;
        return match_retrieval;
    }

    bool toggle_background_removal() {
        background_removal = !background_removal;
        return background_removal;
    }

    bool toggle_visualization() {
        visu = !visu;
        return visu;
    }

    bool toggle_pose_estimation() {
        pose_estimation = !pose_estimation;
        return pose_estimation;
    }

    bool toggle_iss_frame_based_pose_estimation() {
        iss_frame_based_estimation = !iss_frame_based_estimation;
        return iss_frame_based_estimation;
    }

    bool toggle_transformation_matrices_based_pose_estimation() {
        transformation_matrices_based_estimation = !transformation_matrices_based_estimation;
        return transformation_matrices_based_estimation;
    }

     void set_execution_param(string id) {
        for (int i = 0; i < execution_params.size(); ++i) {
            if (id.compare(get<0>(execution_params[i])) == 0) {
                get<1>(execution_params[i]) = true;
            }
        }
    }

     void set_ne_scalefactor(int factor) {
         ne_scalefactor = factor;
     }

     void set_detection_threshold(int threshold) {
         detection_threshold = threshold;
     }

     void set_background_removal_threshold(float threshold) {
         background_removal_threshold = threshold;
     }

     void set_matcher_distance_threshold(float threshold) {
         matcher_distance_threshold = threshold;
     }

     void set_dataset(string dataset_name) {
         dataset = dataset_name;
     }

     void set_preprocessor_mode(string preprocessor) {
         preprocessor_mode = preprocessor;
     }

     void set_object(string object_name) {
         object = object_name;
     }

     void add_detector_threshold(float threshold) {
         keypointdetector_threshold.push_back(threshold);
     }

     void add_detector_nn(int neighbor) {
         keypointdetector_nof_neighbors.push_back(neighbor);
     }

     bool get_filter_state(string id) {
         for (int i = 0; i < filter.size(); ++i) {
             if (id.compare(get<0>(filter[i])) == 0) {
                 return (get<1>(filter[i]));
             }
         }
         return false;
     }

    bool get_exec_param_state(string id) {
        for (int i = 0; i < execution_params.size(); ++i) {
            if (id.compare(get<0>(execution_params[i])) == 0) {
                return (get<1>(execution_params[i]));
            }
        }
        return false;
    }

    bool get_cloudgen_stats_state() {
        return cloudgen_stats;
    }

    bool get_ransac_state() {
        return ransac;
    }

    bool get_filtering_state() {
        return filtering;
    }

    bool get_detection_stats_state() {
        return detection_stats;
    }

    bool get_detection_logging_state() {
        return detection_log;
    }

    bool get_match_retrieval_state() {
        return match_retrieval;
    }

    bool get_background_removal_state() {
        return background_removal;
    }

    bool get_visualization_state() {
        return visu;
    }

    bool get_pose_estimation_state() {
        return pose_estimation;
    }

    bool get_iss_frame_based_pose_estimation_state() {
        return iss_frame_based_estimation;
    }

    bool get_transformation_matrices_based_pose_estimation_state() {
        return transformation_matrices_based_estimation;
    }

    int get_ne_scalefactor() {
        return ne_scalefactor;
    }

    int get_detection_threshold() {
        return detection_threshold;
    }

    float get_background_removal_threshold() {
        return background_removal_threshold;
    }

    float get_matcher_distance_threshold() {
        return matcher_distance_threshold;
    }

    std::string get_dataset() {
        return dataset;
    }

    std::string get_preprocessor_mode() {
        return preprocessor_mode;
    }

    std::string get_object() {
        return object;
    }

    float get_detector_threshold_at(int pos) {
        return keypointdetector_threshold[pos];
    }

    int get_detector_nn_at(int pos) {
        return keypointdetector_nof_neighbors[pos];
    }

    int sizeof_detector_threshold() {
        return keypointdetector_threshold.size();
    }

    int sizeof_detector_nn() {
        return keypointdetector_nof_neighbors.size();
    }

};
