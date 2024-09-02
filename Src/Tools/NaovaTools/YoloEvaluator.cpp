#include "YoloEvaluator.h"
#include "Platform/File.h"
#include "Representations/Infrastructure/CameraImage.h"
#include <vector>
#include <iostream>


YoloEvaluator::YoloEvaluator(const bool is_upper)
{
#ifdef TARGET_ROBOT
    if(is_upper) {
        function = &cnnmodele_balles_robot_upper;
    }
    else {
        function = &cnnmodele_balles_robot_lower;
    }
#else
    if(is_upper) {
        function = &cnnmodele_balles_simulation_upper;
    }
    else {
        function = &cnnmodele_balles_simulation_lower;
    }
#endif
}

void YoloEvaluator::detectBallOnImage(const float *image_array, float* model_output) {
    function(image_array, model_output);
}