#pragma once

#include "Representations/Perception/PlayersPercepts/PlayersImagePercept.h"
#include "Representations/Configuration/YoloPlayerModelsDefinitions.h"
#include "Representations/Infrastructure/Image.h"

#include "Tools/Math/Geometry.h"

class YoloPlayerDetector
{
public:
    YoloPlayerDetector(float _minConfidenceUpper, float _minConfidenceLower) {
        minConfidenceUpper = _minConfidenceUpper;
        minConfidenceLower = _minConfidenceLower;
    }

    //! return int value for ballpercept status
    //! initialize \c rect if there is a ball on the image
    void searchPlayersOnImage(const Image &image, std::vector<PlayersImagePercept::PlayerInImage>& players, int horizon, bool is_upper);

private:
   void read_model_output(float* model_output, std::vector<PlayersImagePercept::PlayerInImage>& players, int horizon, bool is_upper);
    
    //There shouldn't be any ball above horizon (until we have a better kick?)
    //Discard every above-horizon prediction.
    void limit_box_borders(PlayersImagePercept::PlayerInImage &p, bool is_upper);
    bool is_under_horizon(int horizon, float* model_output, uint8_t x, uint8_t y, bool is_upper);
    float get_model_output_value(float* model_output, uint8_t i, uint8_t j, uint8_t k);
    int get_final_coord_x(float* model_output, uint8_t i, uint8_t j, bool is_upper);
    int get_final_coord_y(float* model_output, uint8_t i, uint8_t j, bool is_upper);

    float minConfidenceUpper;
    float minConfidenceLower; 
};
