#include "Representations/Perception/BallPercepts/BallPercept.h"
#include "Representations/Configuration/YoloModelDefinitions.h"
#include "Representations/Infrastructure/CameraImage.h"
#include "Tools/NaovaTools/YoloEvaluator.h"

#include "Tools/Math/Geometry.h"

class YoloBallDetector
{
public:
    YoloBallDetector(float lowerGuessedTreshold,
                    float lowerDetectionTreshold,
                    float upperGuessedTreshold,
                    float upperDetectionTreshold);

    bool print_confidence_upper = false;
    bool print_confidence_lower = false;

    float lowerGuessedTreshold;
    float lowerDetectionTreshold;
    float upperGuessedTreshold;
    float upperDetectionTreshold;

    //Evaluateur Yolo
    YoloEvaluator cnnEvaluatorUpper, cnnEvaluatorLower;

    //! initialize \c rect if there is a ball on the image
    BallPercept::Status searchBallOnImage(const CameraImage &image, float* const rect, bool is_upper, Geometry::Line horizon);

private:
    BallPercept::Status read_model_output(float* model_output, float* const rect, bool is_upper, Geometry::Line horizon);
    
    //There shouldn't be any ball above horizon (until we have a better kick?)
    //Discard every above-horizon prediction.
    bool is_under_horizon(Geometry::Line horizon, float* model_output, uint8_t x, uint8_t y, bool is_upper);
    float get_model_output_value(float* model_output, uint8_t i, uint8_t j, uint8_t k, bool is_upper);
    float get_final_coord_x(float* model_output, uint8_t i, uint8_t j, bool is_upper);
    float get_final_coord_y(float* model_output, uint8_t i, uint8_t j, bool is_upper);
    float get_min_guessed_confidence(bool is_upper) {return is_upper ? upperGuessedTreshold : lowerGuessedTreshold;}
    float get_min_confidence(bool is_upper) {return is_upper ? upperDetectionTreshold : lowerDetectionTreshold;}
};