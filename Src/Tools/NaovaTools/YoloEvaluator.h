#include <string>
#include "Representations/Configuration/YoloBallModelsDefinitions.h"

#ifdef TARGET_ROBOT
#include "cnn_yolo_modele_robot_lower.h"
#include "cnn_yolo_modele_robot_upper.h"
#else
#include "cnn_yolo_modele_simulation_lower.h"
#include "cnn_yolo_modele_simulation_upper.h"
#endif
class YoloEvaluator {
public:
    YoloEvaluator(const bool is_upper);

    /**
     * retourne les coordonnees du centre de la balle sur l'image
     * @param image l'image dans laquelle on cherche
     * */
    void detectBallOnImage(const float * image_array, float* model_output);

protected:
    void (*function)(const float*, float*);
};
