
#pragma once

#include "Representations/Infrastructure/CameraImage.h"
#include "Representations/Configuration/YoloModelDefinitions.h"

class DatasetCreator
{
private:
    unsigned int nb_images_saved = 1;
    const int wait = 4;
    int current_wait = 0;
	const int batch_number_min = 200;
    const int batch_number_max = 299;
    int batch_number;
public:
#ifdef TARGET_ROBOT
    bool save_lower = false;
    bool save_upper = false;
#else
    bool save_lower = false;
    bool save_upper = false;
#endif

    bool save_with_ball_only = false;

    DatasetCreator();
    void update(const CameraImage& image, bool label = false, float x = 0.f, float y = 0.f, float radius = 0.f);
    std::string generate_filepath(bool is_upper) const;
    void saveCurrentImageToFile(std::string filePath, const CameraImage& image) const;
    void saveLabelToFile(std::string filePath, float x, float y, float radius) const;
};