
#pragma once

#include "Representations/Infrastructure/Image.h"
#include "Representations/Configuration/YoloBallModelsDefinitions.h"

class DatasetCreator
{
private:
    unsigned int nb_images_saved = 1;
    const int wait = 4;
    int current_wait = 0;
	const int batch_number_min = 300;
    const int batch_number_max = 399;
    int batch_number;
public:
#ifdef TARGET_ROBOT
    bool save_lower = true;
    bool save_upper = true;
#else
    bool save_lower = false;
    bool save_upper = false;
#endif

    bool save_with_ball_only = false;

    DatasetCreator();
    void update(const Image& image, bool label = false, float x = 0.f, float y = 0.f, float radius = 0.f);
    std::string generate_filepath(bool is_upper) const;
    void saveCurrentImageToFile(std::string filePath, const Image& image) const;
    void saveLabelToFile(std::string filePath, float x, float y, float radius) const;
};
