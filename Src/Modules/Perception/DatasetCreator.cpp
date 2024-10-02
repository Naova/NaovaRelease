#include "DatasetCreator.h"
#include "Platform/File.h"
#include "Tools/Module/Module.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <cstdlib>
#include <ctime>
#include "fpng.h"

DatasetCreator::DatasetCreator() {
	fpng::fpng_init();
	std::srand(static_cast<unsigned int>(std::time(nullptr)));
	batch_number = batch_number_min + std::rand() % (batch_number_max - batch_number_min);
}

std::string DatasetCreator::generate_filepath(bool is_upper) const
{
	//zero padding
	std::stringstream ss;
	ss << batch_number;
	std::string batch;
	ss >> batch;
	while (batch.length() < 2) {
		batch = '0' + batch;
	}
	ss.clear();
	ss << nb_images_saved;
	std::string nb_images;
	ss >> nb_images;
	while (nb_images.length() < 4) {
		nb_images = '0' + nb_images;
	}

	std::ostringstream os;
#ifdef TARGET_ROBOT
	std::string filePath = "/var/volatile";
#else
	std::string filePath = File::getBHDir();
#endif
	//create folders if they don't exist
	std::string s = "mkdir -p " + filePath + "/Dataset/upper/YCbCr/batch_" + batch +
					" " + filePath + "/Dataset/lower/YCbCr/batch_" + batch;
	std::system(s.c_str());

	std::string which_camera = "upper";
	if (!is_upper) which_camera = "lower";
	filePath += "/Dataset/" + which_camera + "/YCbCr/batch_" + batch + "/batch_";
	filePath += batch + "_image_" + nb_images + ".png";
	//std::cout << filePath << std::endl;
	return filePath;
}

void DatasetCreator::update(const CameraImage& image, bool label, float x, float y, float radius)
{
	bool is_upper = image.width == 320;

	if(nb_images_saved > 5000)
		return;

	if (!label && save_with_ball_only) {
		return;
	}
	
	if ((is_upper && !save_upper) || (!is_upper && !save_lower))
		return;
	
	if (current_wait < wait) {
		++current_wait;
		return;
	} else {
		current_wait = 0;
	}
	
	std::string filePath = generate_filepath(is_upper);
	saveCurrentImageToFile(filePath, image);
	if (label)
		saveLabelToFile(filePath + "_label", x, y, radius);

	nb_images_saved += 1;
}

void DatasetCreator::saveCurrentImageToFile(std::string filePath, const CameraImage& image) const
{
	uint8_t * image_array;
	CameraImage img;
	if (image.width == 320)
		image.getResizedImage(320,240,img);
	else 
		image.getResizedImage(160,120,img);
	image_array = new uint8_t[img.height * img.width* YOLO_BALL_INPUT_CHANNELS];
	img.convertToFPNGformat(image_array);

	fpng::fpng_encode_image_to_file(filePath.c_str(), image_array, img.width, img.height, 3);
	
	delete image_array;
}

void DatasetCreator::saveLabelToFile(std::string filePath, float x, float y, float radius) const
{
	float array[3];
	array[0] = x;
	array[1] = y;
	array[2] = radius;
	std::fstream out_file = std::fstream(filePath.c_str(), std::ios::out | std::ios::binary);
	out_file.write(reinterpret_cast<char *>(array), 3 * sizeof(float));
	out_file.close();
}