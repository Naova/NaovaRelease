#include "YoloPlayerDetector.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/Stopwatch.h"
#include "Tools/NaovaTools/cnn_modele_robots_robot_upper.h"
#include "Modules/Perception/PlayersPerceptors/PlayersPerceptor.h"
#include <iostream>
#include <utility>


void YoloPlayerDetector::searchPlayersOnImage(const Image &image, std::vector<PlayersImagePercept::PlayerInImage>& players, int horizon, bool is_upper) {
  float * image_array = new float[YOLO_PLAYER_INPUT_SIZE];
  
  if(YOLO_PLAYER_INPUT_HEIGHT != image.height || YOLO_PLAYER_INPUT_WIDTH != image.width) {
    Image img(false, YOLO_PLAYER_INPUT_WIDTH, YOLO_PLAYER_INPUT_HEIGHT);
    image.getResizedImage(YOLO_PLAYER_INPUT_WIDTH, YOLO_PLAYER_INPUT_HEIGHT, img);
    img.convertToYoloFormat(image_array, true);
  }
  else {
    image.convertToYoloFormat(image_array, true);
  }

  float * model_output = new float[YOLO_PLAYER_OUTPUT_SIZE];

  cnnmodele_robots_robot_upper(image_array, model_output);
  read_model_output(model_output, players, horizon, is_upper);
  

  delete[] image_array;
  delete[] model_output;

}


float YoloPlayerDetector::get_model_output_value(float* model_output, uint8_t i, uint8_t j, uint8_t k) {
  return model_output[
    i * YOLO_PLAYER_OUTPUT_HEIGHT_STEP +
    j * YOLO_PLAYER_OUTPUT_CHANNELS +
    k
  ];
}
int YoloPlayerDetector::get_final_coord_x(float* model_output, uint8_t i, uint8_t j, bool is_upper) {
  float x1 = get_model_output_value(model_output, i, j, 1);
  float x2 = get_model_output_value(model_output, i, j, 2);
  int width = (is_upper ? 640 : 320);
  if(x1 > x2)
    return static_cast<int>((j + 0.25f) * width / YOLO_PLAYER_OUTPUT_WIDTH);
  else
    return static_cast<int>((j + 0.75f) * width / YOLO_PLAYER_OUTPUT_WIDTH);
}
int YoloPlayerDetector::get_final_coord_y(float* model_output, uint8_t i, uint8_t j, bool is_upper) {
  float y1 = get_model_output_value(model_output, i, j, 3);
  float y2 = get_model_output_value(model_output, i, j, 4);
  int height = (is_upper ? 480 : 240);
  if(y1 > y2)
    return static_cast<int>((i + 0.25f) * height / YOLO_PLAYER_OUTPUT_HEIGHT);
  else
    return static_cast<int>((i + 0.75f) * height / YOLO_PLAYER_OUTPUT_HEIGHT);
}

void YoloPlayerDetector::limit_box_borders(PlayersImagePercept::PlayerInImage &p, bool is_upper) {
  if(p.x1 < 1) p.x1 = 1;
  if(p.y1 < 1) p.y1 = 1;
  if(p.x2 > (is_upper ? 639 : 319)) p.x2 = (is_upper ? 639 : 319);
  if(p.y2 > (is_upper ? 479 : 239)) p.y2 = (is_upper ? 479 : 239);
}

void YoloPlayerDetector::read_model_output(float* model_output, std::vector<PlayersImagePercept::PlayerInImage>& players, int horizon, bool is_upper) {
  for(uint8_t i = 0; i < YOLO_PLAYER_OUTPUT_HEIGHT; ++i) {
    for(uint8_t j = 0; j < YOLO_PLAYER_OUTPUT_WIDTH; ++j) {
      float confidence = model_output[i * YOLO_PLAYER_OUTPUT_HEIGHT_STEP + j * YOLO_PLAYER_OUTPUT_CHANNELS];
      if (is_under_horizon(horizon, model_output, i, j, is_upper)) {
        if((confidence > minConfidenceUpper && is_upper) || (confidence > minConfidenceLower && !is_upper)) {
          int player_center_x = get_final_coord_x(model_output, i, j, is_upper);
          int player_center_y = get_final_coord_y(model_output, i, j, is_upper);

          //extract player size from the model output
          int highest_k = 0;
          float highest_value = 0;
          for(int k = 0; k < 3; ++k) {
            float value = model_output[
              i * YOLO_PLAYER_OUTPUT_HEIGHT_STEP +
              j * YOLO_PLAYER_OUTPUT_CHANNELS +
              5 + k
            ];
            if(value > highest_value) {
              highest_k = k;
              highest_value = value;
            }
          }
          //Height and width of players using the anchors 
          int player_width = YOLO_PLAYER_ANCHORS_WIDTH(highest_k);
          
          //extract player size from the model output
          highest_k = 0;
          highest_value = 0;
          for(int k = 0; k < 3; ++k) {
            float value = model_output[
              i * YOLO_PLAYER_OUTPUT_HEIGHT_STEP +
              j * YOLO_PLAYER_OUTPUT_CHANNELS +
              8 + k
            ];
            if(value > highest_value) {
              highest_k = k;
              highest_value = value;
            }
          }
          
          int player_height = YOLO_PLAYER_ANCHORS_HEIGHT(highest_k);
          
          //Players coordinates
          int player_x1 = player_center_x - player_width / 2;
          int player_x2 = player_center_x + player_width / 2;
          int player_y2 = player_center_y;
          int player_y1 = player_y2 - player_height; //approximation tiree de l'ancien detecteur
          
          // Updating players coordinates
          PlayersImagePercept::PlayerInImage player;
          player.x1 = player_x1;
          player.x2 = player_x2;
          player.y1 = player_y1;
          player.y2 = player_y2;
          limit_box_borders(player, is_upper);
          if(player.x2 == player.x1 || player.y1 == player.y2) {
            std::cout << "ERREUR: boite de dimension 0" << std::endl;
            continue;
          } 
          player.realCenterX = player_x1 + (player_x2 - player_x1) / 2;
          player.x1FeetOnly = player_x1;
          player.x2FeetOnly = player_x2;
          player.lowerCamera = !is_upper;
          player.detectedJersey = false;
          player.detectedFeet = true;
          player.ownTeam = false;
          player.fallen = false;
          player.confidence = confidence;
          
          players.push_back(player);
        }
      }
    }
  }
}

bool YoloPlayerDetector::is_under_horizon(int horizon, float* model_output, uint8_t x, uint8_t y, bool is_upper) {
  int j = get_final_coord_y(model_output, x, y, is_upper);

  return j > horizon;
}
