#include "YoloBallDetector.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/Stopwatch.h"
#include <iostream>

YoloBallDetector::YoloBallDetector(float lowerGuessedTreshold,
                    float lowerDetectionTreshold,
                    float upperGuessedTreshold,
                    float upperDetectionTreshold)
: cnnEvaluatorUpper{YoloEvaluator(true)},
  cnnEvaluatorLower{YoloEvaluator(false)},
  lowerGuessedTreshold{lowerGuessedTreshold},
  lowerDetectionTreshold{lowerDetectionTreshold},
  upperGuessedTreshold{upperGuessedTreshold},
  upperDetectionTreshold{upperDetectionTreshold}
{}

BallPercept::Status YoloBallDetector::searchBallOnImage(const Image &image, float* const detected_ball, bool is_upper, Geometry::Line horizon) {
  float * image_array = new float[get_yolo_ball_input_size(is_upper)];
  
  if(get_yolo_ball_input_height(is_upper) != image.height || get_yolo_ball_input_width(is_upper) != image.width) {
    Image img(false, get_yolo_ball_input_width(is_upper), get_yolo_ball_input_height(is_upper));
    image.getResizedImage(get_yolo_ball_input_width(is_upper), get_yolo_ball_input_height(is_upper), img);
    img.convertToYoloFormat(image_array);
  }
  else {
    image.convertToYoloFormat(image_array);
  }

  float * model_output = new float[get_yolo_ball_input_size(is_upper)];

  if (is_upper) {
    STOPWATCH("BallDetectorUpper") {
      cnnEvaluatorUpper.detectBallOnImage(image_array, model_output);
    }
  }
  else {
    STOPWATCH("BallDetectorLower") {
      cnnEvaluatorLower.detectBallOnImage(image_array, model_output);
    }
  }
  
  BallPercept::Status res = read_model_output(model_output, detected_ball, is_upper, horizon);

  delete[] image_array;
  delete[] model_output;

  return res;
}


float YoloBallDetector::get_model_output_value(float* model_output, uint8_t i, uint8_t j, uint8_t k, bool is_upper) {
  return model_output[
    i * get_yolo_ball_output_height_step(is_upper) +
    j * get_yolo_ball_output_channels(is_upper) +
    k
  ];
}
float YoloBallDetector::get_final_coord_x(float* model_output, uint8_t i, uint8_t j, bool is_upper) {
  float x1 = get_model_output_value(model_output, i, j, 1, is_upper);
  float x2 = get_model_output_value(model_output, i, j, 2, is_upper);
  if(x1 > x2)
    return (j + 0.25f) / get_yolo_ball_output_width(is_upper);
  else
    return (j + 0.75f) / get_yolo_ball_output_width(is_upper);
}
float YoloBallDetector::get_final_coord_y(float* model_output, uint8_t i, uint8_t j, bool is_upper) {
  float y1 = get_model_output_value(model_output, i, j, 3, is_upper);
  float y2 = get_model_output_value(model_output, i, j, 4, is_upper);
  if(y1 > y2)
    return (i + 0.25f) / get_yolo_ball_output_height(is_upper);
  else
    return (i + 0.75f) / get_yolo_ball_output_height(is_upper);
}

BallPercept::Status YoloBallDetector::read_model_output(float* model_output, float* const detected_ball, bool is_upper, Geometry::Line horizon) {
  float highest_confidence = 0.f;
  uint8_t highest_i, highest_j;
  uint8_t j;
  for(uint8_t i = 0; i < get_yolo_ball_output_height(is_upper); ++i) {
    for(j = 0; j < get_yolo_ball_output_width(is_upper); ++j) {
      float confidence = model_output[i * get_yolo_ball_output_height_step(is_upper) + j * get_yolo_ball_output_channels(is_upper)];
      if (confidence > highest_confidence) {
        if (is_under_horizon(horizon, model_output, i, j, is_upper)) {
          highest_confidence = confidence;
          highest_i = i;
          highest_j = j;
        }
      }
    }
  }
  
  if ((print_confidence_upper && is_upper) || (print_confidence_lower && !is_upper)) {
    OUTPUT_TEXT("Confidence : " + std::to_string(highest_confidence * 100) + "% Is Upper : " + std::to_string(is_upper));
  }
  if(highest_confidence > get_min_ball_guessed_confidence(is_upper)) {
    detected_ball[0] = get_final_coord_x(model_output, highest_i, highest_j, is_upper);
    detected_ball[1] = get_final_coord_y(model_output, highest_i, highest_j, is_upper);
    
    //extract ball size from the model output
    int highest_k = 0;
    float highest_value = 0.f;
    for(int k = 0; k < get_yolo_ball_output_anchors_number(is_upper); ++k) {
      float value = model_output[
        highest_i * get_yolo_ball_output_height_step(is_upper) +
        highest_j * get_yolo_ball_output_channels(is_upper) +
        5 + k
      ];
      if(value >= highest_value) {
        highest_k = k;
        highest_value = value;
      }
    }
    detected_ball[2] = get_yolo_ball_output_anchors(is_upper, highest_k) * get_yolo_ball_original_width(is_upper);
    detected_ball[3] = highest_confidence;
    if(highest_confidence > get_min_ball_confidence(is_upper)) {
      return BallPercept::seen;
    } else {
      return BallPercept::guessed;
    }
  }
  return BallPercept::notSeen;
}

bool YoloBallDetector::is_under_horizon(Geometry::Line horizon, float* model_output, uint8_t x, uint8_t y, bool is_upper) {
  //put everything between 0 and 1
  float ratio = horizon.base[0] * 2 / get_yolo_ball_input_width(is_upper); //probably 2
  
  float bx = (horizon.base[0] / ratio) / get_yolo_ball_input_width(is_upper); //probably always 0.5
  float by = (horizon.base[1] / ratio) / get_yolo_ball_input_height(is_upper); //variable

  float dx = horizon.direction[0] / get_yolo_ball_input_width(is_upper); //probably always around 1.f/input_width
  float dy = horizon.direction[1] / get_yolo_ball_input_height(is_upper); //variable

  float i = get_final_coord_x(model_output, x, y, is_upper); //already between 0 and 1
  float j = get_final_coord_y(model_output, x, y, is_upper); //already between 0 and 1

  //calculate the vertical distance between the detection point and the horizon line
  float pente = dy / dx;
  float distance_x = i - bx;
  float distance_y = distance_x * pente;
  return j > distance_y + by;
}
