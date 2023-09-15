/**
 * @file BallPerceptor.h
 * This file declares a module that detects balls in CNS images.
 * @author Thomas Röfer
 * @author Udo Frese
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Representations/BehaviorControl/Role.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/YoloBallModelsDefinitions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/CameraIntrinsics.h"
#include "Representations/Infrastructure/DebuggingOutput.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/WorldModelPrediction.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Perception/BallPercepts/BallPercept.h"
#include "Representations/Perception/BallPercepts/BallSpots.h"
#include "Representations/Perception/ImagePreprocessing/BodyContour.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/CNSImage.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Representations/Perception/ImagePreprocessing/ImageRegions.h"
#include "Representations/Perception/FieldPercepts/PenaltyMarkPercept.h"
#include "Modules/Perception/DatasetCreator.h"
#include "Tools/ImageProcessing/YoloDetector/YoloBallDetector.h"
#include "Representations/Infrastructure/Image.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Module/Module.h"
#include "Tools/Streams/OutStreams.h"
#include <unordered_set>

MODULE(BallPerceptor,
{,
  USES(Role),
  USES(RobotPose),
  REQUIRES(MotionInfo),
  REQUIRES(BallSpecification),
  REQUIRES(BodyContour),
  REQUIRES(CameraInfo),
  REQUIRES(CameraIntrinsics),
  REQUIRES(CameraMatrix),
  REQUIRES(Image),
  REQUIRES(ECImage),
  REQUIRES(FieldDimensions),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(WorldModelPrediction),
  REQUIRES(BallPercept),
  REQUIRES(PenaltyMarkPercept),
  PROVIDES(BallPercept),
  LOADS_PARAMETERS(
  {,
    (float)(0.4f) simulationLowerGuessedTreshold,
    (float)(0.55f) simulationLowerDetectionTreshold,
    (float)(0.5f) simulationUpperGuessedTreshold,
    (float)(0.55f) simulationUpperDetectionTreshold,

    (float)(0.35f) robotLowerGuessedTreshold,
    (float)(0.5f) robotLowerDetectionTreshold,
    (float)(0.35f) robotUpperGuessedTreshold,
    (float)(0.56f) robotUpperDetectionTreshold,
  }),
});

class BallPerceptor : public BallPerceptorBase
{
private:
  //the detector
#ifdef TARGET_ROBOT
  YoloBallDetector detector = YoloBallDetector(robotLowerGuessedTreshold,
                                              robotLowerDetectionTreshold,
                                              robotUpperGuessedTreshold,
                                              robotUpperDetectionTreshold);
#else
  YoloBallDetector detector = YoloBallDetector(simulationLowerGuessedTreshold,
                                              simulationLowerDetectionTreshold,
                                              simulationUpperGuessedTreshold,
                                              simulationUpperDetectionTreshold);
#endif
  DatasetCreator dataset_creator;

  /**
   * The main method of this module.
   * @param ballPercept The percept that is filled by this module.
   */
  void update(BallPercept& ballPercept);

public:
  BallPerceptor();
};
