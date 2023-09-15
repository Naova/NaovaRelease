/**
 * @file BallPerceptor.cpp
 * This file implements a module that detects balls in CNS images.
 * @author Thomas Röfer
 * @author Udo Frese
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "BallPerceptor.h"
#include "Platform/File.h"
#include "Platform/SystemCall.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/ImageProcessing/CNS/CNSTools.h"
#include "Tools/ImageProcessing/CNS/LutRasterizer.h"
#include "Tools/ImageProcessing/InImageSizeCalculations.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Settings.h"
#include "Tools/Streams/InStreams.h"

MAKE_MODULE(BallPerceptor, perception)

BallPerceptor::BallPerceptor()
{}

void BallPerceptor::update(BallPercept& ballPercept)
{
  DECLARE_DEBUG_DRAWING("module:BallPerceptor:candidates", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:BallPerceptor:around", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:BallPerceptor:samplePoints", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:BallPerceptor:acceptedPattern", "drawingOnImage");

  DEBUG_RESPONSE_ONCE("module:BallPerceptor:saveUpperImage") {
    dataset_creator.save_upper = !dataset_creator.save_upper;
    OUTPUT_TEXT("Save upper : " + std::to_string(dataset_creator.save_upper));
  }
  DEBUG_RESPONSE_ONCE("module:BallPerceptor:saveLowerImage") {
    dataset_creator.save_lower = !dataset_creator.save_lower;
    OUTPUT_TEXT("Save lower : " + std::to_string(dataset_creator.save_lower));
  }

  DEBUG_RESPONSE_ONCE("module:YoloBallDetector:printConfidenceUpper") {
    detector.print_confidence_upper = !detector.print_confidence_upper;
  }
  DEBUG_RESPONSE_ONCE("module:YoloBallDetector:printConfidenceLower") {
    detector.print_confidence_lower = !detector.print_confidence_lower;
  }
  DEBUG_RESPONSE_ONCE("module:BallPerceptor:saveImageWithBallOnly") {
    dataset_creator.save_with_ball_only = !dataset_creator.save_with_ball_only;
    OUTPUT_TEXT("Save lower : " + std::to_string(dataset_creator.save_with_ball_only));
  }

  if(theCameraMatrix.isValid)
  {
    //center_x, center_y, radius, confidenceLevel
    float detected_ball[4]; //x, y and confidence_level are normalized between 0.f and 1.f, radius is in pixels.
    bool is_upper = theImage.width == 320;

    Geometry::Line horizon = Geometry::calculateHorizon(theCameraMatrix, theCameraInfo);

    ballPercept.status = detector.searchBallOnImage(theImage, detected_ball, is_upper, horizon);
    if (ballPercept.status != BallPercept::notSeen) {
      detected_ball[0] *= theImage.width * 2;
      detected_ball[1] *= theImage.height * 2;
      ballPercept.positionInImage = Vector2f(detected_ball[0], detected_ball[1]);
      ballPercept.radiusInImage = detected_ball[2] * 2;
      ballPercept.confidenceLevel = detected_ball[3];

      dataset_creator.update(theImage, true, detected_ball[0], detected_ball[1], detected_ball[2]);

      const Vector2f correctedCenter = theImageCoordinateSystem.toCorrected(ballPercept.positionInImage);
      Vector3f cameraToBall(theCameraInfo.focalLength, theCameraInfo.opticalCenter.x() - correctedCenter.x(), theCameraInfo.opticalCenter.y() - correctedCenter.y());
      cameraToBall.normalize(theBallSpecification.radius * theCameraInfo.focalLength / ballPercept.radiusInImage);
      Vector3f rotatedCameraToBall = theCameraMatrix.rotation * cameraToBall;
                      
      if(rotatedCameraToBall.z() < 0)
      {
        const Vector3f bearingBasedCenterOnField = theCameraMatrix.translation - rotatedCameraToBall * ((theCameraMatrix.translation.z() - theBallSpecification.radius) / rotatedCameraToBall.z());
        ballPercept.positionOnField.x() = bearingBasedCenterOnField.x();
        ballPercept.positionOnField.y() = bearingBasedCenterOnField.y();
      }
      else
      {
        const Vector3f sizeBasedCenterOnField = theCameraMatrix.translation + rotatedCameraToBall;
        ballPercept.positionOnField.x() = sizeBasedCenterOnField.x();
        ballPercept.positionOnField.y() = sizeBasedCenterOnField.y();
      }
      //Elimine les faux positifs sur la croix
      //On se base sur le fait qu'il est impossible de voir la balle ET la croix a la meme position.
      if(thePenaltyMarkPercept.wasSeen) {
        if((ballPercept.positionOnField - thePenaltyMarkPercept.positionOnField).norm() < 80) {
          ballPercept.status = BallPercept::guessed;
        }
      }
    }
    else {
      dataset_creator.update(theImage);
    }
  }
}
