/**
 * @file LinePerceptor.h
 *
 * Declares a module which detects lines and the center circle.
 *
 * @author Olivier St-Pierre
 * @author Marc-Olivier Bisson
 */

#pragma once

#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/LinePerceptorSettings.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/FieldPercepts/CirclePercept.h"
#include "Representations/Perception/FieldPercepts/LinesPercept.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Representations/Perception/ImagePreprocessing/FieldBoundary.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Representations/Perception/ImagePreprocessing/RelativeFieldColors.h"
#include "Representations/Perception/ObstaclesPercepts/ObstaclesImagePercept.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/LeastSquares.h"
#include "Tools/Module/Module.h"
#include "Tools/Debugging/DebugImages.h"
#include "Tools/ImageProcessing/Image.h"
#include "Tools/ImageProcessing/PixelTypes.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>


MODULE(LinePerceptor,
{,
  REQUIRES(CameraImage),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(ECImage),
  REQUIRES(FieldBoundary),
  REQUIRES(FieldDimensions),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(ObstaclesImagePercept),
  REQUIRES(RelativeFieldColors),
  REQUIRES(LinePerceptorSettings),
  PROVIDES(LinesPercept),
  REQUIRES(LinesPercept),
  PROVIDES(CirclePercept),
  LOADS_PARAMETERS(
  {,
    (unsigned int) whiteCheckStepSize,      /**< step size in px when checking if lines are white */
    (float) minWhiteRatio,               /**< minimum ratio of white pixels in lines */
    (float) maxCircleFittingError,        /**< maximum error of fitted circles through spots on the field in mm */
    (float) maxCircleRadiusDeviation,    /**< maximum deviation in mm of the perceived center circle radius to the expected radius */
    (unsigned int) minSpotsOnCircle,        /**< minimum number of spots on the center circle */
    (Angle) minCircleAngleBetweenSpots, /**< minimum angular distance around the circle center that has to be spanned by a circle candidate */
    (Angle) circleAngleBetweenSpots,    /**< angular distance around the circle center that should be spanned by a circle candidates spots */
    (float) minCircleWhiteRatio,         /**< minimum ratio of white pixels in the center circle */
    (float) sqrCircleClusterRadius, /**< squared radius for clustering center circle candidates */
    (float) squaredWhiteCheckNearField, /**< squared distance in mm from which on the whiteCheckDistance becomes increased due to possibly blurry images */
    (bool) perspectivelyCorrectWhiteCheck, /**< true: transform image to field, compute test points on field, project back; false: compute test points in image */
  }),
});

class LinePerceptor : public LinePerceptorBase
{
private:
  /**
   * Structure for a line spot.
   */
  struct Spot
  {
    Vector2f image;
    Vector2f field;
    unsigned int candidate;

    inline Spot(const float imgX, const float imgY) : image(imgX, imgY) {}
    inline Spot(const Vector2f& image, const Vector2f& field) : image(image), field(field) {}
  };

  /**
   * Structure for a center circle candidate.
   */
  struct CircleCandidate
  {
    Vector2f center;
    float radius;
    std::vector<Vector2f> fieldSpots;
    LeastSquares::CircleFitter fitter;

    /**
     * Adds the given field spot to the candidate and refits the circle.
     *
     * @param spot field spot to add
     */
    inline void addSpot(const Vector2f& spot)
    {
      fieldSpots.emplace_back(spot);
      fitter.add(spot);
      if(!fitter.fit(center, radius))
        radius = std::numeric_limits<float>::max();
    }

    /**
     * Calculates the distance of the given point to this circle candidate.
     *
     * @param point point to calculate the distance to
     */
    inline float getDistance(const Vector2f& point) const
    {
      return std::abs((center - point).norm() - radius);
    }

    /**
     * Calculates the average error of this circle candidate.
     */
    inline float calculateError() const
    {
      float error = 0.f;
      for(const Vector2f& spot : fieldSpots)
        error += getDistance(spot);
      return error / static_cast<float>(fieldSpots.size());
    }

    /**
     * Calculates how much of a circle corresponding to this candidate lies between the outermost fieldSpots.
     * @return Portion of the candidate that is between the outermost fieldSpots expressed as an angle.
     */
    inline Angle circlePartInImage() const
    {
      Vector2f referenceVector = fieldSpots[0] - center;
      Angle low = 0_deg;
      Angle high = 0_deg;
      auto spot = fieldSpots.cbegin();
      ++spot;
      for(; spot != fieldSpots.cend(); ++spot)
      {
        Vector2f spotAngleVector = *spot - center;
        Angle spotToReference = spotAngleVector.angleTo(referenceVector);
        if(spotAngleVector.x()*referenceVector.y() - spotAngleVector.y()*referenceVector.x() < 0)
        {
          if(spotToReference > low)
            low = spotToReference;
        }
        else
        {
          if(spotToReference > high)
            high = spotToReference;
        }
      }
      return low + high;
    }
  };

  /**
   * Structure for clustering center circle candidates.
   */
  struct CircleCluster
  {
    Vector2f center;
    std::vector<Vector2f> centers;

    inline CircleCluster(const Vector2f& center) : center(center) { centers.emplace_back(center); }
  };

  std::vector<CircleCandidate, Eigen::aligned_allocator<CircleCandidate>> circleCandidates;
  std::vector<CircleCluster> clusters;

  /** distance in mm where the field next to the line is sampled during white checks */
  const float whiteCheckDistance = theFieldDimensions.fieldLinesWidth * 2;
  /** squared maximum line width in mm that can occur when fitting circle candidates to the image */
  const float circleCorrectionMaxLineWidthSquared = sqr(theFieldDimensions.fieldLinesWidth * 2);

  /**
   * Updates the LinesPercept for the current frame.
   *
   * @param linesPercept the LinesPercept to update
   */
  void update(LinesPercept& linesPercept) override;

  /**
   * Updates the CirclePercept for the current frame.
   *
   * @param circlePercept the CirclePercept to update
   */
  void update(CirclePercept& circlePercept) override;

  /**
   * Corrects the given center circle candidate by projecting spots on the
   * circle back into the image, shifting them to actual white spots in the
   * image and calculating a new circle in field candidates from the results.
   *
   * @param circle circle candidate to correct
   * @return whether the candidate is valid before and after the correction
   */
  bool correctCircle(CircleCandidate& circle) const;

  /**
   * Adds the given potential circle center to a center circle cluster.
   *
   * @param center potential circle center coordinates
   */
  void clusterCircleCenter(const Vector2f& center);

  /**
   * Sets a flag on all lines in the LinePercept that lie on the detected center
   * circle.
   *
   * @param center center of the detected circle
   */
  void markLinesOnCircle(const Vector2f& center);

  /**
   * Checks whether the given points are connected by a white line.
   *
   * @param a line spot of the first point
   * @param b line spot of the second point
   * @param n0Field normal on field of the line segment or the candidate it shall be added to
   */
  bool isWhite(const Spot& a, const Spot& b, const Vector2f& n0Field) /*const*/;

  /**
   * Calculates the pixel distance a reference point for a white check shoud have to a given point.
   * @param pointOnField the Point in field coordinates. Used to estimate its distance
   * @return the points white check distance in pixels
   */
  float calcWhiteCheckDistanceInImage(const Vector2f& pointOnField) const;

  /**
   * Checks whether the given center circle candidate is white when projected
   * into image coordinates.
   *
   * @param center center of the circle in field coordinates
   * @param radius radius of the circle in field coordinates
   * @return result
   */
  bool isCircleWhite(const Vector2f& center, const float radius) const;

  /**
   * Tests whether a given point qualifies as white based on reference points
   * on both sides of the line or circle it lies on.
   * @param pointOnField checked point in field coordinates
   * @param pointInImage checked point in image coordinates
   * @param n0 normal vector of the checked line point i.e. direction in which to expect field
   * @return true, if the point is considered white
   */
  bool isPointWhite(const Vector2f& pointOnField, const Vector2i& pointInImage, const Vector2f& n0) const;

  /**
   * Tests whether a given point qualifies as white based on reference points
   * on both sides of the line or circle it lies on.
   * @param pointInImage checked point in image coordinates
   * @param n points from the pointInImage to the reference points
   * @return true, if the point is considered white
   */
  bool isPointWhite(const Vector2f& pointInImage, const Vector2f& n) const;

  /**
   * Calculates the on field distance that a reference point for a white check should have to a given point
   * @param pointOnField the point
   * @return the points white check distance on field
   */
  float calcWhiteCheckDistance(const Vector2f& pointOnField) const;

  /**
   * Checks whether the given line has a point inside the obstacle in the ObstaclesPercept.
   *
   * @param firstImgX the x coordinate of the first point of the line
   * @param firstImgY the y coordinate of the first point of the line
   * @param lastImgX the x coordinate of the last point of the line
   * @param lastImgY the y coordinate of the last point of the line
   */
  bool isSpotInsideObstacle(const int firstImgX, const int firstImgY, const int lastImgX, const int lastImgY) const;

  bool debugIsPointWhite = false;

  std::vector<LinesPercept::Line> detectLine();
  cv::Mat convertToMat(Image<PixelTypes::GrayscaledPixel> image);
  Image<PixelTypes::GrayscaledPixel> convertToImage(cv::Mat image);
  cv::Mat excludeBackground(cv::Mat image);
  cv::Mat filterBlobs(cv::Mat image, int areaToSelect);
  cv::Mat skeletonize(cv::Mat& img);
  std::vector<LinesPercept::Line> createLines(std::vector<cv::Vec4i> lines);
  std::vector<Vector2i> createSpotsInLine(Vector2i firstSpot, Vector2i lastSpot);
  void drawSpots(LinesPercept& linesPercept);


  std::vector<std::vector<cv::Vec4i>> groupLines(std::vector<cv::Vec4i> lines, int angleThreshold, int distanceThresholdMin, int distanceThresholdMax, int distanceThresholdContainedMin, int distanceThresholdContainedMax, int imageHeight);
  bool isPartOfGroup(std::vector<cv::Vec4i> group, cv::Vec4i line, int angleThreshold, int distanceThresholdMin, int distanceThresholdMax, int distanceThresholdContainedMin, int distanceThresholdContainedMax, int imageHeight);
  double angleBetweenLines(cv::Vec4i line1, cv::Vec4i line2);
  bool linesRespectDistanceThreshold(cv::Vec4i line, cv::Vec4i lineToGroup, int distanceThresholdMin, int distanceThresholdMax, int imageHeight);
  bool isLineContained(cv::Vec4i smallLine, cv::Vec4i largeLine, int distanceThresholdContainedMin, int distanceThresholdContainedMax, int imageHeight);
  double pointToLineDistance(cv::Point point, cv::Vec4i line);

  std::vector<cv::Vec4i> combineLines(std::vector<std::vector<cv::Vec4i>> groupedLines);
  double distance(cv::Point p1, cv::Point p2);

  LinePerceptorSettings::Settings settings;
  double scaleX;
  double scaleY;
  int frameCounter = 0;


  void draw() const
  {
    SEND_DEBUG_IMAGE("GrayscaledImage", grayscaled);
    SEND_DEBUG_IMAGE("GrayscaledImageBlur", grayscaledBlur);
    SEND_DEBUG_IMAGE("GrayscaledImageAdaptiveThreshold", grayscaledAdaptiveThreshold);
    SEND_DEBUG_IMAGE("GrayscaledImageSkel", grayscaledSkel);
  }

  Image<PixelTypes::GrayscaledPixel> grayscaled;
  Image<PixelTypes::GrayscaledPixel> grayscaledBlur;
  Image<PixelTypes::GrayscaledPixel> grayscaledAdaptiveThreshold;
  Image<PixelTypes::GrayscaledPixel> grayscaledSkel;

public:
  LinePerceptor()
  {
    circleCandidates.reserve(50);
    clusters.reserve(20);
  }
};
