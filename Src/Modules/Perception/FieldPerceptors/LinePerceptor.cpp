/**
 * @file LinePerceptor.cpp
 *
 * Declares a module which detects lines and the center circle.
 *
 * @author Olivier St-Pierre
 * @author Marc-Olivier Bisson
 */

#include "LinePerceptor.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/ImageProcessing/PixelTypes.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Math/Deviation.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Transformation.h"

MAKE_MODULE(LinePerceptor, perception);

void LinePerceptor::update(LinesPercept& linesPercept)
{
    frameCounter++;
    settings = theLinePerceptorSettings[theCameraInfo.camera];
    drawSpots(linesPercept);
    if (frameCounter % settings.frameSkipModulo == 0)
    {
      DECLARE_DEBUG_DRAWING("module:LinePerceptor:spots", "drawingOnImage");
      DECLARE_DEBUG_DRAWING("module:LinePerceptor:visited", "drawingOnImage");
      DECLARE_DEBUG_DRAWING("module:LinePerceptor:upLow", "drawingOnImage");
      DECLARE_DEBUG_DRAWING("module:LinePerceptor:isWhite", "drawingOnImage");
      linesPercept.lines.clear();
      circleCandidates.clear();
      STOPWATCH("LinesPercept(detectLine)") { linesPercept.lines = detectLine(); }
      draw();
      frameCounter = 0;
    }
}

void LinePerceptor::drawSpots(LinesPercept& linesPercept)
{
  for(const LinesPercept::Line& line : linesPercept.lines)
  {
    for(const Vector2i& spot : line.spotsInImg)
    {
      CROSS("module:LinePerceptor:spots", spot.x(), spot.y(), 5, 3, Drawings::PenStyle::solidPen, ColorRGBA::red);
    }
  }
}

void LinePerceptor::update(CirclePercept& circlePercept)
{
  DECLARE_DEBUG_DRAWING("module:LinePerceptor:circlePoint", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:LinePerceptor:circleCheckPoint", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:LinePerceptor:circlePointField", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:LinePerceptor:circleCheckPointField", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:LinePerceptor:clustersCenter", "drawingOnField");
  DECLARE_DEBUG_RESPONSE("module:LinePerceptor:circleErrorStats");

  circlePercept.wasSeen = false;

  if(!circleCandidates.empty())
  {
    // find the best circle candidate to check extensively
    // the corrected circle position gets calculated only for a single candidate, to create an upper bound for the run time
    bool checkCandidate = false;
    size_t bestCandidateSpots = 0;
    CircleCandidate& bestCandidate = circleCandidates[0];

    // Find a valid center circle in the circle candidates
    for(CircleCandidate& candidate : circleCandidates)
    {
      if(candidate.fieldSpots.size() >= minSpotsOnCircle)
      {
        Angle inImageAngle = candidate.circlePartInImage();
        float maxFittingError = maxCircleFittingError;
        float maxRadiusDeviation = maxCircleRadiusDeviation;
        if(inImageAngle < circleAngleBetweenSpots && inImageAngle > minCircleAngleBetweenSpots)
        {
          // More narrow bounds for circles of which the robot sees only a small portion
          float reductionFactor = (inImageAngle - minCircleAngleBetweenSpots) / (circleAngleBetweenSpots - minCircleAngleBetweenSpots);
          maxFittingError *= reductionFactor;
          maxRadiusDeviation *= reductionFactor;
        }
        if(inImageAngle >= minCircleAngleBetweenSpots &&
           getAbsoluteDeviation(candidate.radius, theFieldDimensions.centerCircleRadius) <= maxRadiusDeviation &&
           candidate.calculateError() <= maxFittingError)
        {
          if(candidate.fieldSpots.size() > bestCandidateSpots)
          {
            checkCandidate = true;
            bestCandidateSpots = candidate.fieldSpots.size();
            bestCandidate = candidate;
          }
        }
      }
    }

    // time intensive checks
    if(checkCandidate && correctCircle(bestCandidate) &&
        isCircleWhite(bestCandidate.center, bestCandidate.radius))
    {
      circlePercept.pos = bestCandidate.center;
      circlePercept.wasSeen = true;
      markLinesOnCircle(bestCandidate.center);
      DEBUG_RESPONSE("module:LinePerceptor:circleErrorStats")
      {
        OUTPUT_TEXT("Part in image: " << bestCandidate.circlePartInImage());
        OUTPUT_TEXT("Fitting error: " << bestCandidate.calculateError());
        OUTPUT_TEXT("Radius deviation: " << getAbsoluteDeviation(bestCandidate.radius, theFieldDimensions.centerCircleRadius));
      }
      return;
    }
  }

  // Construct a circle from detected lines:
  // Cluster centers of arcs from detected lines
  clusters.clear();
  for(const LinesPercept::Line& line : theLinesPercept.lines)
  {
    if (line.spotsInField.size() >= 2){
      for(auto it = line.spotsInField.cbegin(); it < line.spotsInField.cend() -1; it++)
      {
      const Vector2f& a = *it;
      const Vector2f& b = *(++it);
      const Vector2f lineDirection = a-b;
      const Vector2f v_perp1(-lineDirection.y(), lineDirection.x());
      const Vector2f v_perp2(lineDirection.y(), -lineDirection.x());
      const Vector2f centerOfLine = (a+b)/2;
      const Vector2f centerA = (v_perp1.normalized() * theFieldDimensions.centerCircleRadius)+ centerOfLine;
      const Vector2f centerB = (v_perp2.normalized() * theFieldDimensions.centerCircleRadius) + centerOfLine;
      clusterCircleCenter(centerA);
      clusterCircleCenter(centerB);
      }
    }
  }

  // Find biggest valid cluster
  for(size_t n = clusters.size(); n; --n)
  {
    // Find current biggest cluster
    auto biggestCluster = clusters.begin();
    size_t maxSize = biggestCluster->centers.size();
    for(auto it = biggestCluster + 1; it < clusters.end(); ++it)
    {
      const size_t curSize = it->centers.size();
      if(curSize > maxSize)
      {
        maxSize = curSize;
        biggestCluster = it;
      }
    }
    
    // Abort if the minimum size is not reached
    if(maxSize < settings.minCircleClusterSize)
      break;

    // Check if the cluster is valid
    if(isCircleWhite(biggestCluster->center, theFieldDimensions.centerCircleRadius))
    {
      circlePercept.pos = biggestCluster->center;
      circlePercept.wasSeen = true;
      markLinesOnCircle(biggestCluster->center);
      break;
    }
    // Remove the cluster
    biggestCluster->centers.clear();
  }
}

std::vector<LinesPercept::Line> LinePerceptor::detectLine()
{
  GrayscaledImage grayscaledImage;
  STOPWATCH("LinesPercept(getGrayscaled)")
  {
    grayscaledImage = theECImage.grayscaled;
    grayscaled = grayscaledImage;
  }

  cv::Mat image;
  STOPWATCH("LinesPercept(convertToMat)")
  {
    image = convertToMat(grayscaledImage);
  }

  STOPWATCH("LinesPercept(resize)")
  {
    scaleX = static_cast<double>(image.cols) / settings.resizeWidth;
    scaleY = static_cast<double>(image.rows) / settings.resizeHeight;
    cv::resize(image, image, cv::Size(settings.resizeWidth, settings.resizeHeight));
  }

  STOPWATCH("LinesPercept(GaussianBlur)")
  {
    cv::GaussianBlur(image, image, cv::Size(15, 15), 0);
  }

  STOPWATCH("LinesPercept(convertToImage)")
  {
    grayscaledBlur = convertToImage(image);
  }
  
  STOPWATCH("LinesPercept(adaptiveThreshold)")
  {
    cv::adaptiveThreshold(image, image, settings.maxValue, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, settings.blockSize, settings.subtractedC);
  }

  STOPWATCH("LinesPercept(excludeBackground)")
  {
    image = excludeBackground(image);
  }

  STOPWATCH("LinesPercept(filterBlobs)")
  {
    image = filterBlobs(image, settings.areaToSelect);
    grayscaledAdaptiveThreshold = convertToImage(image);
  }

  cv::Mat skel;
  STOPWATCH("LinesPercept(skeletonize)")
  {
    skel = skeletonize(image);
    grayscaledSkel = convertToImage(skel);
  }

  std::vector<cv::Vec4i> lines;
  double theta = CV_PI / settings.thetaDivision;

  STOPWATCH("LinesPercept(HoughLinesP)")
  {
    cv::HoughLinesP(skel, lines, settings.rho, theta, settings.threshold, settings.minLineLength, settings.maxLineGap);
  }


  std::vector<std::vector<cv::Vec4i>> groupedLines;
  STOPWATCH("LinesPercept(groupLines)")
  {
    groupedLines = groupLines(lines, settings.angleThreshold, settings.distanceThresholdMin, settings.distanceThresholdMax, settings.distanceThresholdContainedMin, settings.distanceThresholdContainedMax, settings.resizeHeight);
  }

  STOPWATCH("LinesPercept(combineLines)")
  {
    lines = combineLines(groupedLines);
  }
  
  std::vector<LinesPercept::Line> linesPercept;
  STOPWATCH("LinesPercept(combineLines)")
  {
    linesPercept = createLines(lines);
  }
  
  return linesPercept;
}

cv::Mat LinePerceptor::convertToMat(Image<PixelTypes::GrayscaledPixel> image)
{
  cv::Mat mat(image.height, image.width, CV_8UC1);
  for(unsigned int y = 0; y < image.height; y++)
  {
    for(unsigned int x = 0; x < image.width; x++)
    {
      mat.at<uchar>(y, x) = image[y][x];
    }
  }
  return mat;
}

Image<PixelTypes::GrayscaledPixel> LinePerceptor::convertToImage(cv::Mat image)
{
  Image<PixelTypes::GrayscaledPixel> img(image.cols, image.rows);
  for(int y = 0; y < image.rows; y++)
  {
    for(int x = 0; x < image.cols; x++)
    {
      img[y][x] = image.at<uchar>(y, x);
    }
  }
  return img;
}

cv::Mat LinePerceptor::excludeBackground(cv::Mat image)
{
  if (theFieldBoundary.boundaryInImage.empty())
  {
    return image;
  }

  int height = image.rows;
  int width = image.cols;

  // Create a copy of fieldBoundaries and sort it by x-coordinate
  std::vector<cv::Point> polygonPoints;
  for (const auto& point : theFieldBoundary.boundaryInImage)
  {
    polygonPoints.push_back(cv::Point(static_cast<int>(point.x() / scaleX), static_cast<int>(point.y() / scaleY)));
  }

  std::sort(polygonPoints.begin(), polygonPoints.end(), [](const cv::Point& a, const cv::Point& b) {
    return a.x < b.x;
  });

  // Add bottom-right and bottom-left corners
  polygonPoints.push_back(cv::Point(width - 1, height - 1));
  polygonPoints.push_back(cv::Point(0, height - 1));

  // Create a mask with the same size as the image, initialized to zero
  cv::Mat mask = cv::Mat::zeros(image.size(), CV_8UC1);

  // Fill the polygon on the mask with white color (255)
  cv::fillPoly(mask, std::vector<std::vector<cv::Point>>{polygonPoints}, cv::Scalar(255));

  // Apply the mask to the image
  cv::Mat result;
  cv::bitwise_and(image, mask, result);

  return result;
}

cv::Mat LinePerceptor::filterBlobs(cv::Mat image, int areaToSelect)
{
  cv::Mat labels, stats, centroids;
  int numComponents = cv::connectedComponentsWithStats(image, labels, stats, centroids);

  // Extract the areas of the components, skipping the background (index 0)
  std::vector<int> areas;
  for (int i = 1; i < numComponents; ++i)
  {
    areas.push_back(stats.at<int>(i, cv::CC_STAT_AREA));
  }

  // Sort the areas in descending order and get the indices
  std::vector<int> sortedIndices(areas.size());
  std::iota(sortedIndices.begin(), sortedIndices.end(), 0);
  std::sort(sortedIndices.begin(), sortedIndices.end(), [&areas](int a, int b) {
    return areas[a] > areas[b];
  });

  // Select the top N largest areas
  int topN = std::min(areaToSelect, static_cast<int>(sortedIndices.size()));
  std::vector<int> largestIndices(sortedIndices.begin(), sortedIndices.begin() + topN);

  // Create a mask for the largest components
  cv::Mat filteredImage = cv::Mat::zeros(image.size(), CV_8UC1);
  for (int i : largestIndices)
  {
    int componentLabel = i + 1; // Labels start from 1 (0 is background)
    filteredImage.setTo(255, labels == componentLabel);
  }

  return filteredImage;
}

cv::Mat LinePerceptor::skeletonize(cv::Mat& img)
{
  unsigned long size = img.total() * img.channels();
  cv::Mat skel = cv::Mat::zeros(img.size(), CV_8UC1);

  cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
  bool done = false;

  while (!done)
  {
    cv::Mat eroded;
    cv::erode(img, eroded, element);
    cv::Mat dilated;
    cv::dilate(eroded, dilated, element);
    cv::Mat temp;
    cv::subtract(img, dilated, temp);
    cv::bitwise_or(skel, temp, skel);
    eroded.copyTo(img);

    unsigned long zeros = size - cv::countNonZero(img);

    if (zeros == size)
      done = true;
  }
  return skel;
}

std::vector<std::vector<cv::Vec4i>> LinePerceptor::groupLines(std::vector<cv::Vec4i> lines, int angleThreshold, int distanceThresholdMin, int distanceThresholdMax, int distanceThresholdContainedMin, int distanceThresholdContainedMax, int imageHeight)
{
  std::vector<std::vector<cv::Vec4i>> groupedLines;
  for (const auto& lineToGroup : lines)
  {
    bool assigned = false;
    for (auto& group : groupedLines)
    {
      assigned = isPartOfGroup(group, lineToGroup, angleThreshold, distanceThresholdMin, distanceThresholdMax, distanceThresholdContainedMin, distanceThresholdContainedMax, imageHeight);
      if (assigned)
        break;
    }
    
    if (!assigned)
    {
      groupedLines.push_back({lineToGroup});
    }
  }
  return groupedLines;
}

bool LinePerceptor::isPartOfGroup(std::vector<cv::Vec4i> group, cv::Vec4i line, int angleThreshold, int distanceThresholdMin, int distanceThresholdMax, int distanceThresholdContainedMin, int distanceThresholdContainedMax, int imageHeight)
{
  for (const auto& groupedLine : group)
  {
    double angleDiff = angleBetweenLines(line, groupedLine);

    if (angleDiff > angleThreshold)
      continue;

    if (linesRespectDistanceThreshold(line, groupedLine, distanceThresholdMin, distanceThresholdMax, imageHeight))
    {
      group.push_back(line);
      return true;
    }
    else
    {
      bool isContained1 = isLineContained(groupedLine, line, distanceThresholdContainedMin, distanceThresholdContainedMax, imageHeight);
      bool isContained2 = isLineContained(line, groupedLine, distanceThresholdContainedMin, distanceThresholdContainedMax, imageHeight);

      if (isContained1 || isContained2)
      {
        group.push_back(line);
        return true;
      }
    }

  }
  return false;
}

double LinePerceptor::angleBetweenLines(cv::Vec4i line1, cv::Vec4i line2)
{
  int x1_1 = line1[0];
  int y1_1 = line1[1];
  int x1_2 = line1[2];
  int y1_2 = line1[3];

  int x2_1 = line2[0];
  int y2_1 = line2[1];
  int x2_2 = line2[2];
  int y2_2 = line2[3];

  int dx1 = x1_2 - x1_1;
  int dy1 = y1_2 - y1_1;
  int  dx2 = x2_2 - x2_1;
  int dy2 = y2_2 - y2_1;

  int dot_product = dx1 * dx2 + dy1 * dy2;
  double magnitude1 = std::sqrt(std::pow(dx1,2) + std::pow(dy1,2));
  double magnitude2 = std::sqrt(std::pow(dx2,2) + std::pow(dy2,2));

  double cosTheta = dot_product / (magnitude1 * magnitude2);

  cosTheta = std::max(-1.0, std::min(1.0, cosTheta));

  double angleRadians = acos(cosTheta);
  double angleDegrees = angleRadians * (180.0 / M_PI);

  if(angleDegrees > 90)
  {
    angleDegrees = 180 - angleDegrees;
  }

  return angleDegrees;
}

bool LinePerceptor::linesRespectDistanceThreshold(cv::Vec4i line, cv::Vec4i lineToGroup, int distanceThresholdMin, int distanceThresholdMax, int imageHeight)
{
  int lineX1 = line[0], lineY1 = line[1], lineX2 = line[2], lineY2 = line[3];
  int lineToGroupX1 = lineToGroup[0], lineToGroupY1 = lineToGroup[1], lineToGroupX2 = lineToGroup[2], lineToGroupY2 = lineToGroup[3];

  std::vector<std::tuple<int, int, int, int>> pointCombinations = {
        {lineX1, lineY1, lineToGroupX1, lineToGroupY1},
        {lineX2, lineY2, lineToGroupX1, lineToGroupY1},
        {lineX1, lineY1, lineToGroupX2, lineToGroupY2},
        {lineX2, lineY2, lineToGroupX2, lineToGroupY2}
    };

    // Iterate over combinations and check the threshold condition
    for (const auto& combination : pointCombinations) {
        int x1 = std::get<0>(combination);
        int y1 = std::get<1>(combination);
        int x2 = std::get<2>(combination);
        int y2 = std::get<3>(combination);

        // Calculate yMean
        double yMean = (y1 + y2) / 2.0;

        // Calculate threshold scaling
        double thresholdScaling = distanceThresholdMax - distanceThresholdMin;

        // Calculate dynamic threshold
        double threshold = distanceThresholdMin + (thresholdScaling * yMean) / imageHeight;

        // Calculate distance
        double distance = std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));

        // Check if distance is below threshold
        if (distance < threshold) {
            return true;
        }
    }

    return false;
}

bool LinePerceptor::isLineContained(cv::Vec4i smallLine, cv::Vec4i largeLine, int distanceThresholdContainedMin, int distanceThresholdContainedMax, int imageHeight)
{
  cv::Point smallLineP1(smallLine[0], smallLine[1]);
  cv::Point smallLineP2(smallLine[2], smallLine[3]);
  double dist1 = pointToLineDistance(smallLineP1, largeLine);
  double dist2 = pointToLineDistance(smallLineP2, largeLine);

  double scaling_threshold = distanceThresholdContainedMax - distanceThresholdContainedMin;

  double threshold1 = distanceThresholdContainedMin + scaling_threshold * smallLineP1.y / imageHeight;
  double threshold2 = distanceThresholdContainedMin + scaling_threshold * smallLineP2.y / imageHeight;
  if (dist1 == -1)
      dist1 = threshold1+1;


  if (dist2 == -1)
      dist2 = threshold2+1;


  return dist1 <= threshold1 or dist2 <= threshold2;
}

double LinePerceptor::pointToLineDistance(cv::Point point, cv::Vec4i line)
{
  cv::Vec2i lineVector = cv::Vec2i(line[2] - line[0], line[3] - line[1]);
  cv::Vec2i pointVector = cv::Vec2i(point.x - line[0], point.y - line[1]);
  double linePenSquared = lineVector.dot(lineVector);

  // Cas spécial : si la ligne est un point (x1, y1 == x2, y2)
  if (linePenSquared == 0)
  {
    // Retourne la distance entre le point et l'extrémité
    return std::sqrt(std::pow(pointVector[0], 2) + std::pow(pointVector[1], 2));
  }

  // Projection scalaire du vecteur point sur le vecteur ligne
  double t = static_cast<double>(pointVector.dot(lineVector)) / linePenSquared;

  if (t > 1 || t < 0)
    return -1.f;

  double xProj = line[0] + t * lineVector[0];
  double yProj = line[1] + t * lineVector[1];

  return std::sqrt(std::pow(point.x - xProj, 2) + std::pow(point.y - yProj, 2));
}

std::vector<cv::Vec4i> LinePerceptor::combineLines(std::vector<std::vector<cv::Vec4i>> groupedLines)
{
  std::vector<cv::Vec4i> combinedLines;

  if (!groupedLines.empty())
  {
    for (const auto& group : groupedLines)
    {
      // Initialize variables to keep track of the most distant endpoints
      double maxDistance = 0;
      std::pair<cv::Point, cv::Point> farthestPoints;

      // Iterate through all endpoints in the group and calculate distances
      std::vector<cv::Point> endpoints;
      for (const auto& line : group)
      {
        endpoints.emplace_back(line[0], line[1]);
        endpoints.emplace_back(line[2], line[3]);
      }

      // Compare all pairs of endpoints across all lines in the group
      for (size_t i = 0; i < endpoints.size(); ++i)
      {
        for (size_t j = i + 1; j < endpoints.size(); ++j)
        {
          cv::Point p1 = endpoints[i];
          cv::Point p2 = endpoints[j];
          double dist = distance(p1, p2);
          if (dist > maxDistance)
          {
            maxDistance = dist;
            farthestPoints = {p1, p2};
          }
        }
      }

      if (maxDistance > 0)
      {
        combinedLines.emplace_back(cv::Vec4i(farthestPoints.first.x, farthestPoints.first.y, farthestPoints.second.x, farthestPoints.second.y));
      }
    }
  }

  return combinedLines;
}

double LinePerceptor::distance(cv::Point p1, cv::Point p2)
{
  return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

std::vector<LinesPercept::Line> LinePerceptor::createLines(std::vector<cv::Vec4i> lines)
{
  std::vector<LinesPercept::Line> perceptLines;

  for(const auto& l : lines)
  {
    LinesPercept::Line perceptLine;
    cv::Point p1(static_cast<int>(l[0] * scaleX), static_cast<int>(l[1] * scaleY));
    cv::Point p2(static_cast<int>(l[2] * scaleX), static_cast<int>(l[3] * scaleY));

    perceptLine.firstImg = Vector2i(p1.x, p1.y);
    perceptLine.spotsInImg.push_back(perceptLine.firstImg);
    perceptLine.lastImg = Vector2i(p2.x, p2.y);
    perceptLine.spotsInImg.push_back(perceptLine.lastImg);

    if (theFieldBoundary.isValid &&
        p1.y >= ((theFieldBoundary.getBoundaryY(p1.x))+ theLinePerceptorSettings[theCameraInfo.camera].fieldBoundaryOffSet) &&
        p2.y >= ((theFieldBoundary.getBoundaryY(p2.x))+ theLinePerceptorSettings[theCameraInfo.camera].fieldBoundaryOffSet) &&
        !isSpotInsideObstacle(perceptLine.firstImg.x(), perceptLine.firstImg.y(), perceptLine.lastImg.x(), perceptLine.lastImg.y()))
    {
      for(Vector2i& spotInImg : perceptLine.spotsInImg)
      {
        Vector2f spotInField;
        if(Transformation::imageToRobot(spotInImg, theCameraMatrix, theCameraInfo, spotInField))
          perceptLine.spotsInField.push_back(spotInField);
        else
          break;
      }

      if(perceptLine.spotsInField.size() < 2){
        continue;
      }
      perceptLine.firstField = perceptLine.spotsInField.front();
      perceptLine.lastField = perceptLine.spotsInField.back();

      Vector2f n0 = (perceptLine.lastField - perceptLine.firstField);
      n0.rotateLeft();
      n0.normalize();

      if (!settings.isWhiteEnable || isWhite(Spot(perceptLine.firstImg.cast<float>(), perceptLine.firstField), Spot(perceptLine.lastImg.cast<float>(), perceptLine.lastField), n0))
      {
        perceptLine.line = Geometry::Line(perceptLine.firstField, perceptLine.lastField - perceptLine.firstField);
        perceptLines.push_back(perceptLine);
      }
    }
  }
  return perceptLines;
}

bool LinePerceptor::isWhite(const Spot& a, const Spot& b, const Vector2f& n0Field)
{
  const Geometry::PixeledLine line(a.image.cast<int>(), b.image.cast<int>(), std::min(static_cast<int>(whiteCheckStepSize),
                                     static_cast<int>(std::ceil(static_cast<float>(std::max(getAbsoluteDeviation(a.image.x(), b.image.x()),
                                     getAbsoluteDeviation(a.image.y(), b.image.y()))) * minWhiteRatio))));

  const auto maxNonWhitePixels = static_cast<unsigned int>(static_cast<float>(line.size()) * (1 - minWhiteRatio));
  if(maxNonWhitePixels == line.size())
  {
    CROSS("module:LinePerceptor:isWhite", a.image.x(), a.image.y(), 1, 1, Drawings::solidPen, ColorRGBA::blue);
    CROSS("module:LinePerceptor:isWhite", b.image.x(), b.image.y(), 1, 1, Drawings::solidPen, ColorRGBA::blue);
    LINE("module:LinePerceptor:isWhite", a.image.x(), a.image.y(), b.image.x(), b.image.y(), 1, Drawings::solidPen, ColorRGBA::blue);
    return true;
  }

  if(perspectivelyCorrectWhiteCheck)
  {
    Vector2f pointOnField = a.field;
    unsigned int nonWhiteCount = 0;
    COMPLEX_DRAWING("module:LinePerceptor:isWhite") debugIsPointWhite = true;
    for(const Vector2i& p : line)
    {
      if(Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(p), theCameraMatrix, theCameraInfo, pointOnField))
      {
        if(!isPointWhite(pointOnField, p, n0Field))
        {
          ++nonWhiteCount;
          if(nonWhiteCount > maxNonWhitePixels)
          {
            COMPLEX_DRAWING("module:LinePerceptor:isWhite")
            {
              debugIsPointWhite = false;
              CROSS("module:LinePerceptor:isWhite", a.image.x(), a.image.y(), 1, 1, Drawings::solidPen, ColorRGBA::red);
              CROSS("module:LinePerceptor:isWhite", b.image.x(), b.image.y(), 1, 1, Drawings::solidPen, ColorRGBA::red);
              LINE("module:LinePerceptor:isWhite", a.image.x(), a.image.y(), b.image.x(), b.image.y(), 1, Drawings::solidPen, ColorRGBA::red);
            }
            return false;
          }
        }
      }
    }
    COMPLEX_DRAWING("module:LinePerceptor:isWhite")
    {
      debugIsPointWhite = false;
      CROSS("module:LinePerceptor:isWhite", a.image.x(), a.image.y(), 1, 1, Drawings::solidPen, ColorRGBA::blue);
      CROSS("module:LinePerceptor:isWhite", b.image.x(), b.image.y(), 1, 1, Drawings::solidPen, ColorRGBA::blue);
      LINE("module:LinePerceptor:isWhite", a.image.x(), a.image.y(), b.image.x(), b.image.y(), 1, Drawings::solidPen, ColorRGBA::blue);
    }
    return true;
  }
  else
  {
    Vector2f n0Image = b.image - a.image;
    n0Image.rotateLeft();
    n0Image.normalize();
    Vector2f pointOnField;
    float s = 1.f;
    unsigned int nonWhiteCount = 0;
    COMPLEX_DRAWING("module:LinePerceptor:isWhite") debugIsPointWhite = true;
    for(const Vector2i& p : line)
    {
      if(Transformation::imageToRobot(theImageCoordinateSystem.toCorrected(p), theCameraMatrix, theCameraInfo, pointOnField))
        s = calcWhiteCheckDistanceInImage(pointOnField);
      else
        return false;
      if(!isPointWhite(p.cast<float>(), s * n0Image))
      {
        ++nonWhiteCount;
        if(nonWhiteCount > maxNonWhitePixels)
        {
          COMPLEX_DRAWING("module:LinePerceptor:isWhite")
          {
            debugIsPointWhite = false;
            CROSS("module:LinePerceptor:isWhite", a.image.x(), a.image.y(), 1, 1, Drawings::solidPen, ColorRGBA::red);
            CROSS("module:LinePerceptor:isWhite", b.image.x(), b.image.y(), 1, 1, Drawings::solidPen, ColorRGBA::red);
            LINE("module:LinePerceptor:isWhite", a.image.x(), a.image.y(), b.image.x(), b.image.y(), 1, Drawings::solidPen, ColorRGBA::red);
          }
          return false;
        }
      }
    }
    COMPLEX_DRAWING("module:LinePerceptor:isWhite")
    {
      debugIsPointWhite = false;
      CROSS("module:LinePerceptor:isWhite", a.image.x(), a.image.y(), 1, 1, Drawings::solidPen, ColorRGBA::blue);
      CROSS("module:LinePerceptor:isWhite", b.image.x(), b.image.y(), 1, 1, Drawings::solidPen, ColorRGBA::blue);
      LINE("module:LinePerceptor:isWhite", a.image.x(), a.image.y(), b.image.x(), b.image.y(), 1, Drawings::solidPen, ColorRGBA::blue);
    }
    return true;
  }
}

float LinePerceptor::calcWhiteCheckDistanceInImage(const Vector2f& pointOnField) const
{
  // sorry for these magic numbers and formulas. I can't explain them, but they kind of work
  if(theCameraInfo.camera == CameraInfo::lower)
  {
    return 15.f + 32.f * (1.f - (pointOnField.x() + 0.5f * std::abs(pointOnField.y())) / 1500.f);
  }
  else
  {
    // avoid computing a root. accurate enough for this purpose
    float yFactor = pointOnField.x() <= 0 ? 1.f : pointOnField.x() <= 350.f ? 0.25f + 0.75f * (1.f - pointOnField.x() / 350.f) : 0.25f;
    float estimatedDistance = pointOnField.x() + yFactor * std::abs(pointOnField.y());
    return 35000.f / estimatedDistance;
  }
}

void LinePerceptor::clusterCircleCenter(const Vector2f& center)
{
  for(CircleCluster& cluster : clusters)
  {
    if((cluster.center - center).squaredNorm() <= sqrCircleClusterRadius)
    {
      cluster.centers.emplace_back(center);
      Vector2f newCenter(0, 0);
      for(const Vector2f& c : cluster.centers)
      {
        newCenter += c;
      }
      cluster.center = newCenter / static_cast<float>(cluster.centers.size());
      return;
    }
  }
  clusters.emplace_back(center);
}

void LinePerceptor::markLinesOnCircle(const Vector2f& center)
{
  for(const LinesPercept::Line& line : theLinesPercept.lines)
  {
    for(const Vector2f& spot : line.spotsInField)
    {
      if(getAbsoluteDeviation((center - spot).norm(), theFieldDimensions.centerCircleRadius) > maxCircleFittingError)
        goto lineNotOnCircle;
    }
    const_cast<LinesPercept::Line&>(line).belongsToCircle = true;
  lineNotOnCircle :
    ;
  }
}

bool LinePerceptor::correctCircle(CircleCandidate& circle) const
{
  Vector2f centerInImage;
  if(!Transformation::robotToImage(circle.center, theCameraMatrix, theCameraInfo, centerInImage))
    return false;

  centerInImage = theImageCoordinateSystem.fromCorrected(centerInImage);

  circle.fieldSpots.clear();

  for(Angle a = 0_deg; a < 360_deg; a += 5_deg)
  {
    Vector2f pointOnField(circle.center.x() + std::cos(a) * circle.radius, circle.center.y() + std::sin(a) * circle.radius);
    Vector2f pointInImage;
    if(Transformation::robotToImage(pointOnField, theCameraMatrix, theCameraInfo, pointInImage))
    {
      auto imageWidth = static_cast<float>(theCameraInfo.width);
      auto imageHeight = static_cast<float>(theCameraInfo.height);
      pointInImage = theImageCoordinateSystem.fromCorrected(pointInImage);
      if(pointInImage.x() >= 0 && pointInImage.x() < imageWidth && pointInImage.y() >= 0 && pointInImage.y() < imageHeight)
      {
        if(static_cast<float>(theFieldBoundary.getBoundaryY(static_cast<int>(pointInImage.x()))) > pointInImage.y())
          return false;

        Vector2f dir(pointInImage - centerInImage);
        float s = calcWhiteCheckDistance(pointOnField) / circle.radius * dir.norm();
        dir.normalize();
        Vector2f n = s * dir;

        Vector2f outer(pointInImage);
        short maxLuminance = 0;
        int maxLoops = static_cast<int>(s) + 1;
        for(int loops = 0; loops <= maxLoops && !isPointWhite(outer, n); ++loops)
        {
          outer += dir;
          if(loops == maxLoops || !(outer.x() >= 0 && outer.x() < imageWidth && outer.y() >= 0 && outer.y() < imageHeight))
          {
            outer = pointInImage;
            for(int inwardLoops = 0; inwardLoops <= maxLoops && !isPointWhite(outer, n); ++inwardLoops)
            {
              outer -= dir;
              if(inwardLoops == maxLoops || !(outer.x() >= 0 && outer.x() < imageWidth && outer.y() >= 0 && outer.y() < imageHeight))
                goto skipPoint;
            }
            break;
          }
        }
        pointInImage = outer;
        do
        {
          maxLuminance = std::max(maxLuminance, static_cast<short>(theECImage.grayscaled[static_cast<Vector2i>(outer.cast<int>())]));
          outer += dir;
        }
        while(outer.x() >= 0 && outer.x() < imageWidth && outer.y() >= 0 && outer.y() < imageHeight &&
              isPointWhite(outer, n) && theECImage.grayscaled[static_cast<Vector2i>(outer.cast<int>())] +
                                        theRelativeFieldColors.rfcParameters.minWhiteToFieldLuminanceDifference >= maxLuminance);
        outer -= dir;
        Vector2f inner(pointInImage);
        n = -n;
        while(!isPointWhite(inner, n))
        {
          inner -= dir;
          if(!(inner.x() >= 0 && inner.x() < imageWidth && inner.y() >= 0 && inner.y() < imageHeight))
            goto skipPoint;
        }
        do
        {
          maxLuminance = std::max(maxLuminance, static_cast<short>(theECImage.grayscaled[static_cast<Vector2i>(outer.cast<int>())]));
          inner -= dir;
        }
        while(inner.x() >= 0 && inner.x() < imageWidth && inner.y() >= 0 && inner.y() < imageHeight &&
              isPointWhite(inner, n) && theECImage.grayscaled[static_cast<Vector2i>(outer.cast<int>())] +
                                        theRelativeFieldColors.rfcParameters.minWhiteToFieldLuminanceDifference >= maxLuminance);
        inner += dir;

        CROSS("module:LinePerceptor:circlePoint", outer.x(), outer.y(), 5, 2, Drawings::solidPen, ColorRGBA::cyan);
        CROSS("module:LinePerceptor:circlePoint", inner.x(), inner.y(), 5, 2, Drawings::solidPen, ColorRGBA::yellow);

        outer = theImageCoordinateSystem.toCorrected(outer);
        inner = theImageCoordinateSystem.toCorrected(inner);
        Vector2f outerField, innerField;
        if(Transformation::imageToRobot(outer, theCameraMatrix, theCameraInfo, outerField) &&
           Transformation::imageToRobot(inner, theCameraMatrix, theCameraInfo, innerField) &&
           (outerField - innerField).squaredNorm() <= circleCorrectionMaxLineWidthSquared)
        {
          circle.fieldSpots.emplace_back((outerField + innerField) / 2);
          COMPLEX_DRAWING("module:LinePerceptor:circlePoint")
          {
            Vector2f uncor = (theImageCoordinateSystem.fromCorrected(outer) + theImageCoordinateSystem.fromCorrected(inner)) / 2.f;
            CROSS("module:LinePerceptor:circlePoint", uncor.x(), uncor.y(), 3, 1, Drawings::solidPen, ColorRGBA::gray);
          }
          CROSS("module:LinePerceptor:circlePointField", circle.fieldSpots.back().x(), circle.fieldSpots.back().y(), 5, 2, Drawings::solidPen, ColorRGBA::black);
        }
      }
    }

  skipPoint:
    ;
  }

  if(circle.fieldSpots.size() < minSpotsOnCircle)
    return false;

  LeastSquares::fitCircle(circle.fieldSpots, circle.center, circle.radius);

  for(size_t i = 0; i < circle.fieldSpots.size(); ++i)
  {
    Vector2f& spot = circle.fieldSpots[i];
    if(circle.getDistance(spot) > maxCircleFittingError)
    {
      do
      {
        spot = circle.fieldSpots.back();
        circle.fieldSpots.pop_back();
        if(circle.fieldSpots.size() < minSpotsOnCircle)
        {
          return false;
        }
      }
      while(i < circle.fieldSpots.size() && circle.getDistance(spot) > maxCircleFittingError);
    }
  }

  circle.fitter = LeastSquares::CircleFitter();
  circle.fitter.add(circle.fieldSpots);

  return circle.fitter.fit(circle.center, circle.radius) &&
         getAbsoluteDeviation(circle.radius, theFieldDimensions.centerCircleRadius) <= maxCircleRadiusDeviation &&
         circle.calculateError() <= maxCircleFittingError;
}

bool LinePerceptor::isCircleWhite(const Vector2f& center, const float radius) const
{
  unsigned int whiteCount = 0, count = 0;
  for(Angle a = 0_deg; a < 360_deg; a += 5_deg)
  {
    Vector2f pointOnField(center.x() + std::cos(a) * radius, center.y() + std::sin(a) * radius);
    Vector2f pointInImage;
    if(Transformation::robotToImage(pointOnField, theCameraMatrix, theCameraInfo, pointInImage))
    {
      pointInImage = theImageCoordinateSystem.fromCorrected(pointInImage);
      if(pointInImage.x() >= 0 && pointInImage.x() < static_cast<float>(theCameraInfo.width) &&
         pointInImage.y() >= 0 && pointInImage.y() < static_cast<float>(theCameraInfo.height))
      {
        CROSS("module:LinePerceptor:circleCheckPoint", pointInImage.x(), pointInImage.y(), 5, 2, Drawings::solidPen, ColorRGBA::black);
        CROSS("module:LinePerceptor:circleCheckPointField", pointOnField.x(), pointOnField.y(), 5, 2, Drawings::solidPen, ColorRGBA::black);
        count++;
        Vector2f n0(std::cos(a), std::sin(a)); // normal vector pointing outward
        if(isPointWhite(pointOnField, pointInImage.cast<int>(), n0))
          whiteCount++;
      }
    }
  }
  return count > 0 && static_cast<float>(whiteCount) / static_cast<float>(count) >= minCircleWhiteRatio;
}

bool LinePerceptor::isPointWhite(const Vector2f& pointOnField, const Vector2i& pointInImage, const Vector2f& n0) const
{
  Vector2f nw = calcWhiteCheckDistance(pointOnField) * n0;
  Vector2f outerPointOnField = pointOnField + nw;
  Vector2f innerPointOnField = pointOnField - nw;
  Vector2f referencePointInImage;
  unsigned short luminanceReference = 0, saturationReference = 0;
  bool isOuterPointInImage = false;
  if(Transformation::robotToImage(outerPointOnField, theCameraMatrix, theCameraInfo, referencePointInImage))
  {
    referencePointInImage = theImageCoordinateSystem.fromCorrected(referencePointInImage);
    Vector2i integerReferenceInImage = referencePointInImage.cast<int>();
    if(integerReferenceInImage.x() >= 0 && integerReferenceInImage.x() < theCameraInfo.width &&
       integerReferenceInImage.y() >= 0 && integerReferenceInImage.y() < theCameraInfo.height)
    {
      luminanceReference = theECImage.grayscaled[integerReferenceInImage];
      saturationReference = theECImage.saturated[integerReferenceInImage];
      isOuterPointInImage = true;
      if(debugIsPointWhite)
      {
        CROSS("module:LinePerceptor:isWhite", referencePointInImage.x(), referencePointInImage.y(), 2, 1, Drawings::solidPen, ColorRGBA::orange);
        LINE("module:LinePerceptor:isWhite", pointInImage.x(), pointInImage.y(), referencePointInImage.x(), referencePointInImage.y(), 1, Drawings::solidPen,
             ColorRGBA::orange);
      }
    }
  }
  if(Transformation::robotToImage(innerPointOnField, theCameraMatrix, theCameraInfo, referencePointInImage))
  {
    referencePointInImage = theImageCoordinateSystem.fromCorrected(referencePointInImage);
    Vector2i integerReferenceInImage = referencePointInImage.cast<int>();
    if(integerReferenceInImage.x() >= 0 && integerReferenceInImage.x() < theCameraInfo.width &&
       integerReferenceInImage.y() >= 0 && integerReferenceInImage.y() < theCameraInfo.height)
    {
      if(isOuterPointInImage)
      {
        luminanceReference = (luminanceReference + theECImage.grayscaled[integerReferenceInImage] + 1) / 2;
        saturationReference = (saturationReference + theECImage.saturated[integerReferenceInImage]) / 2;
      }
      else
      {
        luminanceReference = theECImage.grayscaled[integerReferenceInImage];
        saturationReference = theECImage.saturated[integerReferenceInImage];
      }
      if(debugIsPointWhite)
      {
        CROSS("module:LinePerceptor:isWhite", referencePointInImage.x(), referencePointInImage.y(), 2, 1, Drawings::solidPen, ColorRGBA::orange);
        LINE("module:LinePerceptor:isWhite", pointInImage.x(), pointInImage.y(), referencePointInImage.x(), referencePointInImage.y(),
               1, Drawings::solidPen, ColorRGBA::orange);
      }
    }
  }
  return theRelativeFieldColors.isWhiteNearField(theECImage.grayscaled[pointInImage],theECImage.saturated[pointInImage],
                                                  static_cast<unsigned char>(luminanceReference), static_cast<unsigned char>(saturationReference));
}

bool LinePerceptor::isPointWhite(const Vector2f& pointInImage, const Vector2f& n) const
{
  bool isOuterPointInImage = false;
  unsigned short luminanceReference = 0, saturationReference = 0;
  Vector2i outerReference = (pointInImage + n).cast<int>();
  if(outerReference.x() >= 0 && outerReference.x() < theCameraInfo.width &&
      outerReference.y() >= 0 && outerReference.y() < theCameraInfo.height)
  {
    luminanceReference = theECImage.grayscaled[outerReference];
    saturationReference = theECImage.saturated[outerReference];
    isOuterPointInImage = true;
    if(debugIsPointWhite)
    {
      CROSS("module:LinePerceptor:isWhite", outerReference.x(), outerReference.y(), 2, 1, Drawings::solidPen, ColorRGBA::orange);
      LINE("module:LinePerceptor:isWhite", pointInImage.x(), pointInImage.y(), outerReference.x(), outerReference.y(), 1, Drawings::solidPen, ColorRGBA::orange);
    }
  }
  Vector2i innerReference = (pointInImage - n).cast<int>();
  if(innerReference.x() >= 0 && innerReference.x() < theCameraInfo.width &&
     innerReference.y() >= 0 && innerReference.y() < theCameraInfo.height)
  {
    if(isOuterPointInImage)
    {
      luminanceReference = (luminanceReference + theECImage.grayscaled[innerReference] + 1) / 2;
      saturationReference = (saturationReference + theECImage.saturated[innerReference]) / 2;
    }
    else
    {
      luminanceReference = theECImage.grayscaled[innerReference];
      saturationReference = theECImage.saturated[innerReference];
    }
    if(debugIsPointWhite)
    {
      CROSS("module:LinePerceptor:isWhite", innerReference.x(), innerReference.y(), 2, 1, Drawings::solidPen, ColorRGBA::orange);
      LINE("module:LinePerceptor:isWhite", pointInImage.x(), pointInImage.y(), innerReference.x(), innerReference.y(), 1, Drawings::solidPen, ColorRGBA::orange);
    }
  }
  Vector2i intPointInImage = pointInImage.cast<int>();
  return theRelativeFieldColors.isWhiteNearField(theECImage.grayscaled[intPointInImage],theECImage.saturated[intPointInImage],
                                                 static_cast<unsigned char>(luminanceReference), static_cast<unsigned char>(saturationReference));
}

float LinePerceptor::calcWhiteCheckDistance(const Vector2f& pointOnField) const
{
  if(pointOnField.squaredNorm() > squaredWhiteCheckNearField)
    return whiteCheckDistance * (1.f + 0.5f * sqr(1.f - squaredWhiteCheckNearField / pointOnField.squaredNorm()));
  else
    return whiteCheckDistance;
}

bool LinePerceptor::isSpotInsideObstacle(const int firstImgX, const int firstImgY, const int lastImgX, const int lastImgY) const
{
  auto predicate = [&](const ObstaclesImagePercept::Obstacle& obstacle)
  {
    if(((firstImgX >= obstacle.left && firstImgX <= obstacle.right) || (lastImgX >= obstacle.left && lastImgX <= obstacle.right))
      && ((firstImgY >= obstacle.top && firstImgY <= obstacle.bottom) || (lastImgY >= obstacle.top && lastImgY <= obstacle.bottom)))
      return true;
    else
      return false;
  };
  return std::any_of(theObstaclesImagePercept.obstacles.cbegin(), theObstaclesImagePercept.obstacles.cend(), predicate);
}
