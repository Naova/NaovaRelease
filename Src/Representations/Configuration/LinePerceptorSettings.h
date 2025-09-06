/**
 * @file LinesPerceptorSettings.h
 * Declaration of a representation that contains information about the line perceptor settings
 * @author Marc-Olivier Bisson
 */

#pragma once

#include "Tools/Math/Angle.h"
#include "Tools/Motion/MotionPhase.h"
#include "Tools/Range.h"
#include "Tools/RobotParts/Legs.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"
#include "Tools/Streams/EnumIndexedArray.h"
#include "Representations/Infrastructure/CameraInfo.h"

/**
 * @struct LinePerceptorSettings
 * A representation that contains the settings for the line perceptor
 */
STREAMABLE(LinePerceptorSettings,
{
  STREAMABLE(Settings,
  {,
    (int) resizeWidth,
    (int) resizeHeight,
    (double) maxValue,
    (int) blockSize,
    (double) subtractedC,
    (int) areaToSelect,
    (int) rho,
    (int) thetaDivision,
    (int) threshold,
    (int) minLineLength,
    (int) maxLineGap,
    (int) angleThreshold,
    (int) distanceThresholdMin,
    (int) distanceThresholdMax,
    (int) distanceThresholdContainedMin,
    (int) distanceThresholdContainedMax,
    (int) fieldBoundaryOffSet,
    (int) frameSkipModulo,
    (unsigned int) minCircleClusterSize, /**< minimum size of a cluster to be considered a valid circle candidate */
    (bool) isWhiteEnable,
  });

  const LinePerceptorSettings::Settings& operator[](const CameraInfo::Camera cameraType) const { return settings[cameraType]; },

  (ENUM_INDEXED_ARRAY(Settings, CameraInfo::Camera)) settings,
});
