/**
 * @file CameraSettings.cpp
 * Implementation of CameraSettings.
 */

#include "CameraSettings.h"
#include "Platform/BHAssert.h"
#include "Platform/Camera.h"

CameraSettings::CameraSettingsCollection::CameraSettingsCollection()
{
  settings.fill(-1000);
}

void CameraSettings::CameraSettingsCollection::serialize(In* in, Out* out)
{
  bool autoExposure = settings[CameraSettings::autoExposure] == 0;
  int& autoExposureBrightness = settings[CameraSettings::autoExposureBrightness];
  int& exposure = settings[CameraSettings::exposure];
  int& gain = settings[CameraSettings::gain];
  bool autoWhiteBalance = settings[CameraSettings::autoWhiteBalance] != 0;
  bool autoFocus = settings[CameraSettings::autoFocus] != 0;
  int& focus = settings[CameraSettings::focus];
  bool autoHue = settings[CameraSettings::autoHue] !=0;
  int& hue = settings[CameraSettings::hue];
  int& saturation = settings[CameraSettings::saturation];
  int& contrast = settings[CameraSettings::contrast];
  int& sharpness = settings[CameraSettings::sharpness];
  int& redGain = settings[CameraSettings::redGain];
  int& greenGain = settings[CameraSettings::greenGain];
  int& blueGain = settings[CameraSettings::blueGain];



  // ExposureAlgorithm autoExposureAlgorithm = static_cast<ExposureAlgorithm>(settings[CameraSettings::autoExposureAlgorithm]);
  //int& autoExposureBrightnessDark = settings[CameraSettings::autoExposureBrightnessDark];
  // float autoExposureMinVirtAnalogGain = FixedPoint5::fromRaw(settings[CameraSettings::autoExposureMinVirtAnalogGain]);
  // float autoExposureMaxVirtAnalogGain = FixedPoint5::fromRaw(settings[CameraSettings::autoExposureMaxVirtAnalogGain]);
  // float autoExposureMinVirtDigitalGain = FixedPoint7::fromRaw(settings[CameraSettings::autoExposureMinVirtDigitalGain]);
  // float autoExposureMaxVirtDigitalGain = FixedPoint7::fromRaw(settings[CameraSettings::autoExposureMaxVirtDigitalGain]);
  // float autoExposureTargetGain = FixedPoint5::fromRaw(settings[CameraSettings::autoExposureTargetGain]);
  // bool fadeToBlack = settings[CameraSettings::fadeToBlack] != 0;
  // //float gain = FixedPoint5::fromRaw(settings[CameraSettings::gain]);
  // PowerLineFrequency powerLineFrequency = static_cast<PowerLineFrequency>(settings[CameraSettings::powerLineFrequency] - 1);
  // //float saturation = FixedPoint7::fromRaw(settings[CameraSettings::saturation]);
  // int& whiteBalanceTemperature = settings[CameraSettings::whiteBalanceTemperature];

  STREAM_REGISTER_BEGIN;
  STREAM(autoExposure);
  // STREAM(autoExposureAlgorithm, CameraSettings);
  STREAM(autoExposureBrightness);
  STREAM(exposure);
  STREAM(gain);
  STREAM(autoWhiteBalance);
  STREAM(autoFocus);
  STREAM(focus);
  STREAM(autoHue);
  STREAM(hue);
  STREAM(saturation);
  STREAM(contrast);
  STREAM(sharpness);
  STREAM(redGain);
  STREAM(blueGain);
  STREAM(greenGain);



  //STREAM(autoExposureBrightnessDark);
  // STREAM(autoExposureMinVirtAnalogGain);
  // STREAM(autoExposureMaxVirtAnalogGain);
  // STREAM(autoExposureMinVirtDigitalGain);
  // STREAM(autoExposureMaxVirtDigitalGain);
  // STREAM(autoExposureTargetGain);
  // STREAM(fadeToBlack);
  // STREAM(powerLineFrequency, CameraSettings);
  // STREAM(whiteBalanceTemperature);
  STREAM_REGISTER_FINISH;

  if(in)
  {
    settings[CameraSettings::autoExposure] = autoExposure ? 0 : 1;
    settings[CameraSettings::autoWhiteBalance] = autoWhiteBalance ? 1 : 0;
    settings[CameraSettings::autoFocus] = autoFocus ? 1 : 0;
    settings[CameraSettings::autoHue] = autoHue ? 1 : 0;

    // settings[CameraSettings::autoExposureAlgorithm] = static_cast<int>(autoExposureAlgorithm);
    // settings[CameraSettings::autoExposureMinVirtAnalogGain] = FixedPoint5(autoExposureMinVirtAnalogGain).getRaw();
    // settings[CameraSettings::autoExposureMaxVirtAnalogGain] = FixedPoint5(autoExposureMaxVirtAnalogGain).getRaw();
    // settings[CameraSettings::autoExposureMinVirtDigitalGain] = FixedPoint7(autoExposureMinVirtDigitalGain).getRaw();
    // settings[CameraSettings::autoExposureMaxVirtDigitalGain] = FixedPoint7(autoExposureMaxVirtDigitalGain).getRaw();
    // settings[CameraSettings::autoExposureTargetGain] = FixedPoint5(autoExposureTargetGain).getRaw();
    //settings[CameraSettings::gain] = FixedPoint5(gain).getRaw();
    //settings[CameraSettings::saturation] = FixedPoint7(saturation).getRaw();
  }
}
