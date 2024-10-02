/**
 * @file CameraImage.h
 *
 * Declares a representation that allows for using the CameraImage as a Image<YUYVPixel>.
 *
 * @author Felix Thielke
 */

#pragma once

#include "Tools/ImageProcessing/Image.h"
#include "Tools/ImageProcessing/PixelTypes.h"
#include "Representations/Configuration/YoloModelDefinitions.h"

struct CameraImage : public Image<PixelTypes::YUYVPixel>
{
private:
  bool reference = false;

public:
  unsigned int timestamp = 0;

  static constexpr unsigned int maxResolutionWidth = 1280;
  static constexpr unsigned int maxResolutionHeight = 960;

  bool isReference() const
  {
    return reference;
  }

  void setResolution(const unsigned int width, const unsigned int height, const unsigned int padding = 0) override
  {
    Image::setResolution(width, height, padding);
    reference = false;
    widthStep = width*2;
  }

  void setReference(const unsigned int width, const unsigned int height, void* data, const unsigned int timestamp = 0)
  {
    reference = true;

    this->width = width;
    this->height = height;
    this->timestamp = timestamp;
    image = reinterpret_cast<PixelType*>(data);
  }

  unsigned char getY(const size_t x, const size_t y) const
  {
    return *(reinterpret_cast<const unsigned char*>(image) + y * width * 4 + x * 2);
  }
  unsigned char getU(const size_t x, const size_t y) const { return (*this)(x / 2, y).u; }
  unsigned char getV(const size_t x, const size_t y) const { return (*this)(x / 2, y).v; }
  PixelTypes::YUVPixel getYUV(const size_t x, const size_t y) const
  {
    const PixelTypes::YUYVPixel& yuyv = (*this)(x / 2, y);
    PixelTypes::YUVPixel yuv;
    yuv.y = yuyv.y(x);
    yuv.u = yuyv.u;
    yuv.v = yuyv.v;
    return yuv;
  }

  GrayscaledImage getGrayscaled() const
  {
    GrayscaledImage ret(width * 2, height);
    unsigned char* dest = ret[0];
    const PixelTypes::YUYVPixel* src = (*this)[0];
    for(unsigned y = 0; y < width; y++)
      for(unsigned x = 0; x < height; x++)
      {
        *dest++ = src->y0;
        *dest++ = src->y1;
        src++;
      }
    return ret;
  }

  void getResizedImage(int newWidth, int newHeight, CameraImage& image) const
  {
    float realWidth = (width * 2.0f);
    float x_scale = realWidth / newWidth;
    float y_scale = (height *1.f) / newHeight;
    image.setResolution(newWidth, newHeight);
    unsigned int x;
    unsigned int y;
    
    for(int i = 0; i < newHeight; ++i) {
      x = static_cast<int>(std::floor(1.f + (i * y_scale)));
      if(x >= realWidth) x = static_cast<int>(realWidth) - 1;
      for(int j = 0; j < newWidth; ++j) {
        y = static_cast<int>(std::floor(1.f + (j * x_scale)));
        if(y >= height) y = height - 1;
        image[i][j].y0 = (*this)(y,x).y0;
        image[i][j].u = (*this)(y,x).u;
        image[i][j].v = (*this)(y,x).v;
        image[i][j].y1 = (*this)(y,x).y1;
        image[i][j].color = (*this)(y,x).color;
      }
    }
  }

  void convertToFPNGformat(uint8_t *image_array) const
  {
    for(unsigned int i = 0; i < height; ++i) {
      for(unsigned int j = 0; j < width; ++j) {
        image_array[i * width * YOLO_BALL_INPUT_CHANNELS + j * YOLO_BALL_INPUT_CHANNELS] =  (*this).getY(j, i);
        image_array[i * width * YOLO_BALL_INPUT_CHANNELS + j * YOLO_BALL_INPUT_CHANNELS + 1] = (*this).getYUV(j, i).u;
        image_array[i * width * YOLO_BALL_INPUT_CHANNELS + j * YOLO_BALL_INPUT_CHANNELS + 2] = (*this).getYUV(j, i).v;
      }
    }
  }

protected:
  /**
   * Read this object from a stream.
   * @param stream The stream from which the object is read.
   */
  void read(In& stream) override
  {
    STREAM(width);
    STREAM(height);
    STREAM(timestamp);

    if(timestamp & (1 << 31))
    {
      height *= 2;
      timestamp &= ~(1 << 31);
    }

    setResolution(width, height, 0);
    stream.read(image, width * height * sizeof(PixelType));
  }

  /**
   * Write this object to a stream.
   * @param stream The stream to which the object is written.
   */
  void write(Out& stream) const override
  {
    STREAM(width);
    STREAM(height);
    STREAM(timestamp);
    stream.write(image, width * height * sizeof(PixelType));
  }

private:
  static void reg()
  {
    PUBLISH(reg);
    REG_CLASS(CameraImage);
    REG(width);
    REG(height);
    REG(timestamp);
  }
};
