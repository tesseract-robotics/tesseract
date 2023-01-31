/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#ifndef TESSERACT_COMMON_IMAGE_H
#define TESSERACT_COMMON_IMAGE_H

#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <tesseract_common/color.h>
#include <tesseract_common/utils.h>

namespace tesseract_common
{
/// \brief String names for the pixel formats.
/// \sa Image::PixelFormat.
static std::string PixelFormatNames[] = { "UNKNOWN_PIXEL_FORMAT",
                                          "L_INT8",
                                          "L_INT16",
                                          "RGB_INT8",
                                          "RGBA_INT8",
                                          "BGRA_INT8",
                                          "RGB_INT16",
                                          "RGB_INT32",
                                          "BGR_INT8",
                                          "BGR_INT16",
                                          "BGR_INT32",
                                          "R_FLOAT16",
                                          "RGB_FLOAT16",
                                          "R_FLOAT32",
                                          "RGB_FLOAT32",
                                          "BAYER_RGGB8",
                                          "BAYER_BGGR8",
                                          "BAYER_GBRG8",
                                          "BAYER_GRBG8" };

class ImagePrivate;

/// \class Image Image.hh gz/common/common.hh
/// \brief Encapsulates an image
class Image
{
public:
  using Ptr = std::shared_ptr<Image>;
  using ConstPtr = std::shared_ptr<const Image>;

  /// \brief Pixel formats enumeration
  enum PixelFormatType
  {
    UNKNOWN_PIXEL_FORMAT = 0,
    L_INT8,
    L_INT16,
    RGB_INT8,
    RGBA_INT8,
    BGRA_INT8,
    RGB_INT16,
    RGB_INT32,
    BGR_INT8,
    BGR_INT16,
    BGR_INT32,
    R_FLOAT16,
    RGB_FLOAT16,
    R_FLOAT32,
    RGB_FLOAT32,
    BAYER_RGGB8,
    BAYER_BGGR8,
    BAYER_GBRG8,
    BAYER_GRBG8,
    COMPRESSED_PNG,
    PIXEL_FORMAT_COUNT
  };

  /// \brief Convert a string to a Image::PixelFormat.
  /// \param[in] _format Pixel format string. \sa Image::PixelFormatNames
  /// \return Image::PixelFormat
  static Image::PixelFormatType convertPixelFormat(const std::string& format);

  /// \brief Constructor
  /// \param[in] _filename the path to the image
  explicit Image(const std::string& filename = "");

  /// \brief Destructor
  virtual ~Image();

  /// \brief Load an image. Return 0 on success
  /// \param[in] _filename the path to the image file
  /// \return 0 when the operation succeeds to open a file or -1 when fails.
  int load(const std::string& filename);

  /// \brief Save the image in PNG format
  /// \param[in] _filename The name of the saved image
  void savePNG(const std::string& filename);

  /// \brief Save the image in PNG format
  /// \param[in] _filename The name of the saved image
  void savePNGToBuffer(std::vector<unsigned char>& buffer);

  /// \brief Set the image from raw data
  /// \param[in] _data Pointer to the raw image data
  /// \param[in] _width Width in pixels
  /// \param[in] _height Height in pixels
  /// \param[in] _format Pixel format of the provided data
  void setFromData(const unsigned char* data, unsigned int width, unsigned int height, Image::PixelFormatType format);

  /// \brief Set the image from compressed (i.e. png) data
  /// \param[in] _data Pointer to the raw image data
  /// \param[in] _size Size of the buffer
  /// \param[in] _format Pixel format of the provided data
  void setFromCompressedData(unsigned char* data, unsigned int size, Image::PixelFormatType format);

  /// \brief Get the image as a data array
  /// \return The image data
  std::vector<unsigned char> getData() const;

  /// \brief Get only the RGB data from the image. This will drop the
  /// alpha channel if one is present.
  /// \return The image RGB data
  std::vector<unsigned char> getRGBData() const;

  /// \brief Get the RGBA data from the image. This will add an alpha
  /// channel if one is not present.
  /// \return The image RGBA data
  std::vector<unsigned char> getRGBAData() const;

  /// \brief Get the width
  /// \return The image width
  unsigned int width() const;

  /// \brief Get the height
  /// \return The image height
  unsigned int height() const;

  /// \brief Get the size of one pixel in bits
  /// \return The BPP of the image
  unsigned int getBPP() const;

  // \brief Get the size of a row of pixel
  /// \return The pitch of the image
  int getPitch() const;

  /// \brief Get the full filename of the image
  /// \return The filename used to load the image
  std::string getFilename() const;

  /// \brief Get the pixel format
  /// \return PixelFormat
  PixelFormatType getPixelFormat() const;

  /// \brief Get a pixel color value
  /// \param[in] _x Column location in the image
  /// \param[in] _y Row location in the image
  /// \return The color of the given pixel
  Color getPixel(unsigned int x, unsigned int y) const;

  /// \brief Get the average color
  /// \return The average color
  Color getAvgColor() const;

  /// \brief Get the max color
  /// \return The max color
  Color getMaxColor() const;

  /// \brief Rescale the image
  /// \param[in] _width New image width
  /// \param[in] _height New image height
  void rescale(int width, int height);

  /// \brief Returns whether this is a valid image
  /// \return true if image has a bitmap
  bool valid() const;

  /// \brief Convert a single channel image data buffer into an RGB image.
  /// During the conversion, the input image data are normalized to 8 bit
  /// values i.e. [0, 255]. Optionally, specify min and max values to use
  /// when normalizing the input image data. For example, if min and max
  /// are set to 1 and 10, a data value 2 will be normalized to:
  ///    (2 - 1) / (10 - 1) * 255.
  /// \param[in] _data input image data buffer
  /// \param[in] _width image width
  /// \param[in] _height image height
  /// \param[out] _output Output RGB image
  /// \param[in] _min Minimum value to be used when normalizing the input
  /// image data to RGB.
  /// \param[in] _max Maximum value to be used when normalizing the input
  /// image data to RGB.
  /// \param[in] _flip True to flip the values after normalization, i.e.
  /// lower values are converted to brigher pixels.
  template <typename T>
  static void convertToRGBImage(const void* _data,
                                unsigned int _width,
                                unsigned int _height,
                                Image& _output,
                                T _min = std::numeric_limits<T>::max(),
                                T _max = std::numeric_limits<T>::lowest(),
                                bool _flip = false)
  {
    unsigned int samples = _width * _height;
    unsigned int bufferSize = samples * sizeof(T);

    auto buffer = std::vector<T>(samples);
    memcpy(buffer.data(), _data, bufferSize);

    auto outputRgbBuffer = std::vector<uint8_t>(samples * 3);

    // use min and max values found in the data if not specified
    T min = std::numeric_limits<T>::max();
    T max = std::numeric_limits<T>::lowest();
    if (_min > max)
    {
      for (unsigned int i = 0; i < samples; ++i)
      {
        auto v = buffer[i];
        // ignore inf values when computing min/max
        // cast to float when calling isinf to avoid compile error on
        // windows
        if (v > max && !std::isinf(static_cast<float>(v)))
          max = v;
        if (v < min && !std::isinf(static_cast<float>(v)))
          min = v;
      }
    }
    min = almostEqualRelativeAndAbs(_min, std::numeric_limits<T>::max()) ? min : _min;
    max = almostEqualRelativeAndAbs(_max, std::numeric_limits<T>::lowest()) ? max : _max;

    // convert to rgb image
    // color is grayscale, i.e. r == b == g
    double range = static_cast<double>(max - min);
    if (almostEqualRelativeAndAbs(range, 0.0))
      range = 1.0;
    unsigned int idx = 0;
    for (unsigned int j = 0; j < _height; ++j)
    {
      for (unsigned int i = 0; i < _width; ++i)
      {
        auto v = buffer[idx++];
        double t = static_cast<double>(v - min) / range;
        if (_flip)
          t = 1.0 - t;
        uint8_t r = static_cast<uint8_t>(255 * t);
        unsigned int outIdx = j * _width * 3 + i * 3;
        outputRgbBuffer[outIdx] = r;
        outputRgbBuffer[outIdx + 1] = r;
        outputRgbBuffer[outIdx + 2] = r;
      }
    }
    _output.setFromData(outputRgbBuffer.data(), _width, _height, RGB_INT8);
  }

private:
  /// \brief Private data pointer
  std::unique_ptr<ImagePrivate> data_;
};
}  // namespace tesseract_common
#endif  // TESSERACT_COMMON_IMAGE_H
