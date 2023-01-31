/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#ifdef BOOL
#undef BOOL
#endif
#include <FreeImage.h>

#include <cstring>
#include <string>

#include <console_bridge/console.h>

#include <tesseract_common/image.h>
#include <tesseract_common/types.h>

static int count = 0;

namespace tesseract_common
{
/// \brief Private data class
class ImagePrivate
{
public:
  /// \brief bitmap data
  FIBITMAP* bitmap;

  /// \brief path name of the image file
  std::string fullName;

  /// \brief Implementation of GetData
  /// \deprecated remove once the Data functions using raw pointers
  /// are removed, in favor of returning vectors of bytes
  void dataImpl(unsigned char** data, unsigned int& count, FIBITMAP* img) const;

  /// \brief Implementation of Data, returns vector of bytes
  std::vector<unsigned char> dataImpl(FIBITMAP* img) const;

  /// \brief Returns true if SwapRedBlue can and should be called
  /// If it returns false, it may not be safe to call SwapRedBlue
  /// (it could lead to memory corruption!). See CanSwapRedBlue
  /// \return True if we should call SwapRedBlue
  bool shouldSwapRedBlue() const;

  /// \brief Returns true if SwapRedBlue is safe to be called
  /// \return False if it is NOT safe to call SwapRedBlue
  bool canSwapRedBlue() const;

  /// \brief Swap red and blue pixels
  /// \param[in] _width Width of the image
  /// \param[in] _height Height of the image
  /// \return bitmap data with red and blue pixels swapped
  FIBITMAP* swapRedBlue(unsigned int width, unsigned int height) const;

  /// \brief Get pixel value at specified index.
  /// \param[in] _dib Pointer to Freeimage bitmap
  /// \param[in] _x Pixel index in horizontal direction
  /// \param[in] _y Pixel index in vertical direction
  /// \param[out] _color Pixel value at specified index
  /// \return TRUE value if the pixel index was found and the color
  /// value set, FALSE otherwise.
  BOOL getPixelIndex(FIBITMAP* dib, unsigned x, unsigned y, Color& color) const;
};

//////////////////////////////////////////////////
Image::Image(const std::string& filename) : data_(std::make_unique<ImagePrivate>())
{
  if (count == 0)
    FreeImage_Initialise();

  count++;

  this->data_->bitmap = nullptr;
  if (!filename.empty())
  {
    if (!filename.empty())
      this->load(filename);
    else
      CONSOLE_BRIDGE_logError("Unable to find image[%s]", filename);
  }
}

//////////////////////////////////////////////////
Image::~Image()
{
  count--;

  if (this->data_->bitmap)
    FreeImage_Unload(this->data_->bitmap);
  this->data_->bitmap = NULL;

  if (count == 0)
    FreeImage_DeInitialise();
}

//////////////////////////////////////////////////
int Image::load(const std::string& filename)
{
  this->data_->fullName = filename;
  if (tesseract_common::fs::exists((this->data_->fullName)))
  {
    FREE_IMAGE_FORMAT fifmt = FreeImage_GetFIFFromFilename(this->data_->fullName.c_str());

    if (this->data_->bitmap)
      FreeImage_Unload(this->data_->bitmap);
    this->data_->bitmap = NULL;

    if (fifmt == FIF_PNG)
    {
      this->data_->bitmap = FreeImage_Load(fifmt, this->data_->fullName.c_str(), PNG_DEFAULT);
    }
    else if (fifmt == FIF_JPEG)
    {
      this->data_->bitmap = FreeImage_Load(fifmt, this->data_->fullName.c_str(), JPEG_DEFAULT);
    }
    else if (fifmt == FIF_BMP)
    {
      this->data_->bitmap = FreeImage_Load(fifmt, this->data_->fullName.c_str(), BMP_DEFAULT);
    }
    else
    {
      CONSOLE_BRIDGE_logError("Unknown image format[%s]", this->data_->fullName);
      return -1;
    }

    return 0;
  }

  CONSOLE_BRIDGE_logError("Unable to open image file[%s]", this->data_->fullName);
  return -1;
}

//////////////////////////////////////////////////
void Image::savePNG(const std::string& filename)
{
  FreeImage_Save(FIF_PNG, this->data_->bitmap, filename.c_str(), PNG_DEFAULT);
}

//////////////////////////////////////////////////
void Image::savePNGToBuffer(std::vector<unsigned char>& buffer)
{
  FIMEMORY* hmem = FreeImage_OpenMemory();
  FreeImage_SaveToMemory(FIF_PNG, this->data_->bitmap, hmem);
  unsigned char* memBuffer = nullptr;
#ifndef _WIN32
  unsigned int sizeInBytes = 0;
#else
  DWORD sizeInBytes = 0;
#endif
  FreeImage_AcquireMemory(hmem, &memBuffer, &sizeInBytes);
  buffer.resize(sizeInBytes);
  std::memcpy(buffer.data(), memBuffer, sizeInBytes);
  FreeImage_CloseMemory(hmem);
}

//////////////////////////////////////////////////
void Image::setFromData(const unsigned char* data,
                        unsigned int width,
                        unsigned int height,
                        Image::PixelFormatType format)
{
  if (this->data_->bitmap)
    FreeImage_Unload(this->data_->bitmap);
  this->data_->bitmap = NULL;

  // int redmask = FI_RGBA_RED_MASK;
  int redmask = 0x0000ff;

  // int greenmask = FI_RGBA_GREEN_MASK;
  int greenmask = 0x00ff00;

  // int bluemask = FI_RGBA_BLUE_MASK;
  int bluemask = 0xff0000;

  unsigned int bpp;
  int scanlineBytes;

  if (format == L_INT8)
  {
    bpp = 8;
    scanlineBytes = width;
  }
  else if (format == RGB_INT8)
  {
    bpp = 24;
    redmask = 0xff0000;
    greenmask = 0x00ff00;
    bluemask = 0x0000ff;
    scanlineBytes = width * 3;
  }
  else if (format == RGBA_INT8)
  {
    bpp = 32;
    redmask = 0xff000000;
    greenmask = 0x00ff0000;
    bluemask = 0x0000ff00;
    scanlineBytes = width * 4;
  }
  else if (format == BGR_INT8)
  {
    bpp = 24;
    redmask = 0x0000ff;
    greenmask = 0x00ff00;
    bluemask = 0xff0000;
    scanlineBytes = width * 3;
  }
  else
  {
    CONSOLE_BRIDGE_logError("Unable to handle format[%s]", format);
    return;
  }

  this->data_->bitmap = FreeImage_ConvertFromRawBits(
      const_cast<BYTE*>(data), width, height, scanlineBytes, bpp, redmask, greenmask, bluemask, true);

  if (this->data_->shouldSwapRedBlue())
  {
    FIBITMAP* toDelete = this->data_->bitmap;
    this->data_->bitmap = this->data_->swapRedBlue(this->width(), this->height());
    FreeImage_Unload(toDelete);
  }
}

//////////////////////////////////////////////////
void Image::setFromCompressedData(unsigned char* data, unsigned int size, Image::PixelFormatType format)
{
  if (this->data_->bitmap)
    FreeImage_Unload(this->data_->bitmap);
  this->data_->bitmap = nullptr;

  if (format == COMPRESSED_PNG)
  {
    FIMEMORY* fiMem = FreeImage_OpenMemory(data, size);
    this->data_->bitmap = FreeImage_LoadFromMemory(FIF_PNG, fiMem);
    FreeImage_CloseMemory(fiMem);
  }
  else
  {
    CONSOLE_BRIDGE_logError("Unable to handle format[%s]", format);
    return;
  }
}

//////////////////////////////////////////////////
int Image::getPitch() const { return FreeImage_GetLine(this->data_->bitmap); }

//////////////////////////////////////////////////
std::vector<unsigned char> Image::getRGBData() const
{
  std::vector<unsigned char> data;

  FIBITMAP* tmp = this->data_->bitmap;
  FIBITMAP* tmp2 = nullptr;
  if (this->data_->shouldSwapRedBlue())
  {
    tmp = this->data_->swapRedBlue(this->width(), this->height());
    tmp2 = tmp;
  }
  tmp = FreeImage_ConvertTo24Bits(tmp);
  data = this->data_->dataImpl(tmp);
  FreeImage_Unload(tmp);
  if (tmp2)
    FreeImage_Unload(tmp2);

  return data;
}

//////////////////////////////////////////////////
std::vector<unsigned char> Image::getRGBAData() const
{
  std::vector<unsigned char> data;

  FIBITMAP* tmp = this->data_->bitmap;
  FIBITMAP* tmp2 = nullptr;
  if (this->data_->shouldSwapRedBlue())
  {
    tmp = this->data_->swapRedBlue(this->width(), this->height());
    tmp2 = tmp;
  }
  tmp = FreeImage_ConvertTo32Bits(tmp);
  data = this->data_->dataImpl(tmp);
  FreeImage_Unload(tmp);
  if (tmp2)
    FreeImage_Unload(tmp2);

  return data;
}

//////////////////////////////////////////////////
std::vector<unsigned char> Image::getData() const
{
  std::vector<unsigned char> data;
  if (this->data_->shouldSwapRedBlue())
  {
    FIBITMAP* tmp = this->data_->swapRedBlue(this->width(), this->height());
    data = this->data_->dataImpl(tmp);
    FreeImage_Unload(tmp);
  }
  else
  {
    data = this->data_->dataImpl(this->data_->bitmap);
  }
  return data;
}

//////////////////////////////////////////////////
std::vector<unsigned char> ImagePrivate::dataImpl(FIBITMAP* img) const
{
  int redmask = FI_RGBA_RED_MASK;
  // int bluemask = 0x00ff0000;

  int greenmask = FI_RGBA_GREEN_MASK;
  // int greenmask = 0x0000ff00;

  int bluemask = FI_RGBA_BLUE_MASK;
  // int redmask = 0x000000ff;

  int scanWidth = FreeImage_GetLine(img);

  std::vector<unsigned char> data(scanWidth * FreeImage_GetHeight(img));

  FreeImage_ConvertToRawBits(
      reinterpret_cast<BYTE*>(&data[0]), img, scanWidth, FreeImage_GetBPP(img), redmask, greenmask, bluemask, true);

  return data;
}

//////////////////////////////////////////////////
void ImagePrivate::dataImpl(unsigned char** data, unsigned int& count, FIBITMAP* img) const
{
  int redmask = FI_RGBA_RED_MASK;
  // int bluemask = 0x00ff0000;

  int greenmask = FI_RGBA_GREEN_MASK;
  // int greenmask = 0x0000ff00;

  int bluemask = FI_RGBA_BLUE_MASK;
  // int redmask = 0x000000ff;

  int scanWidth = FreeImage_GetLine(img);

  if (*data)
    delete[] * data;

  count = scanWidth * FreeImage_GetHeight(img);
  *data = new unsigned char[count];

  FreeImage_ConvertToRawBits(
      reinterpret_cast<BYTE*>(*data), img, scanWidth, FreeImage_GetBPP(img), redmask, greenmask, bluemask, true);

#ifdef FREEIMAGE_COLORORDER
  // cppcheck-suppress ConfigurationNotChecked
  if (FREEIMAGE_COLORORDER != FREEIMAGE_COLORORDER_RGB)
  {
#else
#ifdef FREEIMAGE_BIGENDIAN
  if (false)
  {
#else
  {
#endif
#endif
    //  FIXME:  why shift by 2 pixels?
    //  this breaks heighmaps by wrapping artificially
    //    int i = 0;
    //    for (unsigned int y = 0; y < this->Height(); ++y)
    //    {
    //      for (unsigned int x = 0; x < this->Width(); ++x)
    //      {
    //        std::swap((*_data)[i], (*_data)[i+2]);
    //        unsigned int d = FreeImage_GetBPP(this->data_->bitmap)/8;
    //        i += d;
    //      }
    //    }
  }
}

//////////////////////////////////////////////////
unsigned int Image::width() const
{
  if (!this->valid())
    return 0;

  return FreeImage_GetWidth(this->data_->bitmap);
}

//////////////////////////////////////////////////
unsigned int Image::height() const
{
  if (!this->valid())
    return 0;

  return FreeImage_GetHeight(this->data_->bitmap);
}

//////////////////////////////////////////////////
unsigned int Image::getBPP() const
{
  if (!this->valid())
    return 0;

  return FreeImage_GetBPP(this->data_->bitmap);
}

//////////////////////////////////////////////////
Color Image::getPixel(unsigned int x, unsigned int y) const
{
  Color clr;

  if (!this->valid())
    return clr;

  FREE_IMAGE_COLOR_TYPE type = FreeImage_GetColorType(this->data_->bitmap);

  if (type == FIC_RGB || type == FIC_RGBALPHA)
  {
    RGBQUAD firgb;

    if (FreeImage_GetPixelColor(this->data_->bitmap, x, y, &firgb) == FALSE)
    {
      CONSOLE_BRIDGE_logError("Image: Coordinates out of range[%f, %f]", x, y);
      return clr;
    }
    clr.set(firgb.rgbRed, firgb.rgbGreen, firgb.rgbBlue);
  }
  else
  {
    if (this->data_->getPixelIndex(this->data_->bitmap, x, y, clr) == FALSE)
    {
      CONSOLE_BRIDGE_logError("Image: Coordinates out of range[%f, %f]", x, y);
      return clr;
    }
  }

  return clr;
}

//////////////////////////////////////////////////
Color Image::getAvgColor() const
{
  unsigned int x, y;
  double rsum, gsum, bsum;
  Color pixel;

  rsum = gsum = bsum = 0.0;
  for (y = 0; y < this->height(); ++y)
  {
    for (x = 0; x < this->width(); ++x)
    {
      pixel = this->getPixel(x, y);
      rsum += pixel.getR();
      gsum += pixel.getG();
      bsum += pixel.getB();
    }
  }

  rsum /= (this->width() * this->height());
  gsum /= (this->width() * this->height());
  bsum /= (this->width() * this->height());

  return Color(rsum, gsum, bsum);
}

//////////////////////////////////////////////////
Color Image::getMaxColor() const
{
  unsigned int x, y;
  Color clr;
  Color maxClr;

  maxClr.set(0, 0, 0, 0);

  if (!this->valid())
    return clr;

  FREE_IMAGE_COLOR_TYPE type = FreeImage_GetColorType(this->data_->bitmap);

  if (type == FIC_RGB || type == FIC_RGBALPHA)
  {
    RGBQUAD firgb;

    for (y = 0; y < this->height(); y++)
    {
      for (x = 0; x < this->width(); x++)
      {
        clr.set(0, 0, 0, 0);

        if (FALSE == FreeImage_GetPixelColor(this->data_->bitmap, x, y, &firgb))
        {
          CONSOLE_BRIDGE_logError("Image: Coordinates out of range[%f, %f]", x, y);
          continue;
        }
        clr.set(firgb.rgbRed, firgb.rgbGreen, firgb.rgbBlue);

        if (clr.getR() + clr.getG() + clr.getB() > maxClr.getR() + maxClr.getG() + maxClr.getB())
        {
          maxClr = clr;
        }
      }
    }
  }
  else
  {
    for (y = 0; y < this->height(); y++)
    {
      for (x = 0; x < this->width(); x++)
      {
        clr.set(0, 0, 0, 0);

        if (this->data_->getPixelIndex(this->data_->bitmap, x, y, clr) == FALSE)

        {
          CONSOLE_BRIDGE_logError("Image: Coordinates out of range[%f, %f]", x, y);
          continue;
        }

        if (clr.getR() + clr.getG() + clr.getB() > maxClr.getR() + maxClr.getG() + maxClr.getB())
        {
          maxClr = clr;
        }
      }
    }
  }

  return maxClr;
}

//////////////////////////////////////////////////
BOOL ImagePrivate::getPixelIndex(FIBITMAP* _dib, unsigned _x, unsigned _y, Color& _color) const
{
  if (!_dib)
    return FALSE;

  FREE_IMAGE_TYPE imageType = FreeImage_GetImageType(_dib);
  // 8 bit images
  if (imageType == FIT_BITMAP)
  {
    BYTE byteValue;
    // FreeImage_GetPixelIndex should also work with 1 and 4 bit images
    if (FreeImage_GetPixelIndex(_dib, _x, _y, &byteValue) == FALSE)
    {
      return FALSE;
    }

    unsigned int bpp = FreeImage_GetBPP(_dib);
    // convert to float value between 0-1
    float value = byteValue / static_cast<float>(((1 << (bpp)) - 1));
    _color.set(value, value, value);
  }
  // 16 bit images
  else if (imageType == FIT_UINT16)
  {
    if ((_x < FreeImage_GetWidth(_dib)) && (_y < FreeImage_GetHeight(_dib)))
    {
      WORD* bits = reinterpret_cast<WORD*>(FreeImage_GetScanLine(_dib, _y));
      uint16_t word = static_cast<uint16_t>(bits[_x]);
      // convert to float value between 0-1
      float value = word / static_cast<float>(std::numeric_limits<uint16_t>::max());
      _color.set(value, value, value);
    }
    else
    {
      return FALSE;
    }
  }
  return TRUE;
}

//////////////////////////////////////////////////
void Image::rescale(int width, int height)
{
  this->data_->bitmap = FreeImage_Rescale(this->data_->bitmap, width, height, FILTER_LANCZOS3);
}

//////////////////////////////////////////////////
bool Image::valid() const { return this->data_->bitmap != nullptr; }

//////////////////////////////////////////////////
std::string Image::getFilename() const { return this->data_->fullName; }

//////////////////////////////////////////////////
Image::PixelFormatType Image::getPixelFormat() const
{
  Image::PixelFormatType fmt = UNKNOWN_PIXEL_FORMAT;
  FREE_IMAGE_TYPE type = FreeImage_GetImageType(this->data_->bitmap);

  unsigned int redMask = FreeImage_GetRedMask(this->data_->bitmap);
  unsigned int bpp = this->getBPP();

  if (type == FIT_BITMAP)
  {
    if (bpp == 8)
      fmt = L_INT8;
    else if (bpp == 16)
      fmt = L_INT16;
    else if (bpp == 24)
      redMask == 0xff0000 ? fmt = RGB_INT8 : fmt = BGR_INT8;
    else if (bpp == 32)
    {
      redMask == 0xff0000 || redMask == 0xff000000 ? fmt = RGBA_INT8 : fmt = BGRA_INT8;
    }
  }
  else if (type == FIT_RGB16)
    fmt = RGB_INT16;
  else if (type == FIT_RGBF)
    fmt = RGB_FLOAT32;
  else if (type == FIT_UINT16 || type == FIT_INT16)
    fmt = L_INT16;

  return fmt;
}

/////////////////////////////////////////////////
Image::PixelFormatType Image::convertPixelFormat(const std::string& format)
{
  // Handle old format strings
  if (format == "L8" || format == "L_INT8")
    return L_INT8;
  else if (format == "R8G8B8" || format == "RGB_INT8")
    return RGB_INT8;

  // Handle BAYER_BGGR8 since it is after PIXEL_FORMAT_COUNT in the enum
  if (format == "BAYER_BGGR8")
  {
    return BAYER_BGGR8;
  }

  for (unsigned int i = 0; i < PIXEL_FORMAT_COUNT; ++i)
    if (PixelFormatNames[i] == format)
      return static_cast<PixelFormatType>(i);

  return UNKNOWN_PIXEL_FORMAT;
}

//////////////////////////////////////////////////
bool ImagePrivate::shouldSwapRedBlue() const
{
  return canSwapRedBlue() && FREEIMAGE_COLORORDER != FREEIMAGE_COLORORDER_RGB;
}

//////////////////////////////////////////////////
bool ImagePrivate::canSwapRedBlue() const
{
  const unsigned bpp = FreeImage_GetBPP(this->bitmap);
  return bpp == 24u || bpp == 32u;
}

//////////////////////////////////////////////////
FIBITMAP* ImagePrivate::swapRedBlue(unsigned int width, unsigned int height) const
{
  FIBITMAP* copy = FreeImage_Copy(this->bitmap, 0, 0, width, height);

  const unsigned bytesperpixel = FreeImage_GetBPP(this->bitmap) / 8;
  const unsigned pitch = FreeImage_GetPitch(this->bitmap);
  const unsigned lineSize = FreeImage_GetLine(this->bitmap);

  BYTE* line = FreeImage_GetBits(copy);
  for (unsigned y = 0; y < height; ++y, line += pitch)
  {
    for (BYTE* pixel = line; pixel < line + lineSize; pixel += bytesperpixel)
    {
      std::swap(pixel[0], pixel[2]);
    }
  }

  return copy;
}
}  // namespace tesseract_common
