/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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
#include <cmath>
#include <algorithm>

#include <tesseract_common/color.h>
#include <tesseract_common/utils.h>

namespace
{
// Use constexpr storage for the Color constants, to avoid the C++ static
// initialization order fiasco.
constexpr tesseract_common::Color gWhite = tesseract_common::Color(1, 1, 1, 1);
constexpr tesseract_common::Color gBlack = tesseract_common::Color(0, 0, 0, 1);
constexpr tesseract_common::Color gRed = tesseract_common::Color(1, 0, 0, 1);
constexpr tesseract_common::Color gGreen = tesseract_common::Color(0, 1, 0, 1);
constexpr tesseract_common::Color gBlue = tesseract_common::Color(0, 0, 1, 1);
constexpr tesseract_common::Color gYellow = tesseract_common::Color(1, 1, 0, 1);
constexpr tesseract_common::Color gMagenta = tesseract_common::Color(1, 0, 1, 1);
constexpr tesseract_common::Color gCyan = tesseract_common::Color(0, 1, 1, 1);

}  // namespace

namespace tesseract_common
{
const Color& Color::White = gWhite;
const Color& Color::Black = gBlack;
const Color& Color::Red = gRed;
const Color& Color::Green = gGreen;
const Color& Color::Blue = gBlue;
const Color& Color::Yellow = gYellow;
const Color& Color::Magenta = gMagenta;
const Color& Color::Cyan = gCyan;

//////////////////////////////////////////////////
void Color::reset()
{
  this->r_ = this->g_ = this->b_ = 0;
  this->a_ = 1;
}

//////////////////////////////////////////////////
void Color::set(float r, float g, float b, float a)
{
  this->r_ = r;
  this->g_ = g;
  this->b_ = b;
  this->a_ = a;

  this->Clamp();
}

//////////////////////////////////////////////////
void Color::setFromHSV(float h, float s, float v)
{
  int i;
  float f, p, q, t;

  h = static_cast<float>(static_cast<int>(h < 0 ? 0 : h) % 360);

  if (almostEqualRelativeAndAbs(s, 0.0f))
  {
    // acromatic (grey)
    this->r_ = this->g_ = this->b_ = v;
    return;
  }

  // sector 0 - 5
  h /= 60;

  i = static_cast<int>(floor(h));

  f = h - i;

  p = v * (1 - s);
  q = v * (1 - s * f);
  t = v * (1 - s * (1 - f));

  switch (i)
  {
    case 0:
      this->r_ = v;
      this->g_ = t;
      this->b_ = p;
      break;
    case 1:
      this->r_ = q;
      this->g_ = v;
      this->b_ = p;
      break;
    case 2:
      this->r_ = p;
      this->g_ = v;
      this->b_ = t;
      break;
    case 3:
      this->r_ = p;
      this->g_ = q;
      this->b_ = v;
      break;
    case 4:
      this->r_ = t;
      this->g_ = p;
      this->b_ = v;
      break;
    case 5:
    default:
      this->r_ = v;
      this->g_ = p;
      this->b_ = q;
      break;
  }

  this->Clamp();
}

//////////////////////////////////////////////////
Eigen::Vector3f Color::getHSV() const
{
  Eigen::Vector3f hsv;

  float min = std::min(this->r_, std::min(this->g_, this->b_));
  float max = std::max(this->r_, std::max(this->g_, this->b_));
  float delta = max - min;

  hsv.y() = delta / max;
  hsv.z() = max;

  if (almostEqualRelativeAndAbs(delta, 0.0f))
  {
    hsv.x() = 0.0;
    hsv.y() = 0.0;
  }
  else if (almostEqualRelativeAndAbs(this->r_, min))
    hsv.x() = 3 - ((this->g_ - this->b_) / delta);
  else if (almostEqualRelativeAndAbs(this->g_, min))
    hsv.x() = 5 - ((this->b_ - this->r_) / delta);
  else
    hsv.x() = 1 - ((this->r_ - this->g_) / delta);

  hsv.x() *= 60.0;
  return hsv;
}

//////////////////////////////////////////////////
Eigen::Vector3f Color::getYUV() const
{
  Eigen::Vector3f yuv;

  yuv.x() = 0.299f * this->r_ + 0.587f * this->g_ + 0.114f * this->b_;
  yuv.y() = -0.1679f * this->r_ - 0.332f * this->g_ + 0.5f * this->b_ + 0.5f;
  yuv.z() = 0.5f * this->r_ - 0.4189f * this->g_ - 0.08105f * this->b_ + 0.5f;

  yuv.x() = yuv.x() < 0 ? 0 : yuv.x();
  yuv.x() = yuv.x() > 255 ? 255.0f : yuv.x();

  yuv.y() = yuv.y() < 0 ? 0 : yuv.y();
  yuv.y() = yuv.y() > 255 ? 255.0f : yuv.y();

  yuv.z() = yuv.z() < 0 ? 0 : yuv.z();
  yuv.z() = yuv.z() > 255 ? 255.0f : yuv.z();

  return yuv;
}

//////////////////////////////////////////////////
void Color::setFromYUV(float y, float u, float v)
{
  this->r_ = y + 1.140f * v;
  this->g_ = y - 0.395f * u - 0.581f * v;
  this->b_ = y + 2.032f * u;
  this->Clamp();
}

//////////////////////////////////////////////////
float Color::operator[](const unsigned int index) { return (*static_cast<const Color*>(this))[index]; }

//////////////////////////////////////////////////
float Color::operator[](const unsigned int _index) const
{
  switch (_index)
  {
    case 0:
      return this->r_;
    case 1:
      return this->g_;
    case 2:
      return this->b_;
    case 3:
      return this->a_;
    default:
      break;
  }

  return std::numeric_limits<float>::quiet_NaN();
}

//////////////////////////////////////////////////
Color::RGBA Color::asRGBA() const
{
  uint8_t val8;
  unsigned int val32;

  // Convert to 32bit pattern
  // (RGBA = 8888)

  val8 = static_cast<uint8_t>(this->r_ * 255);
  val32 = val8 << 24;

  val8 = static_cast<uint8_t>(this->g_ * 255);
  val32 += val8 << 16;

  val8 = static_cast<uint8_t>(this->b_ * 255);
  val32 += val8 << 8;

  val8 = static_cast<uint8_t>(this->a_ * 255);
  val32 += val8;

  return val32;
}

//////////////////////////////////////////////////
Color::BGRA Color::asBGRA() const
{
  uint8_t val8;
  unsigned int val32 = 0;

  // Convert to 32bit pattern
  // (BGRA = 8888)

  val8 = static_cast<uint8_t>(this->b_ * 255);
  val32 = val8 << 24;

  val8 = static_cast<uint8_t>(this->g_ * 255);
  val32 += val8 << 16;

  val8 = static_cast<uint8_t>(this->r_ * 255);
  val32 += val8 << 8;

  val8 = static_cast<uint8_t>(this->a_ * 255);
  val32 += val8;

  return val32;
}

//////////////////////////////////////////////////
Color::ARGB Color::asARGB() const
{
  uint8_t val8;
  unsigned int val32 = 0;

  // Convert to 32bit pattern
  // (ARGB = 8888)

  val8 = static_cast<uint8_t>(this->a_ * 255);
  val32 = val8 << 24;

  val8 = static_cast<uint8_t>(this->r_ * 255);
  val32 += val8 << 16;

  val8 = static_cast<uint8_t>(this->g_ * 255);
  val32 += val8 << 8;

  val8 = static_cast<uint8_t>(this->b_ * 255);
  val32 += val8;

  return val32;
}

//////////////////////////////////////////////////
Color::ABGR Color::asABGR() const
{
  uint8_t val8;
  unsigned int val32 = 0;

  // Convert to 32bit pattern
  // (ABGR = 8888)

  val8 = static_cast<uint8_t>(this->a_ * 255);
  val32 = val8 << 24;

  val8 = static_cast<uint8_t>(this->b_ * 255);
  val32 += val8 << 16;

  val8 = static_cast<uint8_t>(this->g_ * 255);
  val32 += val8 << 8;

  val8 = static_cast<uint8_t>(this->r_ * 255);
  val32 += val8;

  return val32;
}

//////////////////////////////////////////////////
void Color::setFromRGBA(RGBA v)
{
  unsigned int val32 = v;

  // Convert from 32bit pattern
  // (RGBA = 8888)

  this->r_ = ((val32 >> 24) & 0xFF) / 255.0f;
  this->g_ = ((val32 >> 16) & 0xFF) / 255.0f;
  this->b_ = ((val32 >> 8) & 0xFF) / 255.0f;
  this->a_ = (val32 & 0xFF) / 255.0f;
}

//////////////////////////////////////////////////
void Color::setFromBGRA(BGRA v)
{
  unsigned int val32 = v;

  // Convert from 32bit pattern
  // (BGRA = 8888)

  this->b_ = ((val32 >> 24) & 0xFF) / 255.0f;
  this->g_ = ((val32 >> 16) & 0xFF) / 255.0f;
  this->r_ = ((val32 >> 8) & 0xFF) / 255.0f;
  this->a_ = (val32 & 0xFF) / 255.0f;
}

//////////////////////////////////////////////////
void Color::setFromARGB(ARGB v)
{
  unsigned int val32 = v;

  // Convert from 32bit pattern
  // (ARGB = 8888)

  this->a_ = ((val32 >> 24) & 0xFF) / 255.0f;
  this->r_ = ((val32 >> 16) & 0xFF) / 255.0f;
  this->g_ = ((val32 >> 8) & 0xFF) / 255.0f;
  this->b_ = (val32 & 0xFF) / 255.0f;
}

//////////////////////////////////////////////////
void Color::setFromABGR(ABGR v)
{
  unsigned int val32 = v;

  // Convert from 32bit pattern
  // (ABGR = 8888)

  this->a_ = ((val32 >> 24) & 0xFF) / 255.0f;
  this->b_ = ((val32 >> 16) & 0xFF) / 255.0f;
  this->g_ = ((val32 >> 8) & 0xFF) / 255.0f;
  this->r_ = (val32 & 0xFF) / 255.0f;
}

//////////////////////////////////////////////////
Color Color::operator+(const Color& pt) const
{
  return Color(this->r_ + pt.r_, this->g_ + pt.g_, this->b_ + pt.b_, this->a_ + pt.a_);
}

//////////////////////////////////////////////////
Color Color::operator+(float v) const { return Color(this->r_ + v, this->g_ + v, this->b_ + v, this->a_ + v); }

//////////////////////////////////////////////////
const Color& Color::operator+=(const Color& pt)
{
  this->r_ += pt.r_;
  this->g_ += pt.g_;
  this->b_ += pt.b_;
  this->a_ += pt.a_;

  this->Clamp();

  return *this;
}

//////////////////////////////////////////////////
Color Color::operator-(const Color& pt) const
{
  return Color(this->r_ - pt.r_, this->g_ - pt.g_, this->b_ - pt.b_, this->a_ - pt.a_);
}

//////////////////////////////////////////////////
Color Color::operator-(float v) const { return Color(this->r_ - v, this->g_ - v, this->b_ - v, this->a_ - v); }

//////////////////////////////////////////////////
const Color& Color::operator-=(const Color& pt)
{
  this->r_ -= pt.r_;
  this->g_ -= pt.g_;
  this->b_ -= pt.b_;
  this->a_ -= pt.a_;

  this->Clamp();

  return *this;
}

//////////////////////////////////////////////////
const Color Color::operator/(float pt) const
{
  return Color(this->r_ / pt, this->g_ / pt, this->b_ / pt, this->a_ / pt);
}

//////////////////////////////////////////////////
const Color Color::operator/(const Color& _pt) const
{
  return Color(this->r_ / _pt.r_, this->g_ / _pt.g_, this->b_ / _pt.b_, this->a_ / _pt.a_);
}

//////////////////////////////////////////////////
const Color& Color::operator/=(const Color& pt)
{
  this->r_ /= pt.r_;
  this->g_ /= pt.g_;
  this->b_ /= pt.b_;
  this->a_ /= pt.a_;

  this->Clamp();

  return *this;
}

//////////////////////////////////////////////////
const Color Color::operator*(float pt) const
{
  return Color(this->r_ * pt, this->g_ * pt, this->b_ * pt, this->a_ * pt);
}

//////////////////////////////////////////////////
const Color Color::operator*(const Color& _pt) const
{
  return Color(this->r_ * _pt.r_, this->g_ * _pt.g_, this->b_ * _pt.b_, this->a_ * _pt.a_);
}

//////////////////////////////////////////////////
const Color& Color::operator*=(const Color& pt)
{
  this->r_ *= pt.r_;
  this->g_ *= pt.g_;
  this->b_ *= pt.b_;
  this->a_ *= pt.a_;

  this->Clamp();

  return *this;
}

//////////////////////////////////////////////////
bool Color::operator==(const Color& pt) const
{
  return almostEqualRelativeAndAbs(this->r_, pt.r_) && almostEqualRelativeAndAbs(this->g_, pt.g_) &&
         almostEqualRelativeAndAbs(this->b_, pt.b_) && almostEqualRelativeAndAbs(this->a_, pt.a_);
}

//////////////////////////////////////////////////
bool Color::operator!=(const Color& pt) const { return !(*this == pt); }

//////////////////////////////////////////////////
float Color::getR() const { return this->r_; }

//////////////////////////////////////////////////
float Color::getG() const { return this->g_; }

//////////////////////////////////////////////////
float Color::getB() const { return this->b_; }

//////////////////////////////////////////////////
float Color::getA() const { return this->a_; }

//////////////////////////////////////////////////
float& Color::getR() { return this->r_; }

//////////////////////////////////////////////////
float& Color::getG() { return this->g_; }

//////////////////////////////////////////////////
float& Color::getB() { return this->b_; }

//////////////////////////////////////////////////
float& Color::getA() { return this->a_; }

//////////////////////////////////////////////////
void Color::setR(float r) { this->r_ = r; }

//////////////////////////////////////////////////
void Color::setG(float g) { this->g_ = g; }

//////////////////////////////////////////////////
void Color::setB(float b) { this->b_ = b; }

//////////////////////////////////////////////////
void Color::setA(float a) { this->a_ = a; }
}  // namespace tesseract_common
