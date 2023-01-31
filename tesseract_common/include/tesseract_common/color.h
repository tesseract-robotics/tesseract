/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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
#ifndef TESSERACT_COMMON_COLOR_H
#define TESSERACT_COMMON_COLOR_H

#include <cctype>
#include <istream>
#include <ostream>

#include <Eigen/Core>

namespace tesseract_common
{
//
/// \class Color color.h
/// \brief Defines a color using a red (R), green (G), blue (B), and alpha
/// (A) component. Each color component is in the range [0..1].
///
/// ## Example
///
/// \snippet examples/color_example.cc complete
class Color
{
public:
  /// \brief (1, 1, 1)
  static const Color& White;
  /// \brief (0, 0, 0)
  static const Color& Black;
  /// \brief (1, 0, 0)
  static const Color& Red;
  /// \brief (0, 1, 0)
  static const Color& Green;
  /// \brief (0, 0, 1)
  static const Color& Blue;
  /// \brief (1, 1, 0)
  static const Color& Yellow;
  /// \brief (1, 0, 1)
  static const Color& Magenta;
  /// \brief (0, 1, 1)
  static const Color& Cyan;

  /// \typedef RGBA
  /// \brief A RGBA packed value as an unsigned int
  /// Each 8 bits corresponds to a channel.
  ///
  /// \code
  /// RGBA a = 0xFF0000FF; // (1, 0, 0, 1) for RGBA, i.e. red.
  /// \endcode
  using RGBA = unsigned int;

  /// \typedef BGRA
  /// \brief A BGRA packed value as an unsigned int
  /// Each 8 bits corresponds to a channel.
  ///
  /// \code
  /// BGRA a = 0xFF0000FF; // (0, 0, 1, 1) for RGBA, i.e. blue.
  /// \endcode
  using BGRA = unsigned int;

  /// \typedef ARGB
  /// \brief A ARGB packed value as an unsigned int
  /// Each 8 bits corresponds to a channel.
  ///
  /// \code
  /// ARGB a = 0xFF0000FF; // (0, 0, 1, 1) for RGBA, i.e. blue.
  /// \endcode
  using ARGB = unsigned int;

  /// \typedef ABGR
  /// \brief A ABGR packed value as an unsigned int
  /// Each 8 bits corresponds to a channel.
  ///
  /// \code
  /// ABGR a = 0xFF0000FF; // (1, 0, 0, 1) for RGBA, i.e. red.
  /// \endcode
  using ABGR = unsigned int;

  /// \brief Constructor
  Color() = default;

  /// \brief Constructor
  /// \param[in] r Red value (range 0 to 1)
  /// \param[in] g Green value (range 0 to 1)
  /// \param[in] b Blue value (range 0 to 1)
  /// \param[in] a Alpha value (0=transparent, 1=opaque)
  constexpr Color(float r, float g, float b, float a = 1.0) : r_(r), g_(g), b_(b), a_(a) { this->Clamp(); }

  /// \brief Copy Constructor
  /// \param[in] clr Color to copy
  Color(const Color& clr) = default;

  /// \brief Destructor
  ~Color() = default;

  /// \brief Reset the color to default values to red=0, green=0,
  /// blue=0, alpha=1.
  void reset();

  /// \brief Set the contents of the vector
  /// \param[in] r Red value (range 0 to 1)
  /// \param[in] g Green value (range 0 to 1)
  /// \param[in] b Blue value (range 0 to 1)
  /// \param[in] a Alpha value (0=transparent, 1=opaque)
  void set(float r = 1, float g = 1, float b = 1, float a = 1);

  /// \brief Get the color in HSV colorspace
  /// \return HSV values in a Vector3f format. A vector3f containing
  /// {NAN_F, NAN_F, NAN_F} is returned on error.
  Eigen::Vector3f getHSV() const;

  /// \brief Set a color based on HSV values
  /// \param[in] h Hue(0..360)
  /// \param[in] s Saturation(0..1)
  /// \param[in] v Value(0..1)
  void setFromHSV(float h, float s, float v);

  /// \brief Get the color in YUV colorspace
  /// \return the YUV  color
  Eigen::Vector3f getYUV() const;

  /// \brief Set from yuv
  /// \param[in] y value
  /// \param[in] u value
  /// \param[in] v value
  void setFromYUV(float y, float u, float v);

  /// \brief Equal operator
  /// \param[in] pt Color to copy
  /// \return Reference to this color
  Color& operator=(const Color& pt) = default;

  /// \brief Array index operator
  /// \param[in] index Color component index(0=red, 1=green, 2=blue,
  /// 3=alpha)
  /// \return r, g, b, or a when index is 0, 1, 2 or 3. A NAN_F value is
  /// returned if the index is invalid
  float operator[](const unsigned int index);

  /// \brief Array index operator, const version
  /// \param[in] index Color component index(0=red, 1=green, 2=blue,
  /// 3=alpha)
  /// \return r, g, b, or a when index is 0, 1, 2 or 3. A NAN_F value is
  /// returned if the index is invalid
  float operator[](const unsigned int index) const;

  /// \brief Get as uint32 RGBA packed value
  /// \return the color
  RGBA asRGBA() const;

  /// \brief Get as uint32 BGRA packed value
  /// \return the color
  BGRA asBGRA() const;

  /// \brief Get as uint32 ARGB packed value
  /// \return the color
  ARGB asARGB() const;

  /// \brief Get as uint32 ABGR packed value
  /// \return the color
  ABGR asABGR() const;

  /// \brief Set from uint32 RGBA packed value
  /// \param[in] v the new color
  void setFromRGBA(RGBA v);

  /// \brief Set from uint32 BGRA packed value
  /// \param[in] v the new color
  void setFromBGRA(BGRA v);

  /// \brief Set from uint32 ARGB packed value
  /// \param[in] v the new color
  void setFromARGB(ARGB v);

  /// \brief Set from uint32 ABGR packed value
  /// \param[in] v the new color
  void setFromABGR(ABGR v);

  /// \brief Addition operator (this + pt)
  /// \param[in] pt Color to add
  /// \return The resulting color
  Color operator+(const Color& pt) const;

  /// \brief Add v to all color components
  /// \param[in] v Value to add to each color component
  /// \return The resulting color
  Color operator+(float v) const;

  /// \brief Addition equal operator
  /// \param[in] pt Color to add
  /// \return The resulting color
  const Color& operator+=(const Color& pt);

  /// \brief Subtraction operator
  /// \param[in] pt The color to substract
  /// \return The resulting color
  Color operator-(const Color& pt) const;

  /// \brief Subtract v from all color components
  /// \param[in] v Value to subtract
  /// \return The resulting color
  Color operator-(float v) const;

  /// \brief Subtraction equal operator
  /// \param[in] pt Color to subtract
  /// \return The resulting color
  const Color& operator-=(const Color& pt);

  /// \brief Division operator
  /// \param[in] pt Color to divide by
  /// \return The resulting color
  const Color operator/(const Color& pt) const;

  /// \brief Divide all color component by v
  /// \param[in] v The value to divide by
  /// \return The resulting color
  const Color operator/(float v) const;

  /// \brief Division equal operator
  /// \param[in] pt Color to divide by
  /// \return The resulting color
  const Color& operator/=(const Color& pt);

  /// \brief Multiplication operator
  /// \param[in] pt The color to muliply by
  /// \return The resulting color
  const Color operator*(const Color& pt) const;

  /// \brief Multiply all color components by v
  /// \param[in] v The value to multiply by
  /// \return The resulting color
  const Color operator*(float v) const;

  /// \brief Multiplication equal operator
  /// \param[in] pt The color to muliply by
  /// \return The resulting color
  const Color& operator*=(const Color& pt);

  /// \brief Equality operator
  /// \param[in] pt The color to check for equality
  /// \return True if the this color equals pt
  bool operator==(const Color& pt) const;

  /// \brief Inequality operator
  /// \param[in] pt The color to check for inequality
  /// \return True if the this color does not equal pt
  bool operator!=(const Color& pt) const;

  /// \brief Stream insertion operator
  /// \param[in] out the output stream
  /// \param[in] color the color
  /// \return the output stream
  friend std::ostream& operator<<(std::ostream& out, const Color& color)
  {
    for (auto i : { 0, 1, 2, 3 })
    {
      if (i > 0)
        out << " ";

      //      appendToStream(out, color[i]);
    }
    return out;
  }

  /// \brief Stream insertion operator
  /// \param[in] in the input stream. If the input stream does not include
  /// an alpha value, a default alpha value of 1.0 will be used.
  /// \param[in] pt
  friend std::istream& operator>>(std::istream& in, Color& pt)
  {
    // Skip white spaces
    in.setf(std::ios_base::skipws);
    in >> pt.r_ >> pt.g_ >> pt.b_;
    // Since alpha is optional, check if it's there before parsing
    while (in.good() && std::isspace(in.peek()))
    {
      in.get();
    }
    if (in.good())
    {
      in >> pt.a_;
    }
    else if (!in.fail())
    {
      pt.a_ = 1.0;
    }
    return in;
  }

  /// \brief Get the red value
  /// \return The red value
  float getR() const;

  /// \brief Get the green value
  /// \return The green value
  float getG() const;

  /// \brief Get the blue value
  /// \return The blue value
  float getB() const;

  /// \brief Get the alpha value
  /// \return The alpha value
  float getA() const;

  /// \brief Get a mutable reference to the red value
  /// \return The red value
  float& getR();

  /// \brief Get a mutable reference to the green value
  /// \return The green value
  float& getG();

  /// \brief Get a mutable reference to the blue value
  /// \return The blue value
  float& getB();

  /// \brief Get a mutable reference to the alpha value
  /// \return The alpha value
  float& getA();

  /// \brief Set the red value
  /// \param[in] r New red value
  void setR(float r);

  /// \brief Set the green value
  /// \param[in] g New green value
  void setG(float g);

  /// \brief Set the blue value
  /// \param[in] b New blue value
  void setB(float b);

  /// \brief Set the alpha value
  /// \param[in] a New alpha value
  void setA(float a);

private:
  /// \brief Red value
  float r_ = 0;

  /// \brief Green value
  float g_ = 0;

  /// \brief Blue value
  float b_ = 0;

  /// \brief Alpha value
  float a_ = 1;

  /// \brief Clamp the color values to valid ranges
  constexpr void Clamp()
  {
    // These comparisons are carefully written to handle NaNs correctly.
    if (!(r_ >= 0))
      r_ = 0;

    if (!(g_ >= 0))
      g_ = 0;

    if (!(b_ >= 0))
      b_ = 0;

    if (!(a_ >= 0))
      a_ = 0;

    if (r_ > 1)
      r_ = r_ / 255.0f;

    if (g_ > 1)
      g_ = g_ / 255.0f;

    if (b_ > 1)
      b_ = b_ / 255.0f;

    if (a_ > 1)
      a_ = 1;
  }
};
}  // namespace tesseract_common
#endif  // TESSERACT_COMMON_COLOR_H
