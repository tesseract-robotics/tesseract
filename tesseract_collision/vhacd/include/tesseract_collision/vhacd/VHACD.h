/* Copyright (c) 2011 Khaled Mamou (kmamou at gmail dot com)
 All rights reserved.


 Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
 disclaimer in the documentation and/or other materials provided with the distribution.

 3. The names of the contributors may not be used to endorse or promote products derived from this software without
 specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#pragma once

#define VHACD_VERSION_MAJOR 2
#define VHACD_VERSION_MINOR 3

// Changes for version 2.3
//
// m_gamma : Has been removed.  This used to control the error metric to merge convex hulls.  Now it uses the
// 'm_maxConvexHulls' value instead.
// m_maxConvexHulls : This is the maximum number of convex hulls to produce from the merge operation; replaces
// 'm_gamma'.
//
// Note that decomposition depth is no longer a user provided value.  It is now derived from the
// maximum number of hulls requested.
//
// As a convenience to the user, each convex hull produced now includes the volume of the hull as well as it's center.
//
// This version supports a convenience method to automatically make V-HACD run asynchronously in a background thread.
// To get a fully asynchronous version, call 'CreateVHACD_ASYNC' instead of 'CreateVHACD'.  You get the same interface
// however,
// now when computing convex hulls, it is no longer a blocking operation.  All callback messages are still returned
// in the application's thread so you don't need to worry about mutex locks or anything in that case.
// To tell if the operation is complete, the application should call 'IsReady'.  This will return true if
// the last approximation operation is complete and will dispatch any pending messages.
// If you call 'Compute' while a previous operation was still running, it will automatically cancel the last request
// and begin a new one.  To cancel a currently running approximation just call 'Cancel'.
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <cstdint>
#include <array>
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_collision::VHACD
{
class IVHACD
{
public:
  IVHACD() = default;
  virtual ~IVHACD() = default;
  IVHACD(const IVHACD&) = default;
  IVHACD& operator=(const IVHACD&) = default;
  IVHACD(IVHACD&&) = default;
  IVHACD& operator=(IVHACD&&) = default;

  class IUserCallback
  {
  public:
    IUserCallback() = default;
    virtual ~IUserCallback() = default;
    IUserCallback(const IUserCallback&) = default;
    IUserCallback& operator=(const IUserCallback&) = default;
    IUserCallback(IUserCallback&&) = default;
    IUserCallback& operator=(IUserCallback&&) = default;

    virtual void Update(double overallProgress,
                        double stageProgress,
                        double operationProgress,
                        const std::string& stage,
                        const std::string& operation) = 0;
  };

  class IUserLogger
  {
  public:
    IUserLogger() = default;
    virtual ~IUserLogger() = default;
    IUserLogger(const IUserLogger&) = default;
    IUserLogger& operator=(const IUserLogger&) = default;
    IUserLogger(IUserLogger&&) = default;
    IUserLogger& operator=(IUserLogger&&) = default;

    virtual void Log(const std::string& msg) const = 0;
  };

  class ConvexHull
  {
  public:
    double* m_points;
    uint32_t* m_triangles;
    uint32_t m_nPoints;
    uint32_t m_nTriangles;
    double m_volume;
    std::array<double, 3> m_center;
  };

  class Parameters
  {
  public:
    Parameters() { Init(); }
    void Init()
    {
      m_resolution = 100000;
      m_concavity = 0.001;
      m_planeDownsampling = 4;
      m_convexhullDownsampling = 4;
      m_alpha = 0.05;
      m_beta = 0.05;
      m_pca = 0;
      m_mode = 0;  // 0: voxel-based (recommended), 1: tetrahedron-based
      m_maxNumVerticesPerCH = 64;
      m_minVolumePerCH = 0.0001;
      m_callback = nullptr;
      m_logger = nullptr;
      m_convexhullApproximation = 1U;
      m_oclAcceleration = 1U;
      m_maxConvexHulls = 1024;
      m_projectHullVertices = true;  // This will project the output convex hull vertices onto the original source mesh
                                     // to increase the floating point accuracy of the results
    }
    double m_concavity{ 0.001 };
    double m_alpha{ 0.05 };
    double m_beta{ 0.05 };
    double m_minVolumePerCH{ 0.0001 };
    IUserCallback* m_callback{ nullptr };
    IUserLogger* m_logger{ nullptr };
    uint32_t m_resolution{ 100000 };
    uint32_t m_maxNumVerticesPerCH{ 64 };
    uint32_t m_planeDownsampling{ 4 };
    uint32_t m_convexhullDownsampling{ 4 };
    uint32_t m_pca{ 0 };
    uint32_t m_mode{ 0 };
    uint32_t m_convexhullApproximation{ 1U };
    uint32_t m_oclAcceleration{ 1U };
    uint32_t m_maxConvexHulls{ 1024 };
    bool m_projectHullVertices{ true };
  };

  virtual void Cancel() = 0;
  virtual bool Compute(float const* points,
                       uint32_t countPoints,
                       uint32_t const* triangles,
                       uint32_t countTriangles,
                       const Parameters& params) = 0;
  virtual bool Compute(double const* points,
                       uint32_t countPoints,
                       uint32_t const* triangles,
                       uint32_t countTriangles,
                       const Parameters& params) = 0;
  virtual uint32_t GetNConvexHulls() const = 0;
  virtual void GetConvexHull(uint32_t index, ConvexHull& ch) const = 0;
  virtual void Clean() = 0;    // release internally allocated memory
  virtual void Release() = 0;  // release IVHACD
  virtual bool OCLInit(void const* oclDevice, IUserLogger const* logger = nullptr) = 0;
  virtual bool OCLRelease(IUserLogger const* logger = nullptr) = 0;

  // Will compute the center of mass of the convex hull decomposition results and return it
  // in 'centerOfMass'.  Returns false if the center of mass could not be computed.
  virtual bool ComputeCenterOfMass(std::array<double, 3>& centerOfMass) const = 0;

  // In synchronous mode (non-multi-threaded) the state is always 'ready'
  // In asynchronous mode, this returns true if the background thread is not still actively computing
  // a new solution.  In an asynchronous config the 'IsReady' call will report any update or log
  // messages in the caller's current thread.
  virtual bool IsReady() const { return true; }
};
IVHACD* CreateVHACD();
IVHACD* CreateVHACD_ASYNC();
}  // namespace tesseract_collision::VHACD
