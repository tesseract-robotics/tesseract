#include <tesseract_collision/vhacd/VHACD.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <thread>
#include <atomic>
#include <mutex>
#include <string>
#include <float.h>

#define ENABLE_ASYNC 1

#define HACD_ALLOC(x) malloc(x)
#define HACD_FREE(x) free(x)
#define HACD_ASSERT(x) assert(x)

namespace tesseract_collision
{
namespace VHACD
{
class MyHACD_API : public VHACD::IVHACD, VHACD::IVHACD::IUserCallback, VHACD::IVHACD::IUserLogger
{
public:
  MyHACD_API() { mVHACD = VHACD::CreateVHACD(); }
  virtual ~MyHACD_API() override
  {
    releaseHACD();
    Cancel();
    mVHACD->Release();
  }

  bool Compute(double const* _points,
               uint32_t countPoints,
               uint32_t const* _triangles,
               uint32_t countTriangles,
               const Parameters& _desc) override final
  {
#if ENABLE_ASYNC
    Cancel();  // if we previously had a solution running; cancel it.
    releaseHACD();

    // We need to copy the input vertices and triangles into our own buffers so we can operate
    // on them safely from the background thread.
    mVertices = (double*)HACD_ALLOC(sizeof(double) * countPoints * 3);
    mIndices = (uint32_t*)HACD_ALLOC(sizeof(uint32_t) * countTriangles * 3);
    memcpy(mVertices, _points, sizeof(double) * countPoints * 3);
    memcpy(mIndices, _triangles, sizeof(uint32_t) * countTriangles * 3);
    mRunning = true;
    mThread = new std::thread([this, countPoints, countTriangles, _desc]() {
      ComputeNow(mVertices, countPoints, mIndices, countTriangles, _desc);
      mRunning = false;
    });
#else
    releaseHACD();
    ComputeNow(_points, countPoints, _triangles, countTriangles, _desc);
#endif
    return true;
  }

  bool ComputeNow(double* const points,
                  uint32_t countPoints,
                  uint32_t* const triangles,
                  uint32_t countTriangles,
                  const Parameters& _desc)
  {
    uint32_t ret = 0;

    mHullCount = 0;
    mCallback = _desc.m_callback;
    mLogger = _desc.m_logger;

    IVHACD::Parameters desc = _desc;
    // Set our intercepting callback interfaces if non-null
    desc.m_callback = desc.m_callback ? this : nullptr;
    desc.m_logger = desc.m_logger ? this : nullptr;

    if (countPoints)
    {
      bool ok = mVHACD->Compute(points, countPoints, triangles, countTriangles, desc);
      if (ok)
      {
        ret = mVHACD->GetNConvexHulls();
        mHulls = new IVHACD::ConvexHull[ret];
        for (uint32_t i = 0; i < ret; i++)
        {
          VHACD::IVHACD::ConvexHull vhull;
          mVHACD->GetConvexHull(i, vhull);
          VHACD::IVHACD::ConvexHull h;
          h.m_nPoints = vhull.m_nPoints;
          h.m_points = (double*)HACD_ALLOC(sizeof(double) * 3 * h.m_nPoints);
          memcpy(h.m_points, vhull.m_points, sizeof(double) * 3 * h.m_nPoints);
          h.m_nTriangles = vhull.m_nTriangles;
          h.m_triangles = (uint32_t*)HACD_ALLOC(sizeof(uint32_t) * 3 * h.m_nTriangles);
          memcpy(h.m_triangles, vhull.m_triangles, sizeof(uint32_t) * 3 * h.m_nTriangles);
          h.m_volume = vhull.m_volume;
          h.m_center[0] = vhull.m_center[0];
          h.m_center[1] = vhull.m_center[1];
          h.m_center[2] = vhull.m_center[2];
          mHulls[i] = h;
          if (mCancel)
          {
            ret = 0;
            break;
          }
        }
      }
    }

    mHullCount = ret;
    return ret ? true : false;
  }

  void releaseHull(VHACD::IVHACD::ConvexHull& h)
  {
    HACD_FREE((void*)h.m_triangles);
    HACD_FREE((void*)h.m_points);
    h.m_triangles = nullptr;
    h.m_points = nullptr;
  }

  void GetConvexHull(uint32_t index, VHACD::IVHACD::ConvexHull& ch) const override final
  {
    if (index < mHullCount)
    {
      ch = mHulls[index];
    }
  }

  void releaseHACD()  // release memory associated with the last HACD request
  {
    for (uint32_t i = 0; i < mHullCount; i++)
    {
      releaseHull(mHulls[i]);
    }
    delete[] mHulls;
    mHulls = nullptr;
    mHullCount = 0;
    HACD_FREE(mVertices);
    mVertices = nullptr;
    HACD_FREE(mIndices);
    mIndices = nullptr;
  }

  virtual void release()  // release the HACD_API interface
  {
    delete this;
  }

  virtual uint32_t getHullCount() { return mHullCount; }
  virtual void Cancel() override final
  {
    if (mRunning)
    {
      mVHACD->Cancel();  // Set the cancel signal to the base VHACD
    }
    if (mThread)
    {
      mThread->join();  // Wait for the thread to fully exit before we delete the instance
      delete mThread;
      mThread = nullptr;
      Log("Convex Decomposition thread canceled\n");
    }
    mCancel = false;  // clear the cancel semaphore
  }

  bool Compute(float const* points,
               uint32_t countPoints,
               uint32_t const* triangles,
               uint32_t countTriangles,
               const Parameters& params) override final
  {
    double* vertices = (double*)HACD_ALLOC(sizeof(double) * countPoints * 3);
    const float* source = points;
    double* dest = vertices;
    for (uint32_t i = 0; i < countPoints; i++)
    {
      dest[0] = source[0];
      dest[1] = source[1];
      dest[2] = source[2];
      dest += 3;
      source += 3;
    }

    bool ret = Compute(vertices, countPoints, triangles, countTriangles, params);
    HACD_FREE(vertices);
    return ret;
  }

  uint32_t GetNConvexHulls() const override final
  {
    processPendingMessages();
    return mHullCount;
  }

  void Clean() override final  // release internally allocated memory
  {
    Cancel();
    releaseHACD();
    mVHACD->Clean();
  }

  void Release() override final  // release IVHACD
  {
    delete this;
  }

  bool OCLInit(void const* oclDevice, IVHACD::IUserLogger const* logger = nullptr) override final
  {
    return mVHACD->OCLInit(oclDevice, logger);
  }

  bool OCLRelease(IVHACD::IUserLogger const* logger = nullptr) override final { return mVHACD->OCLRelease(logger); }
  void Update(double overallProgress,
              double stageProgress,
              double operationProgress,
              const std::string& stage,
              const std::string& operation) override final
  {
    mMessageMutex.lock();
    mHaveUpdateMessage = true;
    mOverallProgress = overallProgress;
    mStageProgress = stageProgress;
    mOperationProgress = operationProgress;
    mStage = std::string(stage);
    mOperation = std::string(operation);
    mMessageMutex.unlock();
  }

  void Log(const std::string& msg) const override final
  {
    mMessageMutex.lock();
    mHaveLogMessage = true;
    mMessage = msg;
    mMessageMutex.unlock();
  }

  bool IsReady(void) const override final
  {
    processPendingMessages();
    return !mRunning;
  }

  // As a convenience for the calling application we only send it update and log messages from it's own main
  // thread.  This reduces the complexity burden on the caller by making sure it only has to deal with log
  // messages in it's main application thread.
  void processPendingMessages() const
  {
    // If we have a new update message and the user has specified a callback we send the message and clear the semaphore
    if (mHaveUpdateMessage && mCallback)
    {
      mMessageMutex.lock();
      mCallback->Update(mOverallProgress, mStageProgress, mOperationProgress, mStage, mOperation);
      mHaveUpdateMessage = false;
      mMessageMutex.unlock();
    }
    // If we have a new log message and the user has specified a callback we send the message and clear the semaphore
    if (mHaveLogMessage && mLogger)
    {
      mMessageMutex.lock();
      mLogger->Log(mMessage);
      mHaveLogMessage = false;
      mMessageMutex.unlock();
    }
  }

  // Will compute the center of mass of the convex hull decomposition results and return it
  // in 'centerOfMass'.  Returns false if the center of mass could not be computed.
  bool ComputeCenterOfMass(std::array<double, 3>& centerOfMass) const override
  {
    bool ret = false;

    centerOfMass[0] = 0;
    centerOfMass[1] = 0;
    centerOfMass[2] = 0;

    if (mVHACD && IsReady())
    {
      ret = mVHACD->ComputeCenterOfMass(centerOfMass);
    }
    return ret;
  }

private:
  double* mVertices{ nullptr };
  uint32_t* mIndices{ nullptr };
  std::atomic<uint32_t> mHullCount{ 0 };
  VHACD::IVHACD::ConvexHull* mHulls{ nullptr };
  VHACD::IVHACD::IUserCallback* mCallback{ nullptr };
  VHACD::IVHACD::IUserLogger* mLogger{ nullptr };
  VHACD::IVHACD* mVHACD{ nullptr };
  std::thread* mThread{ nullptr };
  std::atomic<bool> mRunning{ false };
  std::atomic<bool> mCancel{ false };

  // Thread safe caching mechanism for messages and update status.
  // This is so that caller always gets messages in his own thread
  // Member variables are marked as 'mutable' since the message dispatch function
  // is called from const query methods.
  mutable std::mutex mMessageMutex;
  mutable std::atomic<bool> mHaveUpdateMessage{ false };
  mutable std::atomic<bool> mHaveLogMessage{ false };
  mutable double mOverallProgress{ 0 };
  mutable double mStageProgress{ 0 };
  mutable double mOperationProgress{ 0 };
  mutable std::string mStage;
  mutable std::string mOperation;
  mutable std::string mMessage;
};

IVHACD* CreateVHACD_ASYNC(void)
{
  MyHACD_API* m = new MyHACD_API;
  return static_cast<IVHACD*>(m);
}

}  // namespace VHACD
}  // namespace tesseract_collision
