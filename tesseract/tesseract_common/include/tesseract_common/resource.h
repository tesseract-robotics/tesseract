#ifndef TESSERACT_COMMON_RESOURCE_H
#define TESSERACT_COMMON_RESOURCE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_common
{
// Represents resource data available from a file or url
class Resource
{
public:
  using Ptr = std::shared_ptr<Resource>;
  using ConstPtr = std::shared_ptr<const Resource>;

  virtual bool isFile() = 0;

  virtual std::string getUrl() = 0;

  virtual std::string getFilePath() = 0;

  virtual std::vector<uint8_t> getResourceContents() = 0;

  virtual std::shared_ptr<std::istream> getResourceContentStream() = 0;
};

}  // namespace tesseract_common
#endif  // TESSERACT_COMMON_RESOURCE_H