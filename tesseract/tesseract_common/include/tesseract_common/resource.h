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
        
        virtual bool IsFile() = 0;

        virtual std::string GetUrl() = 0;

        virtual std::string GetFilePath() = 0;

        virtual std::vector<uint8_t> GetResourceContents() = 0;

        virtual std::shared_ptr<std::istream> GetResourceContentStream() = 0;
    };

}  // namespace tesseract_common
#endif  // TESSERACT_COMMON_RESOURCE_H