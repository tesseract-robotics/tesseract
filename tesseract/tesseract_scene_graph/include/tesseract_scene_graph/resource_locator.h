#ifndef TESSERACT_SCENE_GRAPH_RESOURCE_LOCATOR_H
#define TESSERACT_SCENE_GRAPH_RESOURCE_LOCATOR_H

#include <tesseract_common/resource.h>
#include <tesseract_scene_graph/allowed_collision_matrix.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/parser/srdf_parser.h>
#include <functional>

namespace tesseract_scene_graph
{
    // ResourceLocator abstract class
    class ResourceLocator
    {
    public:
        using Ptr = std::shared_ptr<ResourceLocator>;
        using ConstPtr = std::shared_ptr<const ResourceLocator>;

        virtual tesseract_common::Resource::Ptr LocateResource(const std::string& url) = 0;
    };

    // SimpleResourceLocator using a function to resolve filenames
    class SimpleResourceLocator : public ResourceLocator
    {
    public:

        using Ptr = std::shared_ptr<SimpleResourceLocator>;
        using ConstPtr = std::shared_ptr<const SimpleResourceLocator>;

        using ResourceLocatorFn = std::function<std::string(const std::string&)>;
        
        SimpleResourceLocator(ResourceLocatorFn locator_function);

        virtual tesseract_common::Resource::Ptr LocateResource(const std::string& url) override;

    protected:
        ResourceLocatorFn locator_function_;        
    };


    // SimpleLocatedResource for file
    class SimpleLocatedResource : public tesseract_common::Resource
    {
    public:
        using Ptr = std::shared_ptr<SimpleLocatedResource>;
        using ConstPtr = std::shared_ptr<const SimpleLocatedResource>;

        SimpleLocatedResource(const std::string& url, const std::string& filename);

        virtual bool IsFile() override;

        virtual std::string GetUrl() override;

        virtual std::string GetFilename() override;

        virtual std::vector<uint8_t> GetResourceContents() override;

        virtual std::shared_ptr<std::istream> GetResourceContentStream() override;

    protected:
        std::string url_;
        std::string filename_;
    };


}  // namespace tesseract_scene_graph

#endif  // TESSERACT_SCENE_GRAPH_RESOURCE_LOCATOR_H