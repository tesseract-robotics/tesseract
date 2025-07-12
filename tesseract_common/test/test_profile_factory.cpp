#include "test_profile.h"
#include <tesseract_common/profile_plugin_factory.h>
#include <yaml-cpp/yaml.h>

namespace tesseract_common
{
class TestProfileFactory : public ProfileFactory
{
public:
  std::unique_ptr<Profile> create(const std::string& /*name*/,
                                  const YAML::Node& config,
                                  const std::shared_ptr<const ProfileFactoryData>& /*data*/,
                                  const ProfilePluginFactory& /*plugin_factory*/) const override final
  {
    auto profile = std::make_unique<TestProfile>();

    if (YAML::Node n = config["enabled"])
      profile->enabled = n.as<bool>();

    return profile;
  };
};
}  // namespace tesseract_common

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_PROFILE_PLUGIN(tesseract_common::TestProfileFactory, TestProfileFactory)
