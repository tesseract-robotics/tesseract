# Migration Guide

## Single Package with Components

Previously, tesseract was split into many separate CMake packages (`tesseract_common`, `tesseract_collision`, `tesseract_kinematics`, etc.). Now it is a **single CMake package** called `tesseract` with components.

## Header Path Changes

All `#include` paths have changed from `tesseract_<name>/` to `tesseract/<name>/`:

| Old Path | New Path |
|----------|----------|
| `tesseract_common/` | `tesseract/common/` |
| `tesseract_geometry/` | `tesseract/geometry/` |
| `tesseract_scene_graph/` | `tesseract/scene_graph/` |
| `tesseract_state_solver/` | `tesseract/state_solver/` |
| `tesseract_collision/` | `tesseract/collision/` |
| `tesseract_srdf/` | `tesseract/srdf/` |
| `tesseract_urdf/` | `tesseract/urdf/` |
| `tesseract_environment/` | `tesseract/environment/` |
| `tesseract_visualization/` | `tesseract/visualization/` |
| `tesseract_kinematics/` | `tesseract/kinematics/` |
| `tesseract_support/` | `tesseract/support/` |

Additionally, the `core/` subdirectory has been flattened for collision and kinematics. Headers that were previously under `core/` are now at the component level:

| Old Path | New Path |
|----------|----------|
| `tesseract/collision/core/types.h` | `tesseract/collision/types.h` |
| `tesseract/collision/core/common.h` | `tesseract/collision/common.h` |
| `tesseract/collision/core/discrete_contact_manager.h` | `tesseract/collision/discrete_contact_manager.h` |
| `tesseract/collision/core/continuous_contact_manager.h` | `tesseract/collision/continuous_contact_manager.h` |
| `tesseract/collision/core/contact_managers_plugin_factory.h` | `tesseract/collision/contact_managers_plugin_factory.h` |
| `tesseract/kinematics/core/joint_group.h` | `tesseract/kinematics/joint_group.h` |
| `tesseract/kinematics/core/kinematic_group.h` | `tesseract/kinematics/kinematic_group.h` |
| `tesseract/kinematics/core/forward_kinematics.h` | `tesseract/kinematics/forward_kinematics.h` |
| `tesseract/kinematics/core/inverse_kinematics.h` | `tesseract/kinematics/inverse_kinematics.h` |
| `tesseract/kinematics/core/kinematics_plugin_factory.h` | `tesseract/kinematics/kinematics_plugin_factory.h` |

This applies to **all** headers formerly in `collision/core/` and `kinematics/core/` — the tables above show representative examples.

### Examples

```cpp
// Before
#include <tesseract_common/types.h>
#include <tesseract_collision/core/types.h>
#include <tesseract_environment/environment.h>
#include <tesseract_kinematics/core/joint_group.h>

// After
#include <tesseract/common/types.h>
#include <tesseract/collision/types.h>
#include <tesseract/environment/environment.h>
#include <tesseract/kinematics/joint_group.h>
```

## CMake Target Name Changes

All CMake targets drop the `tesseract_` prefix (namespace remains `tesseract::`):

| Old Target | New Target |
|------------|------------|
| `tesseract::tesseract_common` | `tesseract::common` |
| `tesseract::tesseract_geometry` | `tesseract::geometry` |
| `tesseract::tesseract_scene_graph` | `tesseract::scene_graph` |
| `tesseract::tesseract_state_solver_core` | `tesseract::state_solver` |
| `tesseract::tesseract_state_solver_kdl` | `tesseract::state_solver_kdl` |
| `tesseract::tesseract_state_solver_ofkt` | `tesseract::state_solver_ofkt` |
| `tesseract::tesseract_collision_core` | `tesseract::collision` |
| `tesseract::tesseract_collision_bullet` | `tesseract::collision_bullet` |
| `tesseract::tesseract_collision_bullet_factories` | `tesseract::collision_bullet_factories` |
| `tesseract::tesseract_collision_hacd_convex_decomposition` | `tesseract::collision_hacd_convex_decomposition` |
| `tesseract::tesseract_collision_fcl` | `tesseract::collision_fcl` |
| `tesseract::tesseract_collision_fcl_factories` | `tesseract::collision_fcl_factories` |
| `tesseract::tesseract_collision_vhacd_convex_decomposition` | `tesseract::collision_vhacd_convex_decomposition` |
| `tesseract::tesseract_collision_test_suite` | `tesseract::collision_test_suite` |
| `tesseract::tesseract_collision_test_suite_benchmarks` | `tesseract::collision_test_suite_benchmarks` |
| `tesseract::tesseract_srdf` | `tesseract::srdf` |
| `tesseract::tesseract_urdf` | `tesseract::urdf` |
| `tesseract::tesseract_kinematics_core` | `tesseract::kinematics` |
| `tesseract::tesseract_kinematics_core_factories` | `tesseract::kinematics_factories` |
| `tesseract::tesseract_kinematics_kdl` | `tesseract::kinematics_kdl` |
| `tesseract::tesseract_kinematics_kdl_factories` | `tesseract::kinematics_kdl_factories` |
| `tesseract::tesseract_kinematics_opw` | `tesseract::kinematics_opw` |
| `tesseract::tesseract_kinematics_opw_factory` | `tesseract::kinematics_opw_factory` |
| `tesseract::tesseract_kinematics_ur` | `tesseract::kinematics_ur` |
| `tesseract::tesseract_kinematics_ur_factory` | `tesseract::kinematics_ur_factory` |
| `tesseract::tesseract_kinematics_ikfast` | `tesseract::kinematics_ikfast` |
| `tesseract::tesseract_environment` | `tesseract::environment` |
| `tesseract::tesseract_visualization` | `tesseract::visualization` |

## CMake `find_package` Changes

All separate `find_package` calls are replaced with a single `find_package(tesseract ...)` using components.

The supported top-level components are: `common`, `geometry`, `scene_graph`, `state_solver`, `collision`, `srdf`, `urdf`, `kinematics`, `environment`, `visualization`, `support`.

Meta-components (`collision`, `state_solver`, `kinematics`) pull in all their sub-components. You can also request individual sub-components directly (e.g., `collision`, `collision_bullet`, `kinematics`).

| Old | New |
|-----|-----|
| `find_package(tesseract_common REQUIRED)` | `find_package(tesseract REQUIRED COMPONENTS common)` |
| `find_package(tesseract_geometry REQUIRED)` | `find_package(tesseract REQUIRED COMPONENTS geometry)` |
| `find_package(tesseract_scene_graph REQUIRED)` | `find_package(tesseract REQUIRED COMPONENTS scene_graph)` |
| `find_package(tesseract_state_solver REQUIRED)` | `find_package(tesseract REQUIRED COMPONENTS state_solver)` |
| `find_package(tesseract_collision REQUIRED COMPONENTS core bullet)` | `find_package(tesseract REQUIRED COMPONENTS collision collision_bullet)` |
| `find_package(tesseract_collision REQUIRED)` | `find_package(tesseract REQUIRED COMPONENTS collision)` |
| `find_package(tesseract_srdf REQUIRED)` | `find_package(tesseract REQUIRED COMPONENTS srdf)` |
| `find_package(tesseract_urdf REQUIRED)` | `find_package(tesseract REQUIRED COMPONENTS urdf)` |
| `find_package(tesseract_kinematics REQUIRED COMPONENTS core kdl)` | `find_package(tesseract REQUIRED COMPONENTS kinematics kinematics_kdl)` |
| `find_package(tesseract_kinematics REQUIRED)` | `find_package(tesseract REQUIRED COMPONENTS kinematics)` |
| `find_package(tesseract_environment REQUIRED)` | `find_package(tesseract REQUIRED COMPONENTS environment)` |
| `find_package(tesseract_visualization REQUIRED)` | `find_package(tesseract REQUIRED COMPONENTS visualization)` |
| `find_package(tesseract_support REQUIRED)` | `find_package(tesseract REQUIRED COMPONENTS support)` |

### Examples

```cmake
# Before
find_package(tesseract_common REQUIRED)
find_package(tesseract_collision REQUIRED COMPONENTS core bullet)
find_package(tesseract_kinematics REQUIRED COMPONENTS core kdl)
find_package(tesseract_environment REQUIRED)

# After
find_package(tesseract REQUIRED COMPONENTS common collision collision_bullet kinematics kinematics_kdl environment)
```

## Migration Script

Run the following script from the root of your project to update all C/C++ source files, headers, and CMake files:

```bash
#!/usr/bin/env bash
# migrate_tesseract.sh — Update tesseract include paths, CMake targets, and find_package calls.
# Usage: ./migrate_tesseract.sh /path/to/your/project
set -euo pipefail

DIR="${1:-.}"

echo "=== Updating #include paths in source/header files ==="
find "$DIR" -type f \( -name '*.h' -o -name '*.hpp' -o -name '*.cpp' -o -name '*.cc' -o -name '*.cxx' -o -name '*.hxx' \) \
  -not -path '*/.git/*' \
  -exec sed -i \
    -e 's|tesseract_common/|tesseract/common/|g' \
    -e 's|tesseract_geometry/|tesseract/geometry/|g' \
    -e 's|tesseract_scene_graph/|tesseract/scene_graph/|g' \
    -e 's|tesseract_state_solver/|tesseract/state_solver/|g' \
    -e 's|tesseract_collision/|tesseract/collision/|g' \
    -e 's|tesseract_srdf/|tesseract/srdf/|g' \
    -e 's|tesseract_urdf/|tesseract/urdf/|g' \
    -e 's|tesseract_environment/|tesseract/environment/|g' \
    -e 's|tesseract_visualization/|tesseract/visualization/|g' \
    -e 's|tesseract_kinematics/|tesseract/kinematics/|g' \
    -e 's|tesseract_support/|tesseract/support/|g' \
    {} +
echo "  Done."

echo "=== Flattening core/ subdirectories in #include paths ==="
find "$DIR" -type f \( -name '*.h' -o -name '*.hpp' -o -name '*.cpp' -o -name '*.cc' -o -name '*.cxx' -o -name '*.hxx' \) \
  -not -path '*/.git/*' \
  -exec sed -i \
    -e 's|tesseract/collision/core/|tesseract/collision/|g' \
    -e 's|tesseract/kinematics/core/|tesseract/kinematics/|g' \
    {} +
echo "  Done."

echo "=== Updating CMake namespaced targets ==="
find "$DIR" -type f \( -name 'CMakeLists.txt' -o -name '*.cmake' -o -name '*.cmake.in' \) \
  -not -path '*/.git/*' \
  -exec sed -i \
    -e 's|tesseract::tesseract_collision_bullet_factories|tesseract::collision_bullet_factories|g' \
    -e 's|tesseract::tesseract_collision_fcl_factories|tesseract::collision_fcl_factories|g' \
    -e 's|tesseract::tesseract_collision_hacd_convex_decomposition|tesseract::collision_hacd_convex_decomposition|g' \
    -e 's|tesseract::tesseract_collision_vhacd_convex_decomposition|tesseract::collision_vhacd_convex_decomposition|g' \
    -e 's|tesseract::tesseract_collision_test_suite_benchmarks|tesseract::collision_test_suite_benchmarks|g' \
    -e 's|tesseract::tesseract_collision_test_suite|tesseract::collision_test_suite|g' \
    -e 's|tesseract::tesseract_collision_bullet|tesseract::collision_bullet|g' \
    -e 's|tesseract::tesseract_collision_fcl|tesseract::collision_fcl|g' \
    -e 's|tesseract::tesseract_collision_vhacd|tesseract::collision_vhacd|g' \
    -e 's|tesseract::tesseract_collision_core|tesseract::collision|g' \
    -e 's|tesseract::tesseract_kinematics_core_factories|tesseract::kinematics_factories|g' \
    -e 's|tesseract::tesseract_kinematics_kdl_factories|tesseract::kinematics_kdl_factories|g' \
    -e 's|tesseract::tesseract_kinematics_opw_factory|tesseract::kinematics_opw_factory|g' \
    -e 's|tesseract::tesseract_kinematics_ur_factory|tesseract::kinematics_ur_factory|g' \
    -e 's|tesseract::tesseract_kinematics_core|tesseract::kinematics|g' \
    -e 's|tesseract::tesseract_kinematics_kdl|tesseract::kinematics_kdl|g' \
    -e 's|tesseract::tesseract_kinematics_opw|tesseract::kinematics_opw|g' \
    -e 's|tesseract::tesseract_kinematics_ur|tesseract::kinematics_ur|g' \
    -e 's|tesseract::tesseract_kinematics_ikfast|tesseract::kinematics_ikfast|g' \
    -e 's|tesseract::tesseract_state_solver_core|tesseract::state_solver|g' \
    -e 's|tesseract::tesseract_state_solver_kdl|tesseract::state_solver_kdl|g' \
    -e 's|tesseract::tesseract_state_solver_ofkt|tesseract::state_solver_ofkt|g' \
    -e 's|tesseract::tesseract_common|tesseract::common|g' \
    -e 's|tesseract::tesseract_geometry|tesseract::geometry|g' \
    -e 's|tesseract::tesseract_scene_graph|tesseract::scene_graph|g' \
    -e 's|tesseract::tesseract_srdf|tesseract::srdf|g' \
    -e 's|tesseract::tesseract_urdf|tesseract::urdf|g' \
    -e 's|tesseract::tesseract_environment|tesseract::environment|g' \
    -e 's|tesseract::tesseract_visualization|tesseract::visualization|g' \
    {} +
echo "  Done."

echo "=== Updating C++ namespace qualifiers ==="
find "$DIR" -type f \( -name '*.h' -o -name '*.hpp' -o -name '*.cpp' -o -name '*.cc' -o -name '*.cxx' -o -name '*.hxx' \) \
  -not -path '*/.git/*' \
  -exec sed -i \
    -e 's|tesseract_common::|tesseract::common::|g' \
    -e 's|tesseract_geometry::|tesseract::geometry::|g' \
    -e 's|tesseract_scene_graph::|tesseract::scene_graph::|g' \
    -e 's|tesseract_state_solver::|tesseract::state_solver::|g' \
    -e 's|tesseract_collision::|tesseract::collision::|g' \
    -e 's|tesseract_srdf::|tesseract::srdf::|g' \
    -e 's|tesseract_urdf::|tesseract::urdf::|g' \
    -e 's|tesseract_environment::|tesseract::environment::|g' \
    -e 's|tesseract_visualization::|tesseract::visualization::|g' \
    -e 's|tesseract_kinematics::|tesseract::kinematics::|g' \
    -e 's|using namespace tesseract_common;|using namespace tesseract::common;|g' \
    -e 's|using namespace tesseract_geometry;|using namespace tesseract::geometry;|g' \
    -e 's|using namespace tesseract_scene_graph;|using namespace tesseract::scene_graph;|g' \
    -e 's|using namespace tesseract_state_solver;|using namespace tesseract::state_solver;|g' \
    -e 's|using namespace tesseract_collision;|using namespace tesseract::collision;|g' \
    -e 's|using namespace tesseract_srdf;|using namespace tesseract::srdf;|g' \
    -e 's|using namespace tesseract_urdf;|using namespace tesseract::urdf;|g' \
    -e 's|using namespace tesseract_environment;|using namespace tesseract::environment;|g' \
    -e 's|using namespace tesseract_visualization;|using namespace tesseract::visualization;|g' \
    -e 's|using namespace tesseract_kinematics;|using namespace tesseract::kinematics;|g' \
    {} +
echo "  Done."

echo "=== Updating find_package() and find_dependency() calls ==="
# NOTE: find_package/find_dependency calls now use a single 'tesseract' package with COMPONENTS.
# The old per-package calls (e.g. find_package(tesseract_common REQUIRED))
# must be converted to: find_package(tesseract REQUIRED COMPONENTS common ...)
#
# The sed rules below handle the simple case of single-component calls.
# For calls with COMPONENTS (e.g. find_package(tesseract_collision REQUIRED COMPONENTS core bullet)),
# manual conversion is required to flatten them (e.g. COMPONENTS collision collision_bullet).
find "$DIR" -type f \( -name 'CMakeLists.txt' -o -name '*.cmake' -o -name '*.cmake.in' \) \
  -not -path '*/.git/*' \
  -exec sed -i \
    -e 's|find_package(tesseract_common REQUIRED)|find_package(tesseract REQUIRED COMPONENTS common)|g' \
    -e 's|find_package(tesseract_geometry REQUIRED)|find_package(tesseract REQUIRED COMPONENTS geometry)|g' \
    -e 's|find_package(tesseract_scene_graph REQUIRED)|find_package(tesseract REQUIRED COMPONENTS scene_graph)|g' \
    -e 's|find_package(tesseract_state_solver REQUIRED)|find_package(tesseract REQUIRED COMPONENTS state_solver)|g' \
    -e 's|find_package(tesseract_collision REQUIRED)|find_package(tesseract REQUIRED COMPONENTS collision)|g' \
    -e 's|find_package(tesseract_srdf REQUIRED)|find_package(tesseract REQUIRED COMPONENTS srdf)|g' \
    -e 's|find_package(tesseract_urdf REQUIRED)|find_package(tesseract REQUIRED COMPONENTS urdf)|g' \
    -e 's|find_package(tesseract_environment REQUIRED)|find_package(tesseract REQUIRED COMPONENTS environment)|g' \
    -e 's|find_package(tesseract_visualization REQUIRED)|find_package(tesseract REQUIRED COMPONENTS visualization)|g' \
    -e 's|find_package(tesseract_kinematics REQUIRED)|find_package(tesseract REQUIRED COMPONENTS kinematics)|g' \
    -e 's|find_package(tesseract_support REQUIRED)|find_package(tesseract REQUIRED COMPONENTS support)|g' \
    -e 's|find_dependency(tesseract_common)|find_dependency(tesseract COMPONENTS common)|g' \
    -e 's|find_dependency(tesseract_geometry)|find_dependency(tesseract COMPONENTS geometry)|g' \
    -e 's|find_dependency(tesseract_scene_graph)|find_dependency(tesseract COMPONENTS scene_graph)|g' \
    -e 's|find_dependency(tesseract_state_solver)|find_dependency(tesseract COMPONENTS state_solver)|g' \
    -e 's|find_dependency(tesseract_collision)|find_dependency(tesseract COMPONENTS collision)|g' \
    -e 's|find_dependency(tesseract_srdf)|find_dependency(tesseract COMPONENTS srdf)|g' \
    -e 's|find_dependency(tesseract_urdf)|find_dependency(tesseract COMPONENTS urdf)|g' \
    -e 's|find_dependency(tesseract_environment)|find_dependency(tesseract COMPONENTS environment)|g' \
    -e 's|find_dependency(tesseract_visualization)|find_dependency(tesseract COMPONENTS visualization)|g' \
    -e 's|find_dependency(tesseract_kinematics)|find_dependency(tesseract COMPONENTS kinematics)|g' \
    -e 's|find_dependency(tesseract_support)|find_dependency(tesseract COMPONENTS support)|g' \
    {} +
echo "  Done."

echo ""
echo "=== IMPORTANT ==="
echo "1. find_package calls with COMPONENTS (e.g. find_package(tesseract_collision REQUIRED COMPONENTS core bullet))"
echo "   must be manually updated to: find_package(tesseract REQUIRED COMPONENTS collision collision_bullet)"
echo "2. Duplicate find_package(tesseract ...) / find_dependency(tesseract ...) calls should be"
echo "   consolidated into a single call listing all needed COMPONENTS."
echo ""
echo "Migration complete. Review changes with 'git diff' and rebuild."
```
