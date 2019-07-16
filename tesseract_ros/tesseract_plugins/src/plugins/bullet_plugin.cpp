#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <class_loader/class_loader.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract_collision/bullet/bullet_discrete_simple_manager.h>
#include <tesseract_collision/bullet/bullet_cast_bvh_manager.h>
#include <tesseract_collision/bullet/bullet_cast_simple_manager.h>

CLASS_LOADER_REGISTER_CLASS(tesseract_collision::tesseract_collision_bullet::BulletDiscreteSimpleManager,
                            tesseract_collision::DiscreteContactManager)
CLASS_LOADER_REGISTER_CLASS(tesseract_collision::tesseract_collision_bullet::BulletDiscreteBVHManager,
                            tesseract_collision::DiscreteContactManager)

CLASS_LOADER_REGISTER_CLASS(tesseract_collision::tesseract_collision_bullet::BulletCastSimpleManager,
                            tesseract_collision::ContinuousContactManager)
CLASS_LOADER_REGISTER_CLASS(tesseract_collision::tesseract_collision_bullet::BulletCastBVHManager,
                            tesseract_collision::ContinuousContactManager)
