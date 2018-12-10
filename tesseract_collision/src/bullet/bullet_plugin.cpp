#include <tesseract_core/macros.h>
TESSERACT_IGNORE_WARNINGS_PUSH
#include <class_loader/class_loader.h>
TESSERACT_IGNORE_WARNINGS_POP

#include <tesseract_collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract_collision/bullet/bullet_discrete_simple_manager.h>
#include <tesseract_collision/bullet/bullet_cast_bvh_manager.h>
#include <tesseract_collision/bullet/bullet_cast_simple_manager.h>

CLASS_LOADER_REGISTER_CLASS(tesseract::tesseract_bullet::BulletDiscreteSimpleManager,
                            tesseract::DiscreteContactManagerBase)
CLASS_LOADER_REGISTER_CLASS(tesseract::tesseract_bullet::BulletDiscreteBVHManager,
                            tesseract::DiscreteContactManagerBase)

CLASS_LOADER_REGISTER_CLASS(tesseract::tesseract_bullet::BulletCastSimpleManager,
                            tesseract::ContinuousContactManagerBase)
CLASS_LOADER_REGISTER_CLASS(tesseract::tesseract_bullet::BulletCastBVHManager, tesseract::ContinuousContactManagerBase)
