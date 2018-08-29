#include <class_loader/class_loader.h>
#include <tesseract_collision/bullet/bullet_discrete_managers.h>
#include <tesseract_collision/bullet/bullet_cast_managers.h>

CLASS_LOADER_REGISTER_CLASS(tesseract::BulletDiscreteSimpleManager, tesseract::DiscreteContactManagerBase)
CLASS_LOADER_REGISTER_CLASS(tesseract::BulletDiscreteBVHManager, tesseract::DiscreteContactManagerBase)

CLASS_LOADER_REGISTER_CLASS(tesseract::BulletCastSimpleManager, tesseract::ContinuousContactManagerBase)
CLASS_LOADER_REGISTER_CLASS(tesseract::BulletCastBVHManager, tesseract::ContinuousContactManagerBase)
