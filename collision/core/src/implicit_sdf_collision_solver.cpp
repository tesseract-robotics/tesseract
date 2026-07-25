/**
 * @file implicit_sdf_collision_solver.cpp
 * @brief Backend-neutral collision queries between implicit signed-distance shapes.
 *
 * The multi-start optimization strategy is adapted from MuJoCo's SDF collision implementation:
 * https://github.com/google-deepmind/mujoco/blob/main/src/engine/engine_collision_sdf.c
 * MuJoCo is licensed under the Apache License, Version 2.0.
 *
 * @copyright Copyright (c) 2026, Tesseract Robotics
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 */

#include <tesseract/collision/implicit_sdf_collision_solver.h>

#include <tesseract/geometry/geometries.h>
#include <tesseract/geometry/impl/signed_distance_field.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace tesseract::collision
{
namespace
{
constexpr double GRADIENT_TOLERANCE = 1e-12;
constexpr double ARMIJO_FACTOR = 0.1;
constexpr double STEP_REDUCTION = 0.5;

struct ObjectiveValue
{
  double value;
  Eigen::Vector3d gradient;
};

void validateConfig(const ImplicitSDFCollisionConfig& config)
{
  if (config.initial_sample_count <= 0)
    throw std::invalid_argument("ImplicitSDFCollisionConfig: initial_sample_count must be positive");
  if (config.max_iterations <= 0)
    throw std::invalid_argument("ImplicitSDFCollisionConfig: max_iterations must be positive");
  if (config.max_contacts <= 0)
    throw std::invalid_argument("ImplicitSDFCollisionConfig: max_contacts must be positive");
  if (!std::isfinite(config.contact_margin) || config.contact_margin < 0.0)
    throw std::invalid_argument("ImplicitSDFCollisionConfig: contact_margin must be finite and nonnegative");
  if (!std::isfinite(config.minimum_step) || config.minimum_step <= 0.0)
    throw std::invalid_argument("ImplicitSDFCollisionConfig: minimum_step must be finite and positive");
  if (!std::isfinite(config.duplicate_tolerance) || config.duplicate_tolerance < 0.0)
    throw std::invalid_argument("ImplicitSDFCollisionConfig: duplicate_tolerance must be finite and nonnegative");
}

double halton(int index, int base)
{
  double result = 0.0;
  double fraction = 1.0;
  while (index > 0)
  {
    fraction /= static_cast<double>(base);
    result += fraction * static_cast<double>(index % base);
    index /= base;
  }
  return result;
}

Eigen::AlignedBox3d expandAABB(const Eigen::AlignedBox3d& aabb, double expansion)
{
  const Eigen::Vector3d offset = Eigen::Vector3d::Constant(std::max(0.0, expansion));
  return { aabb.min() - offset, aabb.max() + offset };
}

Eigen::AlignedBox3d intersectAABBs(const Eigen::AlignedBox3d& aabb0, const Eigen::AlignedBox3d& aabb1)
{
  return { aabb0.min().cwiseMax(aabb1.min()), aabb0.max().cwiseMin(aabb1.max()) };
}

Eigen::AlignedBox3d transformAABB(const Eigen::AlignedBox3d& aabb, const Eigen::Isometry3d& pose)
{
  Eigen::AlignedBox3d transformed;
  transformed.setEmpty();
  for (int index = 0; index < 8; ++index)
  {
    Eigen::Vector3d corner;
    corner.x() = ((index & 1) != 0) ? aabb.max().x() : aabb.min().x();
    corner.y() = ((index & 2) != 0) ? aabb.max().y() : aabb.min().y();
    corner.z() = ((index & 4) != 0) ? aabb.max().z() : aabb.min().z();
    transformed.extend(pose * corner);
  }
  return transformed;
}

Eigen::Vector3d safeGradient(const ImplicitSDFShape& shape, const Eigen::Vector3d& point)
{
  Eigen::Vector3d gradient = shape.gradient(point);
  if (!gradient.allFinite() || gradient.squaredNorm() <= GRADIENT_TOLERANCE)
  {
    const double distance = shape.distance(point);
    const double step = std::max(1e-7, shape.aabb.diagonal().minCoeff() * 1e-6);
    if (!std::isfinite(distance) || !std::isfinite(step) || step <= 0.0)
      return Eigen::Vector3d::Zero();

    for (Eigen::Index axis = 0; axis < 3; ++axis)
    {
      Eigen::Vector3d offset = Eigen::Vector3d::Zero();
      offset[axis] = step;
      gradient[axis] = (shape.distance(point + offset) - distance) / step;
    }
    if (!gradient.allFinite() || gradient.squaredNorm() <= GRADIENT_TOLERANCE)
      return Eigen::Vector3d::Zero();
  }
  return gradient.normalized();
}

ObjectiveValue evaluateCollisionObjective(const ImplicitSDFShape& shape0,
                                          const ImplicitSDFShape& shape1,
                                          const Eigen::Vector3d& point)
{
  const double distance0 = shape0.distance(point);
  const double distance1 = shape1.distance(point);
  const Eigen::Vector3d gradient0 = safeGradient(shape0, point);
  const Eigen::Vector3d gradient1 = safeGradient(shape1, point);

  const bool use_shape0 = distance0 > distance1;
  const double maximum_distance = use_shape0 ? distance0 : distance1;
  const Eigen::Vector3d& maximum_gradient = use_shape0 ? gradient0 : gradient1;
  const double maximum_sign = (maximum_distance > 0.0) ? 1.0 : -1.0;

  return { distance0 + distance1 + std::abs(maximum_distance),
           gradient0 + gradient1 + (maximum_sign * maximum_gradient) };
}

ObjectiveValue evaluateIntersectionObjective(const ImplicitSDFShape& shape0,
                                             const ImplicitSDFShape& shape1,
                                             const Eigen::Vector3d& point)
{
  const double distance0 = shape0.distance(point);
  const double distance1 = shape1.distance(point);
  if (distance0 > distance1)
    return { distance0, safeGradient(shape0, point) };
  return { distance1, safeGradient(shape1, point) };
}

template <typename ObjectiveFunction>
double minimizeObjective(Eigen::Vector3d& point,
                         const Eigen::AlignedBox3d& bounds,
                         int iterations,
                         double minimum_step,
                         const ObjectiveFunction& objective)
{
  double value = std::numeric_limits<double>::infinity();
  for (int iteration = 0; iteration < iterations; ++iteration)
  {
    const ObjectiveValue current = objective(point);
    if (!std::isfinite(current.value) || !current.gradient.allFinite())
      return std::numeric_limits<double>::infinity();

    const double gradient_squared_norm = current.gradient.squaredNorm();
    if (gradient_squared_norm <= GRADIENT_TOLERANCE)
      return current.value;

    double step = 2.0;
    Eigen::Vector3d candidate = point;
    double candidate_value = current.value;
    while (true)
    {
      step *= STEP_REDUCTION;
      candidate = point - (step * current.gradient);
      candidate = candidate.cwiseMax(bounds.min()).cwiseMin(bounds.max());
      candidate_value = objective(candidate).value;
      if (step <= minimum_step || (std::isfinite(candidate_value) &&
                                   candidate_value - current.value <= -ARMIJO_FACTOR * step * gradient_squared_norm))
        break;
    }

    if (!std::isfinite(candidate_value) || candidate_value >= current.value)
      return current.value;

    point = candidate;
    value = candidate_value;
  }
  return value;
}

bool makeContact(ImplicitSDFContact& contact,
                 const ImplicitSDFShape& shape0,
                 const ImplicitSDFShape& shape1,
                 const Eigen::Vector3d& point,
                 double margin)
{
  const double distance0 = shape0.distance(point);
  const double distance1 = shape1.distance(point);
  const Eigen::Vector3d gradient0 = safeGradient(shape0, point);
  const Eigen::Vector3d gradient1 = safeGradient(shape1, point);
  if (!std::isfinite(distance0) || !std::isfinite(distance1) || gradient0.isZero() || gradient1.isZero())
    return false;

  Eigen::Vector3d normal = gradient0 - gradient1;
  if (normal.squaredNorm() <= GRADIENT_TOLERANCE)
    normal = gradient0;
  if (normal.squaredNorm() <= GRADIENT_TOLERANCE)
    return false;
  normal.normalize();

  const Eigen::Vector3d nearest0 = point - (distance0 * gradient0);
  const Eigen::Vector3d nearest1 = point - (distance1 * gradient1);
  const double signed_distance = (nearest1 - nearest0).dot(normal);
  if (!std::isfinite(signed_distance) || signed_distance > margin)
    return false;

  contact.distance = signed_distance;
  contact.normal = normal;
  contact.nearest_points = { nearest0, nearest1 };
  contact.point = (nearest0 + nearest1) / 2.0;
  return true;
}

void appendUniqueContact(std::vector<ImplicitSDFContact>& contacts,
                         ImplicitSDFContact contact,
                         double duplicate_tolerance)
{
  const double tolerance_squared = duplicate_tolerance * duplicate_tolerance;
  for (auto& existing : contacts)
  {
    if ((existing.point - contact.point).squaredNorm() <= tolerance_squared)
    {
      if (contact.distance < existing.distance)
        existing = std::move(contact);
      return;
    }
  }
  contacts.push_back(std::move(contact));
}

std::vector<ImplicitSDFContact> selectContacts(std::vector<ImplicitSDFContact> candidates, int maximum_count)
{
  if (maximum_count <= 0 || candidates.empty())
    return {};

  std::sort(candidates.begin(), candidates.end(), [](const auto& lhs, const auto& rhs) {
    return lhs.distance < rhs.distance;
  });
  if (static_cast<int>(candidates.size()) <= maximum_count)
    return candidates;

  std::vector<ImplicitSDFContact> selected;
  selected.reserve(static_cast<std::size_t>(maximum_count));
  selected.push_back(candidates.front());
  std::vector<double> minimum_distance_squared(candidates.size(), std::numeric_limits<double>::infinity());
  std::vector<bool> used(candidates.size(), false);
  used.front() = true;

  while (static_cast<int>(selected.size()) < maximum_count)
  {
    std::size_t best_index = 0;
    double best_distance_squared = -1.0;
    for (std::size_t index = 0; index < candidates.size(); ++index)
    {
      if (used[index])
        continue;

      const double distance_squared = (candidates[index].point - selected.back().point).squaredNorm();
      minimum_distance_squared[index] = std::min(minimum_distance_squared[index], distance_squared);
      if (minimum_distance_squared[index] > best_distance_squared)
      {
        best_distance_squared = minimum_distance_squared[index];
        best_index = index;
      }
    }

    used[best_index] = true;
    selected.push_back(candidates[best_index]);
  }
  return selected;
}

ImplicitSDFShape makeWorldShape(ImplicitSDFShape::DistanceFunction local_distance,
                                ImplicitSDFShape::GradientFunction local_gradient,
                                const Eigen::AlignedBox3d& local_aabb,
                                const Eigen::Isometry3d& pose)
{
  const Eigen::Isometry3d inverse_pose = pose.inverse();
  ImplicitSDFShape shape;
  shape.distance = [distance = std::move(local_distance), inverse_pose](const Eigen::Vector3d& point) {
    return distance(inverse_pose * point);
  };
  shape.gradient = [gradient = std::move(local_gradient), inverse_pose, pose](const Eigen::Vector3d& point) {
    const Eigen::Vector3d local_result = gradient(inverse_pose * point);
    return Eigen::Vector3d(pose.linear() * local_result);
  };
  shape.aabb = transformAABB(local_aabb, pose);
  return shape;
}

ImplicitSDFShape::GradientFunction makeNumericalGradient(const ImplicitSDFShape::DistanceFunction& distance,
                                                         double step)
{
  return [distance, step](const Eigen::Vector3d& point) {
    Eigen::Vector3d gradient;
    for (Eigen::Index axis = 0; axis < 3; ++axis)
    {
      Eigen::Vector3d offset = Eigen::Vector3d::Zero();
      offset[axis] = step;
      gradient[axis] = (distance(point + offset) - distance(point - offset)) / (2.0 * step);
    }
    return gradient;
  };
}

double cappedConeDistance(const Eigen::Vector3d& point, double radius, double half_height)
{
  const Eigen::Vector2d q(point.head<2>().norm(), point.z());
  const Eigen::Vector2d k1(0.0, half_height);
  const Eigen::Vector2d k2(-radius, 2.0 * half_height);
  const double cap_radius = (q.y() < 0.0) ? radius : 0.0;
  const Eigen::Vector2d cap(q.x() - std::min(q.x(), cap_radius), std::abs(q.y()) - half_height);
  const double denominator = k2.squaredNorm();
  const double projection = std::clamp((k1 - q).dot(k2) / denominator, 0.0, 1.0);
  const Eigen::Vector2d side = q - k1 + (projection * k2);
  const double sign = (side.x() < 0.0 && cap.y() < 0.0) ? -1.0 : 1.0;
  return sign * std::sqrt(std::min(cap.squaredNorm(), side.squaredNorm()));
}
}  // namespace

bool ImplicitSDFShape::isValid() const
{
  return static_cast<bool>(distance) && static_cast<bool>(gradient) && !aabb.isEmpty() && aabb.min().allFinite() &&
         aabb.max().allFinite();
}

std::vector<ImplicitSDFContact> collideImplicitSDF(const ImplicitSDFShape& shape0,
                                                   const ImplicitSDFShape& shape1,
                                                   const ImplicitSDFCollisionConfig& config)
{
  validateConfig(config);
  if (!shape0.isValid())
    throw std::invalid_argument("collideImplicitSDF: shape0 is invalid");
  if (!shape1.isValid())
    throw std::invalid_argument("collideImplicitSDF: shape1 is invalid");

  const double half_margin = std::max(0.0, config.contact_margin) / 2.0;
  const Eigen::AlignedBox3d bounds =
      intersectAABBs(expandAABB(shape0.aabb, half_margin), expandAABB(shape1.aabb, half_margin));
  if (bounds.isEmpty())
    return {};

  std::vector<ImplicitSDFContact> candidates;
  candidates.reserve(static_cast<std::size_t>(config.initial_sample_count));
  for (int sample = 0; sample < config.initial_sample_count; ++sample)
  {
    Eigen::Vector3d point;
    if (sample == 0)
    {
      point = bounds.center();
    }
    else
    {
      point.x() = bounds.min().x() + (bounds.sizes().x() * halton(sample, 2));
      point.y() = bounds.min().y() + (bounds.sizes().y() * halton(sample, 3));
      point.z() = bounds.min().z() + (bounds.sizes().z() * halton(sample, 5));
    }

    const auto collision_objective = [&shape0, &shape1](const Eigen::Vector3d& candidate) {
      return evaluateCollisionObjective(shape0, shape1, candidate);
    };
    const double objective =
        minimizeObjective(point, bounds, config.max_iterations, config.minimum_step, collision_objective);
    if (!std::isfinite(objective))
      continue;

    const auto intersection_objective = [&shape0, &shape1](const Eigen::Vector3d& candidate) {
      return evaluateIntersectionObjective(shape0, shape1, candidate);
    };
    minimizeObjective(point, bounds, 1, config.minimum_step, intersection_objective);

    ImplicitSDFContact contact;
    if (makeContact(contact, shape0, shape1, point, config.contact_margin))
      appendUniqueContact(candidates, std::move(contact), config.duplicate_tolerance);
  }

  return selectContacts(std::move(candidates), config.max_contacts);
}

ImplicitSDFShape makeImplicitSDFShape(const tesseract::geometry::Geometry& geometry, const Eigen::Isometry3d& pose)
{
  if (!pose.matrix().allFinite())
    throw std::invalid_argument("makeImplicitSDFShape: pose must be finite");

  using tesseract::geometry::GeometryType;
  switch (geometry.getType())
  {
    case GeometryType::BOX:
    {
      const auto& box = static_cast<const tesseract::geometry::Box&>(geometry);
      const Eigen::Vector3d half_extents(box.getX() / 2.0, box.getY() / 2.0, box.getZ() / 2.0);
      const auto distance = [half_extents](const Eigen::Vector3d& point) {
        const Eigen::Vector3d q = point.cwiseAbs() - half_extents;
        return q.cwiseMax(0.0).norm() + std::min(q.maxCoeff(), 0.0);
      };
      return makeWorldShape(
          distance, makeNumericalGradient(distance, 1e-6), Eigen::AlignedBox3d(-half_extents, half_extents), pose);
    }
    case GeometryType::SPHERE:
    {
      const auto& sphere = static_cast<const tesseract::geometry::Sphere&>(geometry);
      const double radius = sphere.getRadius();
      return makeWorldShape([radius](const Eigen::Vector3d& point) { return point.norm() - radius; },
                            [](const Eigen::Vector3d& point) {
                              const double norm = point.norm();
                              return (norm > GRADIENT_TOLERANCE) ? Eigen::Vector3d(point / norm) :
                                                                   Eigen::Vector3d::Zero();
                            },
                            Eigen::AlignedBox3d(Eigen::Vector3d::Constant(-radius), Eigen::Vector3d::Constant(radius)),
                            pose);
    }
    case GeometryType::CYLINDER:
    {
      const auto& cylinder = static_cast<const tesseract::geometry::Cylinder&>(geometry);
      const double radius = cylinder.getRadius();
      const double half_height = cylinder.getLength() / 2.0;
      const auto distance = [radius, half_height](const Eigen::Vector3d& point) {
        const Eigen::Vector2d q(point.head<2>().norm() - radius, std::abs(point.z()) - half_height);
        return std::min(q.maxCoeff(), 0.0) + q.cwiseMax(0.0).norm();
      };
      return makeWorldShape(distance,
                            makeNumericalGradient(distance, 1e-6),
                            Eigen::AlignedBox3d(Eigen::Vector3d(-radius, -radius, -half_height),
                                                Eigen::Vector3d(radius, radius, half_height)),
                            pose);
    }
    case GeometryType::CONE:
    {
      const auto& cone = static_cast<const tesseract::geometry::Cone&>(geometry);
      const double radius = cone.getRadius();
      const double half_height = cone.getLength() / 2.0;
      const auto distance = [radius, half_height](const Eigen::Vector3d& point) {
        return cappedConeDistance(point, radius, half_height);
      };
      return makeWorldShape(distance,
                            makeNumericalGradient(distance, 1e-6),
                            Eigen::AlignedBox3d(Eigen::Vector3d(-radius, -radius, -half_height),
                                                Eigen::Vector3d(radius, radius, half_height)),
                            pose);
    }
    case GeometryType::CAPSULE:
    {
      const auto& capsule = static_cast<const tesseract::geometry::Capsule&>(geometry);
      const double radius = capsule.getRadius();
      const double half_height = capsule.getLength() / 2.0;
      const auto distance = [radius, half_height](const Eigen::Vector3d& point) {
        Eigen::Vector3d axis_point(0.0, 0.0, std::clamp(point.z(), -half_height, half_height));
        return (point - axis_point).norm() - radius;
      };
      return makeWorldShape(distance,
                            makeNumericalGradient(distance, 1e-6),
                            Eigen::AlignedBox3d(Eigen::Vector3d(-radius, -radius, -half_height - radius),
                                                Eigen::Vector3d(radius, radius, half_height + radius)),
                            pose);
    }
    case GeometryType::SIGNED_DISTANCE_FIELD:
    {
      const auto& sdf = static_cast<const tesseract::geometry::SignedDistanceField&>(geometry);
      const Eigen::Vector3d scale = sdf.getScale();
      if (!scale.allFinite() || (scale.array() <= 0.0).any())
        throw std::invalid_argument("makeImplicitSDFShape: signed distance field scale must be finite and positive");
      if (!scale.isApprox(Eigen::Vector3d::Constant(scale.x())))
        throw std::invalid_argument("makeImplicitSDFShape: nonuniform signed distance field scaling is unsupported");
      if (sdf.getDimensions().minCoeff() < 2 || sdf.getDomain().isEmpty() || !sdf.getDomain().min().allFinite() ||
          !sdf.getDomain().max().allFinite())
        throw std::invalid_argument("makeImplicitSDFShape: signed distance field domain is invalid");

      const double distance_scale = scale.x();
      const Eigen::AlignedBox3d domain = sdf.getDomain();
      const auto distance = [&sdf, domain, scale, distance_scale](const Eigen::Vector3d& point) {
        const Eigen::Vector3d local_point = point.cwiseQuotient(scale);
        if (!domain.contains(local_point))
          return std::numeric_limits<double>::infinity();
        return sdf.getDistance(local_point) * distance_scale;
      };
      Eigen::AlignedBox3d scaled_domain(sdf.getDomain().min().cwiseProduct(scale),
                                        sdf.getDomain().max().cwiseProduct(scale));
      const double gradient_step = std::max(1e-6, scaled_domain.diagonal().minCoeff() * 1e-5);
      return makeWorldShape(distance, makeNumericalGradient(distance, gradient_step), scaled_domain, pose);
    }
    default:
      return {};
  }
}

}  // namespace tesseract::collision
