#include <tesseract_motion_planners/descartes/descartes_tesseract_kinematics.h>
#include <Eigen/Eigen>
#include <console_bridge/console.h>

namespace tesseract_motion_planners
{
template <typename FloatType>
bool DescartesTesseractKinematics<FloatType>::ik(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p,
                                                 std::vector<FloatType>& solution_set) const
{
  return ik(p, is_valid_fn_, redundant_sol_fn_, solution_set);
}

template <typename FloatType>
bool DescartesTesseractKinematics<FloatType>::ik(
    const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p,
    const descartes_light::IsValidFn<FloatType>& is_valid_fn,
    const descartes_light::GetRedundantSolutionsFn<FloatType>& redundant_sol_fn,
    std::vector<FloatType>& solution_set) const
{
  unsigned int dof = tesseract_ik_->numJoints();

  // Convert to appropriate Eigen types
  Eigen::Isometry3d p_double;
  p_double = p.template cast<double>();
  Eigen::VectorXd solution_eigen;

  // Solve IK
  if (!tesseract_ik_->calcInvKin(solution_eigen, p_double, ik_seed_))
    return false;

  // Convert back to a float array
  Eigen::Matrix<FloatType, Eigen::Dynamic, 1> solution_float_type;
  solution_float_type = solution_eigen.template cast<FloatType>();
  FloatType* sol = solution_float_type.data();

  // Apply is_valid_fn and redundant_sol_fn

  // Levi - Do we need harmonizeTowardZero(sol) here?

  if (is_valid_fn && redundant_sol_fn)
  {
    if (is_valid_fn_(sol))
      solution_set.insert(end(solution_set), sol, sol + dof);  // If good then add to solution set

    std::vector<FloatType> redundant_sols = redundant_sol_fn(sol);
    if (!redundant_sols.empty())
    {
      int num_sol = redundant_sols.size() / dof;
      for (int s = 0; s < num_sol; ++s)
      {
        FloatType* redundant_sol = redundant_sols.data() + dof * s;
        if (is_valid_fn_(redundant_sol))
          solution_set.insert(end(solution_set), redundant_sol, redundant_sol + dof);  // If good then add to solution
                                                                                       // set
      }
    }
  }
  else if (is_valid_fn && !redundant_sol_fn)
  {
    if (is_valid_fn(sol))
      solution_set.insert(end(solution_set), sol, sol + dof);  // If good then add to solution set
  }
  else if (!is_valid_fn && redundant_sol_fn)
  {
    solution_set.insert(end(solution_set), sol, sol + dof);  // If good then add to solution set

    std::vector<FloatType> redundant_sols = redundant_sol_fn(sol);
    if (!redundant_sols.empty())
    {
      unsigned int num_sol = redundant_sols.size() / dof;
      for (unsigned int s = 0; s < num_sol; ++s)
      {
        FloatType* redundant_sol = redundant_sols.data() + dof * s;
        solution_set.insert(end(solution_set), redundant_sol, redundant_sol + dof);  // If good then add to solution
                                                                                     // set
      }
    }
  }
  else
  {
    solution_set.insert(end(solution_set), sol, sol + dof);
  }
  return !solution_set.empty();
}

template <typename FloatType>
bool DescartesTesseractKinematics<FloatType>::fk(const FloatType* pose,
                                                 Eigen::Transform<FloatType, 3, Eigen::Isometry>& solution) const
{
  assert(pose);

  // Convert the Array to an Eigen VectorXd
  Eigen::VectorXd joints(tesseract_fk_->numJoints());
  for (int i = 0; static_cast<unsigned int>(i) < tesseract_fk_->numJoints(); i++)
    joints(i, 0) = pose[i];

  // Get the solution from the Tesseract Kinematics
  Eigen::Isometry3d solution_double;
  bool success = tesseract_fk_->calcFwdKin(solution_double, joints);

  // Cast from double to FloatType
  solution = solution_double.cast<FloatType>();
  return success;
}

template <typename FloatType>
int DescartesTesseractKinematics<FloatType>::dof() const
{
  assert(tesseract_fk_->numJoints() < std::numeric_limits<int>::max());
  return static_cast<int>(tesseract_fk_->numJoints());
}

template <typename FloatType>
void DescartesTesseractKinematics<FloatType>::analyzeIK(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p) const
{
  Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "AnalyzeIK: ", ";");

  std::stringstream ss;
  ss << p.matrix().format(CommaInitFmt);
  CONSOLE_BRIDGE_logInform(ss.str().c_str());

  std::string valid_fn_defined = "\tIs Valid Function: " + std::string(is_valid_fn_ ? "True" : "False");
  CONSOLE_BRIDGE_logInform(valid_fn_defined.c_str());
  std::string redundant_fn_defined = "\tIs Valid Function: " + std::string(is_valid_fn_ ? "True" : "False");
  CONSOLE_BRIDGE_logInform(redundant_fn_defined.c_str());

  std::vector<FloatType> solution_set;
  ik(p, nullptr, nullptr, solution_set);
  ss.clear();
  ss << "\tSampling without functions, found solutions: " << solution_set.size() / 8;
  CONSOLE_BRIDGE_logInform(ss.str().c_str());

  solution_set.clear();
  ik(p, is_valid_fn_, nullptr, solution_set);
  ss.clear();
  ss << "\tSampling with only IsValid functions, found solutions: " << solution_set.size() / 8;
  CONSOLE_BRIDGE_logInform(ss.str().c_str());

  solution_set.clear();
  ik(p, nullptr, redundant_sol_fn_, solution_set);
  ss.clear();
  ss << "\tSampling with only Redundant Solutions functions, found solutions: " << solution_set.size() / 8;
  CONSOLE_BRIDGE_logInform(ss.str().c_str());

  solution_set.clear();
  ik(p, is_valid_fn_, redundant_sol_fn_, solution_set);
  ss.clear();
  ss << "\tSampling with both functions, found solutions: " << solution_set.size() / 8;
  CONSOLE_BRIDGE_logInform(ss.str().c_str());
}

template <typename FloatType>
void DescartesTesseractKinematics<FloatType>::setIKSeed(
    const Eigen::Ref<const Eigen::Matrix<FloatType, Eigen::Dynamic, 1> >& seed)
{
  assert(seed.size() == dof());
  ik_seed_ = seed.template cast<double>();
}

template <typename FloatType>
void DescartesTesseractKinematics<FloatType>::setIKSeed(const std::vector<FloatType>& seed)
{
  assert(seed.size() == dof());
  std::vector<double> seed_copy;
  for (auto& i : seed)
    seed_copy.push_back(static_cast<double>(i));
  ik_seed_ = Eigen::Map<Eigen::VectorXd>(seed_copy.data(), seed_copy.size());
}
}  // namespace tesseract_motion_planners
