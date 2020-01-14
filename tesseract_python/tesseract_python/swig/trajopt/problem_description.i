%{
#include <trajopt/problem_description.hpp>	
%}

%include <attribute.i>

%shared_ptr(trajopt::TrajOptProb);
%shared_ptr(trajopt::TrajOptResult);
%shared_ptr(trajopt::TermInfo);
%shared_ptr(trajopt::DynamicCartPoseTermInfo);
%shared_ptr(trajopt::CartPoseTermInfo);
%shared_ptr(trajopt::CartVelTermInfo);
%shared_ptr(trajopt::JointPosTermInfo);
%shared_ptr(trajopt::JointVelTermInfo);
%shared_ptr(trajopt::JointAccTermInfo);
%shared_ptr(trajopt::JointJerkTermInfo);
%shared_ptr(trajopt::CollisionTermInfo);
%shared_ptr(trajopt::TotalTimeTermInfo);
%shared_ptr(trajopt::SafetyMarginData);


//%template(DblVec) std::vector<double>;

// Convert to Eigen for DblVec so numpy shows up on Python side
%typemap(out, fragment="Eigen_Fragments") std::vector<double>*, std::vector<double> const*
{
  Eigen::VectorXd temp_matrix=Eigen::Map<Eigen::VectorXd>(&(*$1)[0],$1->size());
  if (!ConvertFromEigenToNumPyMatrix<Eigen::VectorXd >(&$result, &temp_matrix))
    SWIG_fail;
}
%typemap(in, fragment="Eigen_Fragments") std::vector<double>* (std::vector<double> temp)
{
  Eigen::VectorXd temp_matrix;  
  if (!ConvertFromNumpyToEigenMatrix<Eigen::VectorXd>(&temp_matrix, $input))
    SWIG_fail;
  temp = std::vector<double>(temp_matrix.data(), temp_matrix.data() + temp_matrix.size());
  $1 = &temp;
}

//%template(IntVec) std::vector<int>;
// Convert to Eigen for DblVec so numpy shows up on Python side
%typemap(out, fragment="Eigen_Fragments") std::vector<int>*, std::vector<int> const*
{
  Eigen::VectorXi temp_matrix=Eigen::Map<Eigen::VectorXi>(&(*$1)[0],$1->size());
  if (!ConvertFromEigenToNumPyMatrix<Eigen::VectorXi >(&$result, &temp_matrix))
    SWIG_fail;
}
%typemap(in, fragment="Eigen_Fragments") std::vector<int>* (std::vector<int> temp)
{
  Eigen::VectorXi temp_matrix;  
  if (!ConvertFromNumpyToEigenMatrix<Eigen::VectorXi>(&temp_matrix, $input))
    SWIG_fail;
  temp = std::vector<int>(temp_matrix.data(), temp_matrix.data() + temp_matrix.size());
  $1 = &temp;
}


%template(SafetyMarginDataPtr_vector) std::vector<std::shared_ptr<trajopt::SafetyMarginData>>;
%template(TermInfoPtr_vector) std::vector<std::shared_ptr<trajopt::TermInfo>>;


namespace sco
{
typedef std::vector<double> DblVec;
typedef std::vector<int> IntVec;

class ModelType
{
public:
  enum Value
  {
    GUROBI,
    BPMPD,
    OSQP,
    QPOASES,
    AUTO_SOLVER
  };

  static const std::vector<std::string> MODEL_NAMES_;

  ModelType();  
  ModelType(const int& v);
  ModelType(const std::string& s);
  //operator int() const;
  %extend
  {
	  int __int__()
	  {
		  return (int)*$self;
	  }
  }
  bool operator==(const ModelType::Value& a) const;
  bool operator==(const ModelType& a) const;
  bool operator!=(const ModelType& a) const;
  void fromJson(const Json::Value& v);
};

struct BasicTrustRegionSQPParameters
{
  double improve_ratio_threshold;
  double min_trust_box_size;
  double min_approx_improve;
  double min_approx_improve_frac;
  double max_iter;                 
  double trust_shrink_ratio;
  double trust_expand_ratio;
  double cnt_tolerance;
  double max_merit_coeff_increases;
  double merit_coeff_increase_ratio;  
  double max_time;                    
  double initial_merit_error_coeff;
  bool inflate_constraints_individually;    
  double trust_box_size;
  bool log_results;
  std::string log_dir;

  BasicTrustRegionSQPParameters();
};


}

namespace trajopt
{

typedef sco::DblVec DblVec;
typedef sco::IntVec IntVec;

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> DblMatrix;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> TrajArray;

class TrajOptProb;
struct ProblemConstructionInfo;
struct TrajOptResult;

enum TermType
{
  TT_COST = 0x1,      
  TT_CNT = 0x2,       
  TT_USE_TIME = 0x4,  
};

class TrajOptProb
{
public:

  using Ptr = std::shared_ptr<TrajOptProb>;
  
  TrajOptProb();
  TrajOptProb(int n_steps, const ProblemConstructionInfo& pci);
  virtual ~TrajOptProb();
  // sco::VarVector GetVarRow(int i, int start_col, int num_col);
  // sco::VarVector GetVarRow(int i);
  // sco::Var& GetVar(int i, int j);
  // VarArray& GetVars();  
  int GetNumSteps();  
  int GetNumDOF();
  tesseract_kinematics::ForwardKinematics::ConstPtr  GetKin();
  tesseract_environment::Environment::ConstPtr GetEnv();
  void SetInitTraj(const TrajArray& x);
  TrajArray GetInitTraj();  
  bool GetHasTime();  
  void SetHasTime(bool tmp);  
};

%nodefaultctor TrajOptResult;
struct TrajOptResult
{
  using Ptr = std::shared_ptr<TrajOptResult>;

  std::vector<std::string> cost_names, cnt_names;
  DblVec cost_vals, cnt_viols;
  TrajArray traj;  
};

struct BasicInfo
{  
  bool start_fixed;
  int n_steps;
  std::string manip;
  std::string robot;
  IntVec dofs_fixed;
  sco::ModelType convex_solver;
  bool use_time;
  double dt_upper_lim;
  double dt_lower_lim;
};

struct InitInfo
{  
  enum Type
  {
    STATIONARY,
    JOINT_INTERPOLATED,
    GIVEN_TRAJ,
  };  
  Type type;  
  TrajArray data;  
  double dt;
};


%nodefaultctor TermInfo;
class TermInfo
{
public:

  using Ptr = std::shared_ptr<TermInfo>;

  std::string name;
  int term_type;
  int getSupportedTypes();
  virtual void fromJson(ProblemConstructionInfo& pci, const Json::Value& v);
  virtual void hatch(TrajOptProb& prob);

  static TermInfo::Ptr fromName(const std::string& type);
  typedef TermInfo::Ptr (*MakerFunc)(void);
  //static void RegisterMaker(const std::string& type, MakerFunc);
  virtual ~TermInfo();
};

struct ProblemConstructionInfo
{
public:
  BasicInfo basic_info;
  sco::BasicTrustRegionSQPParameters opt_info;
  std::vector<TermInfo::Ptr> cost_infos;
  std::vector<TermInfo::Ptr> cnt_infos;
  InitInfo init_info;

  tesseract_environment::Environment::ConstPtr env;
  tesseract_kinematics::ForwardKinematics::ConstPtr kin;

  tesseract_kinematics::ForwardKinematics::ConstPtr getManipulator(const std::string& name) const;

  ProblemConstructionInfo(tesseract::Tesseract::ConstPtr tesseract);
  void fromJson(const Json::Value& v);
};

// TODO: UserDefinedTermInfo

struct DynamicCartPoseTermInfo : public TermInfo
{
  int timestep;
  std::string target;  
  Eigen::Vector3d pos_coeffs, rot_coeffs;  
  std::string link;  
  Eigen::Isometry3d tcp;
  Eigen::Isometry3d target_tcp;
  void fromJson(ProblemConstructionInfo& pci, const Json::Value& v);  
  void hatch(TrajOptProb& prob);  
};

struct CartPoseTermInfo : public TermInfo
{  
  int timestep;  
  Eigen::Vector3d xyz;  
  Eigen::Vector4d wxyz;
  Eigen::Vector3d pos_coeffs, rot_coeffs;  
  std::string link;  
  Eigen::Isometry3d tcp;
  std::string target;

  void fromJson(ProblemConstructionInfo& pci, const Json::Value& v);
  
  void hatch(TrajOptProb& prob) override;  
};

struct CartVelTermInfo : public TermInfo
{  
  int first_step, last_step;  
  std::string link;
  double max_displacement;  
  void fromJson(ProblemConstructionInfo& pci, const Json::Value& v);  
  void hatch(TrajOptProb& prob) override;  
};

struct JointPosTermInfo : public TermInfo
{  
  DblVec coeffs;  
  DblVec targets;  
  DblVec upper_tols;  
  DblVec lower_tols;  
  int first_step;  
  int last_step;
  void fromJson(ProblemConstructionInfo& pci, const Json::Value& v);  
  void hatch(TrajOptProb& prob) override; 
};

struct JointVelTermInfo : public TermInfo
{  
  DblVec coeffs;
  DblVec targets;
  DblVec upper_tols;
  DblVec lower_tols;
  int first_step;
  int last_step;
  void fromJson(ProblemConstructionInfo& pci, const Json::Value& v);
  void hatch(TrajOptProb& prob) override;
};

struct JointAccTermInfo : public TermInfo
{  
  DblVec coeffs;  
  DblVec targets;  
  DblVec upper_tols;  
  DblVec lower_tols; 
  int first_step; 
  int last_step;  
  void fromJson(ProblemConstructionInfo& pci, const Json::Value& v);
  void hatch(TrajOptProb& prob);  
};

struct JointJerkTermInfo : public TermInfo
{
  DblVec coeffs;
  DblVec targets;
  DblVec upper_tols;
  DblVec lower_tols;
  int first_step;
  int last_step;
  void fromJson(ProblemConstructionInfo& pci, const Json::Value& v);
  void hatch(TrajOptProb& prob);  
};

struct SafetyMarginData
{

    using Ptr = std::shared_ptr<SafetyMarginData>;
    using ConstPtr = std::shared_ptr<const SafetyMarginData>;

	SafetyMarginData(const double& default_safety_margin, const double& default_safety_margin_coeff);
	
	void setPairSafetyMarginData(const std::string& obj1,
                               const std::string& obj2,
                               const double& safety_margin,
                               const double& safety_margin_coeff);
	
	Eigen::Vector2d getPairSafetyMarginData(const std::string& obj1, const std::string& obj2) const;
	
	const double getMaxSafetyMargin() const;	
	
};

std::vector<std::shared_ptr<SafetyMarginData> > createSafetyMarginDataVector(int num_elements,
                                                                const double& default_safety_margin,
                                                                const double& default_safety_margin_coeff);

enum class CollisionEvaluatorType
{
  SINGLE_TIMESTEP = 0,
  DISCRETE_CONTINUOUS = 1,
  CAST_CONTINUOUS = 2,
};

class CollisionTermInfo : public TermInfo
{
public:
  int first_step, last_step;
  CollisionEvaluatorType evaluator_type;
  std::vector<int> fixed_steps;
  double longest_valid_segment_length = 0.5;
  std::vector<std::shared_ptr<SafetyMarginData> > info;
  tesseract_collision::ContactTestType contact_test_type;
  void fromJson(ProblemConstructionInfo& pci, const Json::Value& v);
  void hatch(TrajOptProb& prob);
};

struct TotalTimeTermInfo : public TermInfo
{
  double coeff;
  double limit;

  void hatch(TrajOptProb& prob);
  void fromJson(ProblemConstructionInfo& pci, const Json::Value& v);
};

TrajOptProb::Ptr ConstructProblem(const ProblemConstructionInfo&);
TrajOptProb::Ptr ConstructProblem(const Json::Value&, const tesseract::Tesseract::ConstPtr& tesseract);
TrajOptResult::Ptr OptimizeProblem(TrajOptProb::Ptr,
                                               const tesseract_visualization::Visualization::Ptr& plotter = nullptr);

}
