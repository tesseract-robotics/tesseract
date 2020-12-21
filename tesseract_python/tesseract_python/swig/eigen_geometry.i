/**
 * @file eigen_geometry.i
 * @brief Eigen geometry types used by Tesseract
 *
 * @author John Wason
 * @date December 18, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Wason Technology, LLC
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

namespace Eigen
{
    %pythondynamic AngleAxisd;
    %pythondynamic Isometry3d;
    %pythondynamic Quaterniond;
    %pythondynamic Translation3d;
    class AngleAxisd;
    class Quaterniond;
    class Isometry3d;
    class Translation3d;

    class Quaterniond
    {
    public:
        Quaterniond();
        Quaterniond(const AngleAxisd& aa);
        Quaterniond(const Eigen::Matrix3d& other);
        Quterniondd(const Eigen::Vector4d& other);
        Quaterniond(const Quaterniond& other);
        Quaterniond(double w, double x, double y, double z);

        Quaterniond FromTwoVectors(const Eigen::Vector3d a, const Eigen::Vector3d b);
        static Quaterniond UnitRandom();

        double angularDistance(const Quaterniond& other);
        Quaterniond conjugate();
        double dot(const Quaterniond& other);
        Quaterniond inverse();
        bool isApprox(const Quaterniond& other);
        bool isApprox(const Quaterniond& other, double prec);
        double norm();
        void normalize();
        Quaterniond normalized();
        %rename(__mul__) operator*;
        Quaterniond operator* (const Quaterniond& other);
        Isometry3d operator* (const Isometry3d& other);
        Eigen::Vector3d operator* (const Eigen::Vector3d& other);
        Eigen::Matrix3d operator* (const Eigen::Matrix3d& other);
        Isometry3d operator* (const Translation3d& other);

        Quaterniond setFromTwoVectors(const Eigen::Vector3d a, const Eigen::Vector3d b);
        Quaterniond setIdentity();

        Quaterniond slerp(double t, const Eigen::Quaterniond& other);

        double squaredNorm();
        Eigen::Matrix3d toRotationMatrix();

        Eigen::Vector3d vec();
        double w();
        double x();
        double y();
        double z();

        %extend
        {
            void setVec(const Eigen::Vector3d& vec)
            {
                $self->vec() = vec;
            }
            void setW(double w)
            {
                $self->w() = w;
            }
            void setX(double x)
            {
                $self->x() = x;
            }
            void setY(double y)
            {
                $self->y() = y;
            }
            void setZ(double z)
            {
                $self->z() = z;
            }
        }
    };
    
    class AngleAxisd
    {
    public:
        AngleAxisd();
        AngleAxisd(const AngleAxisd& other);
        AngleAxisd(const Eigen::Matrix3d& mat);
        AngleAxisd(double angle, const Eigen::Vector3d& axis);
        AngleAxisd(const Eigen::Quaterniond& q);

        double angle();
        Eigen::Vector3d axis();
        %extend
        {
            void setAngle(double angle)
            {
                $self->angle() = angle;
            }

            void setAxis(const Eigen::Vector3d& axis)
            {
                $self->axis() = axis;
            }
        }

        AngleAxisd inverse();

        bool isApprox(const AngleAxisd& other);
        bool isApprox(const AngleAxisd& other, double prec);

        Eigen::Matrix3d toRotationMatrix();
        AngleAxisd fromRotationMatrix(const Eigen::Matrix3d& mat);

        %rename(__mul__) operator*;
        Quaterniond operator* (const AngleAxisd& other);
        Quaterniond operator* (const Quaterniond& other);
        Isometry3d operator* (const Isometry3d& other);
        Eigen::Vector3d operator* (const Eigen::Vector3d& other);
        Eigen::Matrix3d operator* (const Eigen::Matrix3d& other);
        Isometry3d operator* (const Translation3d& other);
    };

    class Translation3d
    {
    public:
        Translation3d();
        Translation3d(const Translation3d& other);
        Translation3d(const Eigen::Vector3d& vector);
        Translation3d(double x, double y, double z);

        %rename(__mul__) operator*;        
        Translation3d operator* (const Translation3d& other);
        Isometry3d operator* (const Isometry3d& other);
        Isometry3d operator* (const AngleAxisd& other);
        Isometry3d operator* (const Quaterniond& other);

        double x();
        double y();
        double z();
        
        %extend
        {            
            void setX(double x)
            {
                $self->x() = x;
            }
            void setY(double y)
            {
                $self->y() = y;
            }
            void setZ(double z)
            {
                $self->z() = z;
            }
        }

    };

    class Isometry3d
    {
    public:
        Isometry3d();
        Isometry3d(const Eigen::Matrix4d& mat);
        
        bool isApprox(const Eigen::Isometry3d& other);
        bool isApprox(const Eigen::Isometry3d& other, double prec);

        Isometry3d inverse();


        Eigen::Matrix4d matrix();
        Eigen::Matrix3d rotation();
        Eigen::Vector3d translation();
        Eigen::Matrix3d linear();

        %extend {
            void setMatrix(const Eigen::Matrix4d& matrix)
            {
                $self->matrix() = matrix;            
            }
            void setTranslation(const Eigen::Vector3d& translation)
            {
                $self->translation() = translation;
            }
            void setLinear(const Eigen::Matrix3d& linear)
            {
                $self->linear() = linear;
            }
        }

        %rename(__mul__) operator*;
        Isometry3d operator* (const AngleAxisd& other);
        Isometry3d operator* (const Quaterniond& other);
        Isometry3d operator* (const Isometry3d& other);
        Isometry3d operator* (const Translation3d& other);
        Eigen::Matrix4d operator* (const Eigen::Matrix4d& other);

        void rotate(const Eigen::Matrix3d& rotation);
        void rotate(const AngleAxisd& rotation);
        void rotate(const Quaterniond& rotation);
        void translate(const Eigen::Vector3d& vec);
        
        void prerotate(const Eigen::Matrix3d& rotation);
        void prerotate(const AngleAxisd& rotation);
        void prerotate(const Quaterniond& rotation);
        void pretranslate(const Eigen::Vector3d& vec);

        void setIdentity();

        static Isometry3d Identity();

    };
}

// Argout: & (for returning values to in-out arguments)
%typemap(argout) Eigen::Isometry3d &
{
  // Argout: &
  PyObject* ret1 = $result;
  //  PyObject* ret2 = SWIG_NewPointerObj(SWIG_as_voidptr($1), SWIGTYPE_p_Eigen__Isometry3d, SWIG_POINTER_NEW |  0 );  
  PyObject* ret2 = SWIG_NewPointerObj(%new_copy(*$1, $*ltype), $descriptor, SWIG_POINTER_OWN | %newpointer_flags);
  $result = PyTuple_Pack(2, ret1, ret2);
  Py_DECREF(ret1);
  Py_DECREF(ret2);
}

%typemap(in, numinputs=0) Eigen::Isometry3d& (Eigen::Isometry3d temp) {
  $1 = &temp;
}

// Default typemap for const & Eigen::Isometry3d
%typemap(in, noblock=1) Eigen::Isometry3d const& (void *argp = 0, int res = 0) {
  res = SWIG_ConvertPtr($input, &argp, $descriptor, %convertptr_flags);
  if (!SWIG_IsOK(res)) {
    %argument_fail(res, "$type", $symname, $argnum); 
  }
  if (!argp) { %argument_nullref("$type", $symname, $argnum); }
  $1 = %reinterpret_cast(argp, $ltype);
}
%typemap(freearg) Eigen::Isometry3d const& "";
%typemap(argout) Eigen::Isometry3d const& "";
