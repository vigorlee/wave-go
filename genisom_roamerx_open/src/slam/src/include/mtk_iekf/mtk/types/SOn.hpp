/**
 * @file SOn.hpp
 * @brief
 * @author Liuzhao Li (deionllz@foxmail.com)
 * @version 1.0
 * @date 2024-09-27
 * @copyright Copyright (C) 2024 具身智人(北京)科技有限公司
 */
#ifndef SON_H_
#define SON_H_

#include "../src/mtkmath.hpp"
#include "vect.hpp"

#include <Eigen/Geometry>

namespace MTK
{

    template <class _scalar = double, int Options = Eigen::AutoAlign>
    struct SO2 : public Eigen::Rotation2D<_scalar>
    {
        enum
        {
            DOF = 1,
            DIM = 2,
            TYP = 3
        };

        typedef _scalar                    scalar;
        typedef Eigen::Rotation2D<scalar>  base;
        typedef vect<DIM, scalar, Options> vect_type;

        SO2(const scalar& angle = 0)
            : base(angle)
        {
        }

        SO2(const base& src)
            : base(src)
        {
        }

        SO2(const vect_type& vec)
            : base(atan2(vec[1], vec[0])){};

        SO2 operator%(const base& r) const
        {
            return base::inverse() * r;
        }

        template <class Derived>
        vect_type operator%(const Eigen::MatrixBase<Derived>& vec) const
        {
            return base::inverse() * vec;
        }

        SO2 operator/(const SO2& r) const
        {
            return *this * r.inverse();
        }

        operator scalar() const
        {
            return base::angle();
        }
        void S2_hat(Eigen::Matrix<scalar, 3, 3>& res)
        {
            res = Eigen::Matrix<scalar, 3, 3>::Zero();
        }
        void S2_Nx_yy(Eigen::Matrix<scalar, 2, 3>& res)
        {
            std::cerr << "wrong idx for S2" << std::endl;
            std::exit(100);
            res = Eigen::Matrix<scalar, 2, 3>::Zero();
        }

        void S2_Mx(Eigen::Matrix<scalar, 3, 2>& res, MTK::vectview<const scalar, 2> delta)
        {
            std::cerr << "wrong idx for S2" << std::endl;
            std::exit(100);
            res = Eigen::Matrix<scalar, 3, 2>::Zero();
        }

        void oplus(MTK::vectview<const scalar, DOF> vec, scalar scale = 1)
        {
            base::angle() += scale * vec[0];
        }

        void boxplus(MTK::vectview<const scalar, DOF> vec, scalar scale = 1)
        {
            base::angle() += scale * vec[0];
        }
        void boxminus(MTK::vectview<scalar, DOF> res, const SO2<scalar>& other) const
        {
            res[0] = MTK::normalize(base::angle() - other.angle(), scalar(MTK::pi));
        }

        friend std::istream& operator>>(std::istream& is, SO2<scalar>& ang)
        {
            return is >> ang.angle();
        }
    };

    template <class _scalar = double, int Options = Eigen::AutoAlign>
    struct SO3 : public Eigen::Quaternion<_scalar, Options>
    {
        enum
        {
            DOF = 3,
            DIM = 3,
            TYP = 2
        };
        typedef _scalar                            scalar;
        typedef Eigen::Quaternion<scalar, Options> base;
        typedef Eigen::Quaternion<scalar>          Quaternion;
        typedef vect<DIM, scalar, Options>         vect_type;

        template <class OtherDerived>
        EIGEN_STRONG_INLINE Quaternion operator%(const Eigen::QuaternionBase<OtherDerived>& r) const
        {
            return base::conjugate() * r;
        }

        template <class Derived>
        vect_type operator%(const Eigen::MatrixBase<Derived>& vec) const
        {
            return base::conjugate() * vec;
        }

        template <class OtherDerived>
        EIGEN_STRONG_INLINE Quaternion operator/(const Eigen::QuaternionBase<OtherDerived>& r) const
        {
            return *this * r.conjugate();
        }

        SO3(const scalar& w, const scalar& x, const scalar& y, const scalar& z)
            : base(w, x, y, z)
        {
            base::normalize();
        }

        SO3(const base& src = base::Identity())
            : base(src)
        {
        }

        template <class Derived>
        SO3(const Eigen::MatrixBase<Derived>& matrix)
            : base(matrix)
        {
        }

        template <class Derived>
        SO3(const Eigen::RotationBase<Derived, 3>& rotation)
            : base(rotation.derived())
        {
        }

        void boxplus(MTK::vectview<const scalar, DOF> vec, scalar scale = 1)
        {
            SO3 delta = exp(vec, scale);
            *this     = *this * delta;
        }
        void boxminus(MTK::vectview<scalar, DOF> res, const SO3<scalar>& other) const
        {
            res = SO3::log(other.conjugate() * *this);
        }

        void oplus(MTK::vectview<const scalar, DOF> vec, scalar scale = 1)
        {
            SO3 delta = exp(vec, scale);
            *this     = *this * delta;
        }

        void S2_hat(Eigen::Matrix<scalar, 3, 3>& res)
        {
            res = Eigen::Matrix<scalar, 3, 3>::Zero();
        }
        void S2_Nx_yy(Eigen::Matrix<scalar, 2, 3>& res)
        {
            std::cerr << "wrong idx for S2" << std::endl;
            std::exit(100);
            res = Eigen::Matrix<scalar, 2, 3>::Zero();
        }

        void S2_Mx(Eigen::Matrix<scalar, 3, 2>& res, MTK::vectview<const scalar, 2> delta)
        {
            std::cerr << "wrong idx for S2" << std::endl;
            std::exit(100);
            res = Eigen::Matrix<scalar, 3, 2>::Zero();
        }

        friend std::ostream& operator<<(std::ostream& os, const SO3<scalar, Options>& q)
        {
            return os << q.coeffs().transpose() << " ";
        }

        friend std::istream& operator>>(std::istream& is, SO3<scalar, Options>& q)
        {
            vect<4, scalar> coeffs;
            is >> coeffs;
            q.coeffs() = coeffs.normalized();
            return is;
        }

        static SO3 exp(const Eigen::Matrix<scalar, 3, 1>& dvec, scalar scale = 1)
        {
            SO3 res;
            res.w() = MTK::exp<scalar, 3>(res.vec(), dvec, scalar(scale / 2));
            return res;
        }

        static typename base::Vector3 log(const SO3& orient)
        {
            typename base::Vector3 res;
            MTK::log<scalar, 3>(res, orient.w(), orient.vec(), scalar(2), true);
            return res;
        }
    };

    namespace internal
    {
        template <class Scalar, int Options>
        struct UnalignedType<SO2<Scalar, Options>>
        {
            typedef SO2<Scalar, Options | Eigen::DontAlign> type;
        };

        template <class Scalar, int Options>
        struct UnalignedType<SO3<Scalar, Options>>
        {
            typedef SO3<Scalar, Options | Eigen::DontAlign> type;
        };

    }  // namespace internal

}  // namespace MTK

#endif
