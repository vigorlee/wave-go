/**
 * @file S2.hpp
 * @brief
 * @author Liuzhao Li (deionllz@foxmail.com)
 * @version 1.0
 * @date 2024-09-27
 * @copyright Copyright (C) 2024 具身智人(北京)科技有限公司
 */
#ifndef S2_H_
#define S2_H_

#include "../src/mtkmath.hpp"
#include "SOn.hpp"
#include "vect.hpp"

namespace MTK
{

    template <class _scalar = double, int den = 1, int num = 1, int S2_typ = 3>
    struct S2
    {
        typedef _scalar                  scalar;
        typedef vect<3, scalar>          vect_type;
        typedef SO3<scalar>              SO3_type;
        typedef typename vect_type::base vec3;
        scalar                           length = scalar(den) / scalar(num);
        enum
        {
            DOF = 2,
            TYP = 1,
            DIM = 3
        };

        vect_type vec;

    public:
        S2()
        {
            if (S2_typ == 3)
                vec = length * vec3(0, 0, std::sqrt(1));
            if (S2_typ == 2)
                vec = length * vec3(0, std::sqrt(1), 0);
            if (S2_typ == 1)
                vec = length * vec3(std::sqrt(1), 0, 0);
        }
        S2(const scalar& x, const scalar& y, const scalar& z)
            : vec(vec3(x, y, z))
        {
            vec.normalize();
            vec = vec * length;
        }

        S2(const vect_type& _vec)
            : vec(_vec)
        {
            vec.normalize();
            vec = vec * length;
        }

        void oplus(MTK::vectview<const scalar, 3> delta, scalar scale = 1)
        {
            SO3_type res;
            res.w() = MTK::exp<scalar, 3>(res.vec(), delta, scalar(scale / 2));
            vec     = res.toRotationMatrix() * vec;
        }

        void boxplus(MTK::vectview<const scalar, 2> delta, scalar scale = 1)
        {
            Eigen::Matrix<scalar, 3, 2> Bx;
            S2_Bx(Bx);
            vect_type Bu = Bx * delta;
            SO3_type  res;
            res.w() = MTK::exp<scalar, 3>(res.vec(), Bu, scalar(scale / 2));
            vec     = res.toRotationMatrix() * vec;
        }

        void boxminus(MTK::vectview<scalar, 2> res, const S2<scalar, den, num, S2_typ>& other) const
        {
            scalar v_sin = (MTK::hat(vec) * other.vec).norm();
            scalar v_cos = vec.transpose() * other.vec;
            scalar theta = std::atan2(v_sin, v_cos);
            if (v_sin < MTK::tolerance<scalar>())
            {
                if (std::fabs(theta) > MTK::tolerance<scalar>())
                {
                    res[0] = 3.1415926;
                    res[1] = 0;
                }
                else
                {
                    res[0] = 0;
                    res[1] = 0;
                }
            }
            else
            {
                S2<scalar, den, num, S2_typ> other_copy = other;
                Eigen::Matrix<scalar, 3, 2>  Bx;
                other_copy.S2_Bx(Bx);
                res = theta / v_sin * Bx.transpose() * MTK::hat(other.vec) * vec;
            }
        }

        void S2_hat(Eigen::Matrix<scalar, 3, 3>& res)
        {
            Eigen::Matrix<scalar, 3, 3> skew_vec;
            skew_vec << scalar(0), -vec[2], vec[1], vec[2], scalar(0), -vec[0], -vec[1], vec[0], scalar(0);
            res = skew_vec;
        }

        void S2_Bx(Eigen::Matrix<scalar, 3, 2>& res)
        {
            if (S2_typ == 3)
            {
                if (vec[2] + length > tolerance<scalar>())
                {
                    res << length - vec[0] * vec[0] / (length + vec[2]), -vec[0] * vec[1] / (length + vec[2]),
                        -vec[0] * vec[1] / (length + vec[2]), length - vec[1] * vec[1] / (length + vec[2]), -vec[0], -vec[1];
                    res /= length;
                }
                else
                {
                    res       = Eigen::Matrix<scalar, 3, 2>::Zero();
                    res(1, 1) = -1;
                    res(2, 0) = 1;
                }
            }
            else if (S2_typ == 2)
            {
                if (vec[1] + length > tolerance<scalar>())
                {
                    res << length - vec[0] * vec[0] / (length + vec[1]), -vec[0] * vec[2] / (length + vec[1]), -vec[0], -vec[2],
                        -vec[0] * vec[2] / (length + vec[1]), length - vec[2] * vec[2] / (length + vec[1]);
                    res /= length;
                }
                else
                {
                    res       = Eigen::Matrix<scalar, 3, 2>::Zero();
                    res(1, 1) = -1;
                    res(2, 0) = 1;
                }
            }
            else
            {
                if (vec[0] + length > tolerance<scalar>())
                {
                    res << -vec[1], -vec[2], length - vec[1] * vec[1] / (length + vec[0]), -vec[2] * vec[1] / (length + vec[0]),
                        -vec[2] * vec[1] / (length + vec[0]), length - vec[2] * vec[2] / (length + vec[0]);
                    res /= length;
                }
                else
                {
                    res       = Eigen::Matrix<scalar, 3, 2>::Zero();
                    res(1, 1) = -1;
                    res(2, 0) = 1;
                }
            }
        }

        void S2_Nx(Eigen::Matrix<scalar, 2, 3>& res, S2<scalar, den, num, S2_typ>& subtrahend)
        {
            if ((vec + subtrahend.vec).norm() > tolerance<scalar>())
            {
                Eigen::Matrix<scalar, 3, 2> Bx;
                S2_Bx(Bx);
                if ((vec - subtrahend.vec).norm() > tolerance<scalar>())
                {
                    scalar v_sin = (MTK::hat(vec) * subtrahend.vec).norm();
                    scalar v_cos = vec.transpose() * subtrahend.vec;

                    res = Bx.transpose()
                          * (std::atan2(v_sin, v_cos) / v_sin * MTK::hat(vec)
                              + MTK::hat(vec) * subtrahend.vec
                                    * ((-v_cos / v_sin / v_sin / length / length / length / length
                                           + std::atan2(v_sin, v_cos) / v_sin / v_sin / v_sin)
                                            * subtrahend.vec.transpose() * MTK::hat(vec) * MTK::hat(vec)
                                        - vec.transpose() / length / length / length / length));
                }
                else
                {
                    res = 1 / length / length * Bx.transpose() * MTK::hat(vec);
                }
            }
            else
            {
                std::cerr << "No N(x, y) for x=-y" << std::endl;
                std::exit(100);
            }
        }

        void S2_Nx_yy(Eigen::Matrix<scalar, 2, 3>& res)
        {
            Eigen::Matrix<scalar, 3, 2> Bx;
            S2_Bx(Bx);
            res = 1 / length / length * Bx.transpose() * MTK::hat(vec);
        }

        void S2_Mx(Eigen::Matrix<scalar, 3, 2>& res, MTK::vectview<const scalar, 2> delta)
        {
            Eigen::Matrix<scalar, 3, 2> Bx;
            S2_Bx(Bx);
            if (delta.norm() < tolerance<scalar>())
            {
                res = -MTK::hat(vec) * Bx;
            }
            else
            {
                vect_type Bu = Bx * delta;
                SO3_type  exp_delta;
                exp_delta.w() = MTK::exp<scalar, 3>(exp_delta.vec(), Bu, scalar(1 / 2));
                res           = -exp_delta.toRotationMatrix() * MTK::hat(vec) * MTK::A_matrix(Bu).transpose() * Bx;
            }
        }

        operator const vect_type&() const
        {
            return vec;
        }

        const vect_type& get_vect() const
        {
            return vec;
        }

        friend S2<scalar, den, num, S2_typ> operator*(const SO3<scalar>& rot, const S2<scalar, den, num, S2_typ>& dir)
        {
            S2<scalar, den, num, S2_typ> ret;
            ret.vec = rot * dir.vec;
            return ret;
        }

        scalar operator[](int idx) const
        {
            return vec[idx];
        }

        friend std::ostream& operator<<(std::ostream& os, const S2<scalar, den, num, S2_typ>& vec)
        {
            return os << vec.vec.transpose() << " ";
        }
        friend std::istream& operator>>(std::istream& is, S2<scalar, den, num, S2_typ>& vec)
        {
            for (int i = 0; i < 3; ++i)
                is >> vec.vec[i];
            vec.vec.normalize();
            vec.vec = vec.vec * vec.length;
            return is;
        }
    };

}  // namespace MTK

#endif
