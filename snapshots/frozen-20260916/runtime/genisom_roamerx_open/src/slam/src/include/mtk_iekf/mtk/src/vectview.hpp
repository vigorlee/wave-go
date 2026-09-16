/**
 * @file vectview.hpp
 * @brief
 * @author Liuzhao Li (deionllz@foxmail.com)
 * @version 1.0
 * @date 2024-09-27
 * @copyright Copyright (C) 2024 具身智人(北京)科技有限公司
 */

#ifndef VECTVIEW_HPP_
#define VECTVIEW_HPP_

#include <Eigen/Core>

namespace MTK
{

    namespace internal
    {
        template <class Base, class T1, class T2>
        struct CovBlock
        {
            typedef typename Eigen::Block<Eigen::Matrix<typename Base::scalar, Base::DOF, Base::DOF>, T1::DOF, T2::DOF>       Type;
            typedef typename Eigen::Block<const Eigen::Matrix<typename Base::scalar, Base::DOF, Base::DOF>, T1::DOF, T2::DOF> ConstType;
        };

        template <class Base, class T1, class T2>
        struct CovBlock_
        {
            typedef typename Eigen::Block<Eigen::Matrix<typename Base::scalar, Base::DIM, Base::DIM>, T1::DIM, T2::DIM>       Type;
            typedef typename Eigen::Block<const Eigen::Matrix<typename Base::scalar, Base::DIM, Base::DIM>, T1::DIM, T2::DIM> ConstType;
        };

        template <typename Base1, typename Base2, typename T1, typename T2>
        struct CrossCovBlock
        {
            typedef typename Eigen::Block<Eigen::Matrix<typename Base1::scalar, Base1::DOF, Base2::DOF>, T1::DOF, T2::DOF>       Type;
            typedef typename Eigen::Block<const Eigen::Matrix<typename Base1::scalar, Base1::DOF, Base2::DOF>, T1::DOF, T2::DOF> ConstType;
        };

        template <typename Base1, typename Base2, typename T1, typename T2>
        struct CrossCovBlock_
        {
            typedef typename Eigen::Block<Eigen::Matrix<typename Base1::scalar, Base1::DIM, Base2::DIM>, T1::DIM, T2::DIM>       Type;
            typedef typename Eigen::Block<const Eigen::Matrix<typename Base1::scalar, Base1::DIM, Base2::DIM>, T1::DIM, T2::DIM> ConstType;
        };

        template <class scalar, int dim>
        struct VectviewBase
        {
            typedef Eigen::Matrix<scalar, dim, 1>      matrix_type;
            typedef typename matrix_type::MapType      Type;
            typedef typename matrix_type::ConstMapType ConstType;
        };

        template <class T>
        struct UnalignedType
        {
            typedef T type;
        };
    }  // namespace internal

    template <class scalar, int dim>
    class vectview : public internal::VectviewBase<scalar, dim>::Type
    {
        typedef internal::VectviewBase<scalar, dim> VectviewBase;

    public:
        typedef typename VectviewBase::matrix_type matrix_type;
        typedef typename VectviewBase::Type        base;
        explicit vectview(scalar* data, int dim_ = dim)
            : base(data, dim_)
        {
        }
        vectview(matrix_type& m)
            : base(m.data(), m.size())
        {
        }
        vectview(const vectview& v)
            : base(v)
        {
        }
        template <class Base>
        vectview(Eigen::VectorBlock<Base, dim> block)
            : base(&block.coeffRef(0), block.size())
        {
        }
        template <class Base, bool PacketAccess>
        vectview(Eigen::Block<Base, dim, 1, PacketAccess> block)
            : base(&block.coeffRef(0), block.size())
        {
        }

        using base::operator=;
        scalar*     data()
        {
            return const_cast<scalar*>(base::data());
        }
    };

    template <class scalar, int dim>
    class vectview<const scalar, dim> : public internal::VectviewBase<scalar, dim>::ConstType
    {
        typedef internal::VectviewBase<scalar, dim> VectviewBase;

    public:
        typedef typename VectviewBase::matrix_type matrix_type;
        typedef typename VectviewBase::ConstType   base;
        explicit vectview(const scalar* data, int dim_ = dim)
            : base(data, dim_)
        {
        }
        template <int options>
        vectview(const Eigen::Matrix<scalar, dim, 1, options>& m)
            : base(m.data())
        {
        }
        template <int options, int phony>
        vectview(const Eigen::Matrix<scalar, 1, dim, options, phony>& m)
            : base(m.data())
        {
        }
        vectview(vectview<scalar, dim> x)
            : base(x.data())
        {
        }
        vectview(const base& x)
            : base(x)
        {
        }
        template <class Base>
        vectview(Eigen::VectorBlock<Base, dim> block)
            : base(&block.coeffRef(0))
        {
        }
        template <class Base, bool PacketAccess>
        vectview(Eigen::Block<Base, dim, 1, PacketAccess> block)
            : base(&block.coeffRef(0))
        {
        }
    };

}  // namespace MTK

#endif
