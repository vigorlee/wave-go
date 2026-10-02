/**
 * @file startIdx.hpp
 * @brief
 * @author Liuzhao Li (deionllz@foxmail.com)
 * @version 1.0
 * @date 2024-09-27
 * @copyright Copyright (C) 2024 具身智人(北京)科技有限公司
 */
#ifndef GET_START_INDEX_H_
#define GET_START_INDEX_H_

#include "src/SubManifold.hpp"
#include "src/vectview.hpp"

#include <Eigen/Core>

namespace MTK
{

    template <class Base, class T, int idx, int dim>
    int getStartIdx(MTK::SubManifold<T, idx, dim> Base::*)
    {
        return idx;
    }

    template <class Base, class T, int idx, int dim>
    int getStartIdx_(MTK::SubManifold<T, idx, dim> Base::*)
    {
        return dim;
    }

    template <class Base, class T, int idx, int dim>
    int getDof(MTK::SubManifold<T, idx, dim> Base::*)
    {
        return T::DOF;
    }
    template <class Base, class T, int idx, int dim>
    int getDim(MTK::SubManifold<T, idx, dim> Base::*)
    {
        return T::DIM;
    }

    template <class Base, class T, int idx, int dim>
    void setDiagonal(Eigen::Matrix<typename Base::scalar, Base::DOF, Base::DOF>& cov, MTK::SubManifold<T, idx, dim> Base::*,
        const typename Base::scalar& val)
    {
        cov.diagonal().template segment<T::DOF>(idx).setConstant(val);
    }

    template <class Base, class T, int idx, int dim>
    void setDiagonal_(Eigen::Matrix<typename Base::scalar, Base::DIM, Base::DIM>& cov, MTK::SubManifold<T, idx, dim> Base::*,
        const typename Base::scalar& val)
    {
        cov.diagonal().template segment<T::DIM>(dim).setConstant(val);
    }

    template <class Base, class T1, int idx1, int dim1, class T2, int idx2, int dim2>
    typename MTK::internal::CovBlock<Base, T1, T2>::Type subblock(Eigen::Matrix<typename Base::scalar, Base::DOF, Base::DOF>& cov,
        MTK::SubManifold<T1, idx1, dim1> Base::*, MTK::SubManifold<T2, idx2, dim2> Base::*)
    {
        return cov.template block<T1::DOF, T2::DOF>(idx1, idx2);
    }

    template <class Base, class T1, int idx1, int dim1, class T2, int idx2, int dim2>
    typename MTK::internal::CovBlock_<Base, T1, T2>::Type subblock_(Eigen::Matrix<typename Base::scalar, Base::DIM, Base::DIM>& cov,
        MTK::SubManifold<T1, idx1, dim1> Base::*, MTK::SubManifold<T2, idx2, dim2> Base::*)
    {
        return cov.template block<T1::DIM, T2::DIM>(dim1, dim2);
    }

    template <typename Base1, typename Base2, typename T1, typename T2, int idx1, int idx2, int dim1, int dim2>
    typename MTK::internal::CrossCovBlock<Base1, Base2, T1, T2>::Type subblock(
        Eigen::Matrix<typename Base1::scalar, Base1::DOF, Base2::DOF>& cov, MTK::SubManifold<T1, idx1, dim1> Base1::*,
        MTK::SubManifold<T2, idx2, dim2> Base2::*)
    {
        return cov.template block<T1::DOF, T2::DOF>(idx1, idx2);
    }

    template <typename Base1, typename Base2, typename T1, typename T2, int idx1, int idx2, int dim1, int dim2>
    typename MTK::internal::CrossCovBlock_<Base1, Base2, T1, T2>::Type subblock_(
        Eigen::Matrix<typename Base1::scalar, Base1::DIM, Base2::DIM>& cov, MTK::SubManifold<T1, idx1, dim1> Base1::*,
        MTK::SubManifold<T2, idx2, dim2> Base2::*)
    {
        return cov.template block<T1::DIM, T2::DIM>(dim1, dim2);
    }

    template <class Base, class T, int idx, int dim>
    typename MTK::internal::CovBlock_<Base, T, T>::Type subblock_(
        Eigen::Matrix<typename Base::scalar, Base::DIM, Base::DIM>& cov, MTK::SubManifold<T, idx, dim> Base::*)
    {
        return cov.template block<T::DIM, T::DIM>(dim, dim);
    }

    template <class Base, class T, int idx, int dim>
    typename MTK::internal::CovBlock<Base, T, T>::Type subblock(
        Eigen::Matrix<typename Base::scalar, Base::DOF, Base::DOF>& cov, MTK::SubManifold<T, idx, dim> Base::*)
    {
        return cov.template block<T::DOF, T::DOF>(idx, idx);
    }

    template <typename Base>
    class get_cov
    {
    public:
        typedef Eigen::Matrix<typename Base::scalar, Base::DOF, Base::DOF>       type;
        typedef const Eigen::Matrix<typename Base::scalar, Base::DOF, Base::DOF> const_type;
    };

    template <typename Base>
    class get_cov_
    {
    public:
        typedef Eigen::Matrix<typename Base::scalar, Base::DIM, Base::DIM>       type;
        typedef const Eigen::Matrix<typename Base::scalar, Base::DIM, Base::DIM> const_type;
    };

    template <typename Base1, typename Base2>
    class get_cross_cov
    {
    public:
        typedef Eigen::Matrix<typename Base1::scalar, Base1::DOF, Base2::DOF> type;
        typedef const type                                                    const_type;
    };

    template <typename Base1, typename Base2>
    class get_cross_cov_
    {
    public:
        typedef Eigen::Matrix<typename Base1::scalar, Base1::DIM, Base2::DIM> type;
        typedef const type                                                    const_type;
    };

    template <class Base, class T, int idx, int dim>
    vectview<typename Base::scalar, T::DIM> subvector_impl_(
        vectview<typename Base::scalar, Base::DIM> vec, SubManifold<T, idx, dim> Base::*)
    {
        return vec.template segment<T::DIM>(dim);
    }

    template <class Base, class T, int idx, int dim>
    vectview<typename Base::scalar, T::DOF> subvector_impl(vectview<typename Base::scalar, Base::DOF> vec, SubManifold<T, idx, dim> Base::*)
    {
        return vec.template segment<T::DOF>(idx);
    }

    template <class Scalar, int BaseDIM, class Base, class T, int idx, int dim>
    vectview<Scalar, T::DIM> subvector_(vectview<Scalar, BaseDIM> vec, SubManifold<T, idx, dim> Base::*ptr)
    {
        return subvector_impl_(vec, ptr);
    }

    template <class Scalar, int BaseDOF, class Base, class T, int idx, int dim>
    vectview<Scalar, T::DOF> subvector(vectview<Scalar, BaseDOF> vec, SubManifold<T, idx, dim> Base::*ptr)
    {
        return subvector_impl(vec, ptr);
    }

    template <class Scalar, int BaseDOF, class Base, class T, int idx, int dim>
    vectview<Scalar, T::DOF> subvector(Eigen::Matrix<Scalar, BaseDOF, 1>& vec, SubManifold<T, idx, dim> Base::*ptr)
    {
        return subvector_impl(vectview<Scalar, BaseDOF>(vec), ptr);
    }

    template <class Scalar, int BaseDIM, class Base, class T, int idx, int dim>
    vectview<Scalar, T::DIM> subvector_(Eigen::Matrix<Scalar, BaseDIM, 1>& vec, SubManifold<T, idx, dim> Base::*ptr)
    {
        return subvector_impl_(vectview<Scalar, BaseDIM>(vec), ptr);
    }

    template <class Scalar, int BaseDIM, class Base, class T, int idx, int dim>
    vectview<const Scalar, T::DIM> subvector_(const Eigen::Matrix<Scalar, BaseDIM, 1>& vec, SubManifold<T, idx, dim> Base::*ptr)
    {
        return subvector_impl_(vectview<const Scalar, BaseDIM>(vec), ptr);
    }

    template <class Scalar, int BaseDOF, class Base, class T, int idx, int dim>
    vectview<const Scalar, T::DOF> subvector(const Eigen::Matrix<Scalar, BaseDOF, 1>& vec, SubManifold<T, idx, dim> Base::*ptr)
    {
        return subvector_impl(vectview<const Scalar, BaseDOF>(vec), ptr);
    }

    template <class Base, class T, int idx, int dim>
    vectview<const typename Base::scalar, T::DOF> subvector_impl(
        const vectview<const typename Base::scalar, Base::DOF> cvec, SubManifold<T, idx, dim> Base::*)
    {
        return cvec.template segment<T::DOF>(idx);
    }

    template <class Base, class T, int idx, int dim>
    vectview<const typename Base::scalar, T::DIM> subvector_impl_(
        const vectview<const typename Base::scalar, Base::DIM> cvec, SubManifold<T, idx, dim> Base::*)
    {
        return cvec.template segment<T::DIM>(dim);
    }

    template <class Scalar, int BaseDOF, class Base, class T, int idx, int dim>
    vectview<const Scalar, T::DOF> subvector(const vectview<const Scalar, BaseDOF> cvec, SubManifold<T, idx, dim> Base::*ptr)
    {
        return subvector_impl(cvec, ptr);
    }

}  // namespace MTK

#endif
