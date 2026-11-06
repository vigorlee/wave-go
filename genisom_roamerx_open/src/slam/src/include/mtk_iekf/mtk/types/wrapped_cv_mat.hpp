/**
 * @file wrapped_cv_mat.hpp
 * @brief
 * @author Liuzhao Li (deionllz@foxmail.com)
 * @version 1.0
 * @date 2024-09-27
 * @copyright Copyright (C) 2024 具身智人(北京)科技有限公司
 */
#ifndef WRAPPED_CV_MAT_HPP_
#define WRAPPED_CV_MAT_HPP_

#include <Eigen/Core>
#include <opencv/cv.h>

namespace MTK
{

    template <class f_type>
    struct cv_f_type;

    template <>
    struct cv_f_type<double>
    {
        enum
        {
            value = CV_64F
        };
    };

    template <>
    struct cv_f_type<float>
    {
        enum
        {
            value = CV_32F
        };
    };

    /**
     * cv_mat wraps a CvMat around an Eigen Matrix
     */
    template <int rows, int cols, class f_type = double>
    class cv_mat : public matrix<rows, cols, f_type, cols == 1 ? Eigen::ColMajor : Eigen::RowMajor>
    {
        typedef matrix<rows, cols, f_type, cols == 1 ? Eigen::ColMajor : Eigen::RowMajor> base_type;
        enum
        {
            type_ = cv_f_type<f_type>::value
        };
        CvMat cv_mat_;

    public:
        cv_mat()
        {
            cv_mat_ = cvMat(rows, cols, type_, base_type::data());
        }

        cv_mat(const cv_mat& oth)
            : base_type(oth)
        {
            cv_mat_ = cvMat(rows, cols, type_, base_type::data());
        }

        template <class Derived>
        cv_mat(const Eigen::MatrixBase<Derived>& value)
            : base_type(value)
        {
            cv_mat_ = cvMat(rows, cols, type_, base_type::data());
        }

        template <class Derived>
        cv_mat& operator=(const Eigen::MatrixBase<Derived>& value)
        {
            base_type::operator=(value);
            return *this;
        }

        cv_mat& operator=(const cv_mat& value)
        {
            base_type::operator=(value);
            return *this;
        }

        CvMat* operator&()
        {
            return &cv_mat_;
        }
        const CvMat* operator&() const
        {
            return &cv_mat_;
        }
    };

}  // namespace MTK

#endif
