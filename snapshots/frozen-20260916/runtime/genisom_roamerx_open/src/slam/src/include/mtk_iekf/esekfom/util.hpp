/**
 * @file util.hpp
 * @brief
 * @author Liuzhao Li (deionllz@foxmail.com)
 * @version 1.0
 * @date 2024-09-27
 * @copyright Copyright (C) 2024 具身智人(北京)科技有限公司
 */
#ifndef __MEKFOM_UTIL_HPP__
#define __MEKFOM_UTIL_HPP__

#include "../mtk/src/mtkmath.hpp"

#include <Eigen/Core>
namespace esekfom
{

    template <typename T1, typename T2>
    class is_same
    {
    public:
        operator bool()
        {
            return false;
        }
    };
    template <typename T1>
    class is_same<T1, T1>
    {
    public:
        operator bool()
        {
            return true;
        }
    };

    template <typename T>
    class is_double
    {
    public:
        operator bool()
        {
            return false;
        }
    };

    template <>
    class is_double<double>
    {
    public:
        operator bool()
        {
            return true;
        }
    };

    template <typename T>
    static T id(const T& x)
    {
        return x;
    }

}  // namespace esekfom

#endif
