/**
 * @file SubManifold.hpp
 * @brief
 * @author Liuzhao Li (deionllz@foxmail.com)
 * @version 1.0
 * @date 2024-09-27
 * @copyright Copyright (C) 2024 具身智人(北京)科技有限公司
 */

#ifndef SUBMANIFOLD_HPP_
#define SUBMANIFOLD_HPP_

#include "vectview.hpp"

namespace MTK
{

    template <class T, int idx, int dim>
    struct SubManifold : public T
    {
        enum
        {
            IDX = idx,
            DIM = dim
        };
        typedef T type;

        template <class X>
        explicit SubManifold(const X& t)
            : T(t){};

        SubManifold(const T& t)
            : T(t){};

        using T::operator=;
    };

}  // namespace MTK

#endif
