/**
 * @file build_manifold.hpp
 * @brief
 * @author Liuzhao Li (deionllz@foxmail.com)
 * @version 1.0
 * @date 2024-09-27
 * @copyright Copyright (C) 2024 具身智人(北京)科技有限公司
 */
#ifndef MTK_AUTOCONSTRUCT_HPP_
#define MTK_AUTOCONSTRUCT_HPP_

#include "src/SubManifold.hpp"
#include "startIdx.hpp"

#include <Eigen/Core>
#include <boost/preprocessor/cat.hpp>
#include <boost/preprocessor/seq.hpp>
#include <vector>

#ifndef PARSED_BY_DOXYGEN

#define MTK_APPLY_MACRO_ON_TUPLE(r, macro, tuple) macro tuple

#define MTK_TRANSFORM_COMMA(macro, entries) BOOST_PP_SEQ_ENUM(BOOST_PP_SEQ_TRANSFORM_S(1, MTK_APPLY_MACRO_ON_TUPLE, macro, entries))

#define MTK_TRANSFORM(macro, entries) BOOST_PP_SEQ_FOR_EACH_R(1, MTK_APPLY_MACRO_ON_TUPLE, macro, entries)

#define MTK_CONSTRUCTOR_ARG(type, id)  const type& id = type()
#define MTK_CONSTRUCTOR_COPY(type, id) id(id)
#define MTK_BOXPLUS(type, id)          id.boxplus(MTK::subvector(__vec, &self::id), __scale);
#define MTK_OPLUS(type, id)            id.oplus(MTK::subvector_(__vec, &self::id), __scale);
#define MTK_BOXMINUS(type, id)         id.boxminus(MTK::subvector(__res, &self::id), __oth.id);
#define MTK_S2_hat(type, id)                                                                                                               \
    if (id.IDX == idx)                                                                                                                     \
    {                                                                                                                                      \
        id.S2_hat(res);                                                                                                                    \
    }
#define MTK_S2_Nx_yy(type, id)                                                                                                             \
    if (id.IDX == idx)                                                                                                                     \
    {                                                                                                                                      \
        id.S2_Nx_yy(res);                                                                                                                  \
    }
#define MTK_S2_Mx(type, id)                                                                                                                \
    if (id.IDX == idx)                                                                                                                     \
    {                                                                                                                                      \
        id.S2_Mx(res, dx);                                                                                                                 \
    }
#define MTK_OSTREAM(type, id) << __var.id << " "
#define MTK_ISTREAM(type, id) >> __var.id
#define MTK_S2_state(type, id)                                                                                                             \
    if (id.TYP == 1)                                                                                                                       \
    {                                                                                                                                      \
        S2_state.push_back(std::make_pair(id.IDX, id.DIM));                                                                                \
    }
#define MTK_SO3_state(type, id)                                                                                                            \
    if (id.TYP == 2)                                                                                                                       \
    {                                                                                                                                      \
        (SO3_state).push_back(std::make_pair(id.IDX, id.DIM));                                                                             \
    }
#define MTK_vect_state(type, id)                                                                                                           \
    if (id.TYP == 0)                                                                                                                       \
    {                                                                                                                                      \
        (vect_state).push_back(std::make_pair(std::make_pair(id.IDX, id.DIM), type::DOF));                                                 \
    }

#define MTK_SUBVARLIST(seq, S2state, SO3state)                                                                                             \
    BOOST_PP_FOR_1((BOOST_PP_SEQ_SIZE(seq), BOOST_PP_SEQ_HEAD(seq), BOOST_PP_SEQ_TAIL(seq)(~), 0, 0, S2state, SO3state), MTK_ENTRIES_TEST, \
        MTK_ENTRIES_NEXT, MTK_ENTRIES_OUTPUT)

#define MTK_PUT_TYPE(type, id, dof, dim, S2state, SO3state) MTK::SubManifold<type, dof, dim> id;
#define MTK_PUT_TYPE_AND_ENUM(type, id, dof, dim, S2state, SO3state)                                                                       \
    MTK_PUT_TYPE(type, id, dof, dim, S2state, SO3state)                                                                                    \
    enum                                                                                                                                   \
    {                                                                                                                                      \
        DOF = type::DOF + dof                                                                                                              \
    };                                                                                                                                     \
    enum                                                                                                                                   \
    {                                                                                                                                      \
        DIM = type::DIM + dim                                                                                                              \
    };                                                                                                                                     \
    typedef type::scalar scalar;

#define MTK_ENTRIES_OUTPUT(r, state) MTK_ENTRIES_OUTPUT_I state
#define MTK_ENTRIES_OUTPUT_I(s, head, seq, dof, dim, S2state, SO3state)                                                                    \
    MTK_APPLY_MACRO_ON_TUPLE(                                                                                                              \
        ~, BOOST_PP_IF(BOOST_PP_DEC(s), MTK_PUT_TYPE, MTK_PUT_TYPE_AND_ENUM), (BOOST_PP_TUPLE_REM_2 head, dof, dim, S2state, SO3state))

#define MTK_ENTRIES_TEST(r, state) MTK_TUPLE_ELEM_4_0 state

//! this used to be BOOST_PP_TUPLE_ELEM_4_0:
#define MTK_TUPLE_ELEM_4_0(a, b, c, d, e, f, g) a

#define MTK_ENTRIES_NEXT(r, state) MTK_ENTRIES_NEXT_I state
#define MTK_ENTRIES_NEXT_I(len, head, seq, dof, dim, S2state, SO3state)                                                                    \
    (BOOST_PP_DEC(len), BOOST_PP_SEQ_HEAD(seq), BOOST_PP_SEQ_TAIL(seq), dof + BOOST_PP_TUPLE_ELEM_2_0 head::DOF,                           \
        dim + BOOST_PP_TUPLE_ELEM_2_0 head::DIM, S2state, SO3state)

#endif /* not PARSED_BY_DOXYGEN */

#define MTK_BUILD_MANIFOLD(name, entries)                                                                                                  \
    struct name                                                                                                                            \
    {                                                                                                                                      \
        typedef name                                     self;                                                                             \
        std::vector<std::pair<int, int>>                 S2_state;                                                                         \
        std::vector<std::pair<int, int>>                 SO3_state;                                                                        \
        std::vector<std::pair<std::pair<int, int>, int>> vect_state;                                                                       \
        MTK_SUBVARLIST(entries, S2_state, SO3_state)                                                                                       \
        name(MTK_TRANSFORM_COMMA(MTK_CONSTRUCTOR_ARG, entries))                                                                            \
            : MTK_TRANSFORM_COMMA(MTK_CONSTRUCTOR_COPY, entries)                                                                           \
        {                                                                                                                                  \
        }                                                                                                                                  \
        int getDOF() const                                                                                                                 \
        {                                                                                                                                  \
            return DOF;                                                                                                                    \
        }                                                                                                                                  \
        void boxplus(const MTK::vectview<const scalar, DOF>& __vec, scalar __scale = 1)                                                    \
        {                                                                                                                                  \
            MTK_TRANSFORM(MTK_BOXPLUS, entries)                                                                                            \
        }                                                                                                                                  \
        void oplus(const MTK::vectview<const scalar, DIM>& __vec, scalar __scale = 1)                                                      \
        {                                                                                                                                  \
            MTK_TRANSFORM(MTK_OPLUS, entries)                                                                                              \
        }                                                                                                                                  \
        void boxminus(MTK::vectview<scalar, DOF> __res, const name& __oth) const                                                           \
        {                                                                                                                                  \
            MTK_TRANSFORM(MTK_BOXMINUS, entries)                                                                                           \
        }                                                                                                                                  \
        friend std::ostream& operator<<(std::ostream& __os, const name& __var)                                                             \
        {                                                                                                                                  \
            return __os MTK_TRANSFORM(MTK_OSTREAM, entries);                                                                               \
        }                                                                                                                                  \
        void build_S2_state()                                                                                                              \
        {                                                                                                                                  \
            MTK_TRANSFORM(MTK_S2_state, entries)                                                                                           \
        }                                                                                                                                  \
        void build_vect_state()                                                                                                            \
        {                                                                                                                                  \
            MTK_TRANSFORM(MTK_vect_state, entries)                                                                                         \
        }                                                                                                                                  \
        void build_SO3_state()                                                                                                             \
        {                                                                                                                                  \
            MTK_TRANSFORM(MTK_SO3_state, entries)                                                                                          \
        }                                                                                                                                  \
        void S2_hat(Eigen::Matrix<scalar, 3, 3>& res, int idx)                                                                             \
        {                                                                                                                                  \
            MTK_TRANSFORM(MTK_S2_hat, entries)                                                                                             \
        }                                                                                                                                  \
        void S2_Nx_yy(Eigen::Matrix<scalar, 2, 3>& res, int idx)                                                                           \
        {                                                                                                                                  \
            MTK_TRANSFORM(MTK_S2_Nx_yy, entries)                                                                                           \
        }                                                                                                                                  \
        void S2_Mx(Eigen::Matrix<scalar, 3, 2>& res, Eigen::Matrix<scalar, 2, 1> dx, int idx)                                              \
        {                                                                                                                                  \
            MTK_TRANSFORM(MTK_S2_Mx, entries)                                                                                              \
        }                                                                                                                                  \
        friend std::istream& operator>>(std::istream& __is, name& __var)                                                                   \
        {                                                                                                                                  \
            return __is MTK_TRANSFORM(MTK_ISTREAM, entries);                                                                               \
        }                                                                                                                                  \
    };

#endif
