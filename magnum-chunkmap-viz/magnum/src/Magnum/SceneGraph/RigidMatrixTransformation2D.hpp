#ifndef Magnum_SceneGraph_RigidMatrixTransformation2D_hpp
#define Magnum_SceneGraph_RigidMatrixTransformation2D_hpp
/*
    This file is part of Magnum.

    Copyright © 2010, 2011, 2012, 2013, 2014, 2015, 2016, 2017, 2018, 2019,
                2020, 2021, 2022 Vladimír Vondruš <mosra@centrum.cz>

    Permission is hereby granted, free of charge, to any person obtaining a
    copy of this software and associated documentation files (the "Software"),
    to deal in the Software without restriction, including without limitation
    the rights to use, copy, modify, merge, publish, distribute, sublicense,
    and/or sell copies of the Software, and to permit persons to whom the
    Software is furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included
    in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
    THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
    DEALINGS IN THE SOFTWARE.
*/

/** @file
 * @brief @ref compilation-speedup-hpp "Template implementation" for @ref RigidMatrixTransformation2D.h
 * @m_since{2020,06}
 */

#include "RigidMatrixTransformation2D.h"

#include "Magnum/Math/Complex.h"

namespace Magnum { namespace SceneGraph {

/* These are here to avoid including Complex in RigidMatrixTransformation2D.h */

template<class T> Object<BasicRigidMatrixTransformation2D<T>>& BasicRigidMatrixTransformation2D<T>::rotate(const Math::Complex<T>& complex) {
    return transform(Math::Matrix3<T>::from(complex.toMatrix(), {}));
}

template<class T> Object<BasicRigidMatrixTransformation2D<T>>& BasicRigidMatrixTransformation2D<T>::rotateLocal(const Math::Complex<T>& complex) {
    return transformLocal(Math::Matrix3<T>::from(complex.toMatrix(), {}));
}

}}

#endif
