#ifndef Corrade_Utility_System_h
#define Corrade_Utility_System_h
/*
    This file is part of Corrade.

    Copyright © 2007, 2008, 2009, 2010, 2011, 2012, 2013, 2014, 2015, 2016,
                2017, 2018, 2019, 2020, 2021, 2022
              Vladimír Vondruš <mosra@centrum.cz>

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
 * @brief Namespace @ref Corrade::Utility::System
 */

#include <cstddef>

#include "Corrade/Utility/visibility.h"

namespace Corrade { namespace Utility {

/**
@brief System utilities

This library is built if `WITH_UTILITY` is enabled when building Corrade. To
use this library with CMake, request the `Utility` component of the `Corrade`
package and link to the `Corrade::Utility` target.

@code{.cmake}
find_package(Corrade REQUIRED Utility)

# ...
target_link_libraries(your-app PRIVATE Corrade::Utility)
@endcode

See also @ref building-corrade and @ref corrade-cmake for more information.
*/
namespace System {

/**
@brief Whether the application runs in a sandboxed environment
@m_since_latest

Returns @cpp true @ce if running on @ref CORRADE_TARGET_IOS "iOS",
@ref CORRADE_TARGET_ANDROID "Android", as a
@ref CORRADE_TARGET_APPLE "macOS" app bundle,
@ref CORRADE_TARGET_WINDOWS_RT "Windows Phone/Store" application or in a
browser through @ref CORRADE_TARGET_EMSCRIPTEN "Emscripten", @cpp false @ce
otherwise.
*/
CORRADE_UTILITY_EXPORT bool isSandboxed();

/** @brief Sleep for given time */
CORRADE_UTILITY_EXPORT void sleep(std::size_t ms);

}}}

#endif
