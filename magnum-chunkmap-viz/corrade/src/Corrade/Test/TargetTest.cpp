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

/* Including this first so we test it guesses everything properly without
   accidental help from standard headers */
#include "Corrade/configure.h"

#include <sstream>

#include "Corrade/TestSuite/Tester.h"
#include "Corrade/Utility/DebugStl.h" /** @todo remove when <sstream> is gone */

#include "configure.h"

namespace Corrade { namespace Test { namespace {

struct TargetTest: TestSuite::Tester {
    explicit TargetTest();

    void system();
    void architecture();
    void bitness();
    void endian();
    void compiler();
    #if defined(CORRADE_TARGET_MSVC) && !defined(CORRADE_TARGET_CLANG)
    void msvcPermissiveFlag();
    #endif
    void stl();
    void simd();
};

TargetTest::TargetTest() {
    addTests({&TargetTest::system,
              &TargetTest::architecture,
              &TargetTest::bitness,
              &TargetTest::endian,
              &TargetTest::compiler,
              #if defined(CORRADE_TARGET_MSVC) && !defined(CORRADE_TARGET_CLANG)
              &TargetTest::msvcPermissiveFlag,
              #endif
              &TargetTest::stl,
              &TargetTest::simd});
}

void TargetTest::system() {
    std::ostringstream out;
    #ifdef CORRADE_TARGET_APPLE
    Debug{&out} << "CORRADE_TARGET_APPLE";
    #ifndef CORRADE_TARGET_UNIX
    CORRADE_VERIFY(!"CORRADE_TARGET_APPLE defined but CORRADE_TARGET_UNIX not");
    #endif
    #endif

    #ifdef CORRADE_TARGET_ANDROID
    Debug{&out} << "CORRADE_TARGET_ANDROID";
    #ifndef CORRADE_TARGET_UNIX
    CORRADE_VERIFY(!"CORRADE_TARGET_ANDROID defined but CORRADE_TARGET_UNIX not");
    #endif
    #endif

    #ifdef CORRADE_TARGET_EMSCRIPTEN
    Debug{&out} << "CORRADE_TARGET_EMSCRIPTEN";
    #ifdef CORRADE_TARGET_UNIX
    CORRADE_VERIFY(!"CORRADE_TARGET_EMSCRIPTEN defined but CORRADE_TARGET_UNIX as well");
    #endif
    #endif

    #ifdef CORRADE_TARGET_UNIX
    Debug{&out} << "CORRADE_TARGET_UNIX";
    #endif

    #ifdef CORRADE_TARGET_WINDOWS_RT
    Debug{&out} << "CORRADE_TARGET_WINDOWS_RT";
    #ifndef CORRADE_TARGET_WINDOWS
    CORRADE_VERIFY(!"CORRADE_TARGET_WINDOWS_RT defined but CORRADE_TARGET_WINDOWS not");
    #endif
    #ifdef CORRADE_TARGET_UNIX
    CORRADE_VERIFY(!"CORRADE_TARGET_WINDOWS_RT defined but CORRADE_TARGET_UNIX as well");
    #endif
    #endif

    #ifdef CORRADE_TARGET_WINDOWS
    Debug{&out} << "CORRADE_TARGET_WINDOWS";
    #ifdef CORRADE_TARGET_UNIX
    CORRADE_VERIFY(!"CORRADE_TARGET_WINDOWS defined but CORRADE_TARGET_UNIX as well");
    #endif
    #endif

    Debug{Debug::Flag::NoNewlineAtTheEnd} << out.str();
    CORRADE_VERIFY(!out.str().empty() || !"No suitable CORRADE_TARGET_* defined");
}

void TargetTest::architecture() {
    std::ostringstream out;
    int unique = 0;

    #ifdef CORRADE_TARGET_X86
    ++unique;
    Debug{&out} << "CORRADE_TARGET_X86";
    #endif

    #ifdef CORRADE_TARGET_ARM
    ++unique;
    Debug{&out} << "CORRADE_TARGET_ARM";
    #endif

    #ifdef CORRADE_TARGET_POWERPC
    ++unique;
    Debug{&out} << "CORRADE_TARGET_POWERPC";
    #endif

    #ifdef CORRADE_TARGET_WASM
    ++unique;
    Debug{&out} << "CORRADE_TARGET_WASM";
    #endif

    Debug{Debug::Flag::NoNewlineAtTheEnd} << out.str();
    CORRADE_VERIFY(!out.str().empty() || !"No suitable CORRADE_TARGET_* defined");
    CORRADE_COMPARE(unique, 1);
}

void TargetTest::bitness() {
    #ifdef CORRADE_TARGET_32BIT
    Debug{} << "CORRADE_TARGET_32BIT";
    #endif

    #ifdef CORRADE_TARGET_32BIT
    CORRADE_COMPARE(sizeof(void*), 4);
    #else
    CORRADE_COMPARE(sizeof(void*), 8);
    #endif
}

void TargetTest::endian() {
    #ifdef CORRADE_TARGET_BIG_ENDIAN
    Debug{} << "CORRADE_TARGET_BIG_ENDIAN";
    #endif

    union {
        char bytes[4];
        int number;
    } caster;
    caster.number = 0x03020100;
    #ifdef CORRADE_TARGET_BIG_ENDIAN
    CORRADE_COMPARE(caster.bytes[0], 3);
    #else
    CORRADE_COMPARE(caster.bytes[0], 0);
    #endif
}

void TargetTest::compiler() {
    std::ostringstream out;

    #ifdef CORRADE_TARGET_GCC
    Debug{&out} << "CORRADE_TARGET_GCC";
    #endif

    #ifdef CORRADE_TARGET_CLANG
    Debug{&out} << "CORRADE_TARGET_CLANG";
    #endif

    #ifdef CORRADE_TARGET_APPLE_CLANG
    Debug{&out} << "CORRADE_TARGET_APPLE_CLANG";
    #endif

    #ifdef CORRADE_TARGET_CLANG_CL
    Debug{&out} << "CORRADE_TARGET_CLANG_CL";
    #endif

    #ifdef CORRADE_TARGET_MSVC
    Debug{&out} << "CORRADE_TARGET_MSVC";
    #endif

    #ifdef CORRADE_TARGET_MINGW
    Debug{&out} << "CORRADE_TARGET_MINGW";
    #endif

    Debug{Debug::Flag::NoNewlineAtTheEnd} << out.str();
    CORRADE_VERIFY(!out.str().empty() || !"No suitable CORRADE_TARGET_* defined");

    #if defined(CMAKE_CORRADE_TARGET_GCC) != defined(CORRADE_TARGET_GCC)
    CORRADE_VERIFY(!"Inconsistency in CMake-defined CORRADE_TARGET_GCC");
    #endif

    #if defined(CMAKE_CORRADE_TARGET_CLANG) != defined(CORRADE_TARGET_CLANG)
    CORRADE_VERIFY(!"Inconsistency in CMake-defined CORRADE_TARGET_CLANG");
    #endif

    #if defined(CMAKE_CORRADE_TARGET_APPLE_CLANG) != defined(CORRADE_TARGET_APPLE_CLANG)
    CORRADE_VERIFY(!"Inconsistency in CMake-defined CORRADE_TARGET_APPLE_CLANG");
    #endif

    #if defined(CMAKE_CORRADE_TARGET_CLANG_CL) != defined(CORRADE_TARGET_CLANG_CL)
    CORRADE_VERIFY(!"Inconsistency in CMake-defined CORRADE_TARGET_CLANG_CL");
    #endif

    #if defined(CMAKE_CORRADE_TARGET_MSVC) != defined(CORRADE_TARGET_MSVC)
    CORRADE_VERIFY(!"Inconsistency in CMake-defined CORRADE_TARGET_MSVC");
    #endif

    #if defined(CMAKE_CORRADE_TARGET_MINGW) != defined(CORRADE_TARGET_MINGW)
    CORRADE_VERIFY(!"Inconsistency in CMake-defined CORRADE_TARGET_MINGW");
    #endif

    #if defined(CORRADE_TARGET_CLANG) && defined(CORRADE_TARGET_MSVC) == defined(CORRADE_TARGET_GCC)
    CORRADE_VERIFY(!"Clang should have either a MSVC or a GCC frontend, but not both");
    #endif
}

#if defined(CORRADE_TARGET_MSVC) && !defined(CORRADE_TARGET_CLANG)
void TargetTest::msvcPermissiveFlag() {
    /* Without /permissive- (or with /permissive- /Zc:ternary-, or with just
       /Zc:ternary), it'd be sizeof(const char*) instead. Yes, it's not
       bulletproof, but of all cases shown here it's the cheapest check:
       https://docs.microsoft.com/en-us/cpp/build/reference/permissive-standards-conformance?view=msvc-170#ambiguous-conditional-operator-arguments */
    CORRADE_INFO((sizeof(1 ? "" : "") == 1 ?
        "Standards-conforming C++ parser, compiled with the /permissive- flag" :
        "Non-conforming C++ parser, compiled without /permissive-"));

    #ifdef CORRADE_MSVC_COMPATIBILITY
    if(sizeof(1 ? "" : "") == 1)
        CORRADE_WARN("CORRADE_MSVC_COMPATIBILITY set, but the parser is standards-conforming.");
    CORRADE_VERIFY(true);
    #else
    CORRADE_FAIL_IF(sizeof(1 ? "" : "") != 1,
        "CORRADE_MSVC_COMPATIBILITY not set and the parser is not standards-conforming.");
    #endif
}
#endif

void TargetTest::stl() {
    std::ostringstream out;
    int unique = 0;

    #ifdef CORRADE_TARGET_LIBSTDCXX
    ++unique;
    Debug{&out} << "CORRADE_TARGET_LIBSTDCXX";
    #endif

    #ifdef CORRADE_TARGET_LIBCXX
    ++unique;
    Debug{&out} << "CORRADE_TARGET_LIBCXX";
    #endif

    #ifdef CORRADE_TARGET_DINKUMWARE
    ++unique;
    Debug{&out} << "CORRADE_TARGET_DINKUMWARE";
    #endif

    Debug{Debug::Flag::NoNewlineAtTheEnd} << out.str();
    CORRADE_VERIFY(!out.str().empty() || !"No suitable CORRADE_TARGET_* defined");
    CORRADE_COMPARE(unique, 1);
}

void TargetTest::simd() {
    std::ostringstream out;

    #ifdef CORRADE_TARGET_X86
    #ifdef CORRADE_TARGET_SSE2
    Debug{&out} << "CORRADE_TARGET_SSE2";
    #endif

    #ifdef CORRADE_TARGET_SSE3
    Debug{&out} << "CORRADE_TARGET_SSE3";
    #ifndef CORRADE_TARGET_SSE2
    CORRADE_VERIFY(!"CORRADE_TARGET_SSE3 defined but CORRADE_TARGET_SSE2 not");
    #endif
    #endif

    #ifdef CORRADE_TARGET_SSSE3
    Debug{&out} << "CORRADE_TARGET_SSSE3";
    #ifndef CORRADE_TARGET_SSE3
    CORRADE_VERIFY(!"CORRADE_TARGET_SSSE3 defined but CORRADE_TARGET_SSE3 not");
    #endif
    #endif

    #ifdef CORRADE_TARGET_SSE41
    Debug{&out} << "CORRADE_TARGET_SSSE41";
    #ifndef CORRADE_TARGET_SSSE3
    CORRADE_VERIFY(!"CORRADE_TARGET_SSE41 defined but CORRADE_TARGET_SSSE3 not");
    #endif
    #endif

    #ifdef CORRADE_TARGET_SSE42
    Debug{&out} << "CORRADE_TARGET_SSSE42";
    #ifndef CORRADE_TARGET_SSE41
    CORRADE_VERIFY(!"CORRADE_TARGET_SSE42 defined but CORRADE_TARGET_SSE41 not");
    #endif
    #endif

    #ifdef CORRADE_TARGET_AVX
    Debug{&out} << "CORRADE_TARGET_AVX";
    #ifndef CORRADE_TARGET_SSE42
    CORRADE_VERIFY(!"CORRADE_TARGET_AVX defined but CORRADE_TARGET_SSE42 not");
    #endif
    #endif

    #ifdef CORRADE_TARGET_AVX_F16C
    Debug{&out} << "CORRADE_TARGET_AVX_F16C";
    #ifndef CORRADE_TARGET_AVX
    CORRADE_VERIFY(!"CORRADE_TARGET_AVX_F16C defined but CORRADE_TARGET_AVX not");
    #endif
    #endif

    #ifdef CORRADE_TARGET_AVX_FMA
    Debug{&out} << "CORRADE_TARGET_AVX_FMA";
    #ifndef CORRADE_TARGET_AVX_F16C
    CORRADE_VERIFY(!"CORRADE_TARGET_AVX_FMA defined but CORRADE_TARGET_AVX_F16C not");
    #endif
    #endif

    #ifdef CORRADE_TARGET_AVX2
    Debug{&out} << "CORRADE_TARGET_AVX2";
    #ifndef CORRADE_TARGET_AVX_FMA
    CORRADE_VERIFY(!"CORRADE_TARGET_AVX2 defined but CORRADE_TARGET_AVX_FMA not");
    #endif
    #endif

    #ifdef CORRADE_TARGET_AVX512F
    Debug{&out} << "CORRADE_TARGET_AVX512F";
    #ifndef CORRADE_TARGET_AVX2
    CORRADE_VERIFY(!"CORRADE_TARGET_AVX512F defined but CORRADE_TARGET_AVX2 not");
    #endif
    #endif
    #elif defined(CORRADE_TARGET_SSE2) || defined(CORRADE_TARGET_SSE3) || defined(CORRADE_TARGET_SSSE3) || defined(CORRADE_TARGET_SSE41) || defined(CORRADE_TARGET_SSE42) || defined(CORRADE_TARGET_AVX) || defined(CORRADE_TARGET_AVX_F16C) || defined(CORRADE_TARGET_AVX_FMA) || defined(CORRADE_TARGET_AVX2) || defined(CORRADE_TARGET_AVX512F)
    CORRADE_VERIFY(!"CORRADE_TARGET_{SSE*,AVX*} defined but CORRADE_TARGET_X86 not");
    #endif

    #ifdef CORRADE_TARGET_ARM
    #ifdef CORRADE_TARGET_NEON
    Debug{&out} << "CORRADE_TARGET_NEON";
    #endif

    #ifdef CORRADE_TARGET_NEON_FP16
    Debug{&out} << "CORRADE_TARGET_NEON_FP16";
    #ifndef CORRADE_TARGET_NEON
    CORRADE_VERIFY(!"CORRADE_TARGET_NEON_FP16 defined but CORRADE_TARGET_NEON not");
    #endif
    #endif

    #ifdef CORRADE_TARGET_NEON_FMA
    Debug{&out} << "CORRADE_TARGET_NEON_FMA";
    #ifndef CORRADE_TARGET_NEON_FP16
    CORRADE_VERIFY(!"CORRADE_TARGET_NEON_FMA defined but CORRADE_TARGET_NEON_FP16 not");
    #endif
    #endif

    #elif defined(CORRADE_TARGET_NEON) || defined(CORRADE_TARGET_NEON_FP16) || defined(CORRADE_TARGET_NEON_FMA)
    CORRADE_VERIFY(!"CORRADE_TARGET_NEON* defined but CORRADE_TARGET_ARM not");
    #endif

    #ifdef CORRADE_TARGET_EMSCRIPTEN
    #ifdef CORRADE_TARGET_SIMD128
    Debug{&out} << "CORRADE_TARGET_SIMD128";
    #endif
    #elif defined(CORRADE_TARGET_SIMD128)
    CORRADE_VERIFY(!"CORRADE_TARGET_SIMD128 defined but CORRADE_TARGET_EMSCRIPTEN not");
    #endif

    Debug{Debug::Flag::NoNewlineAtTheEnd} << out.str();
    if(out.str().empty()) Debug{} << "No suitable CORRADE_TARGET_* defined";
    CORRADE_VERIFY(true);
}

}}}

CORRADE_TEST_MAIN(Corrade::Test::TargetTest)
