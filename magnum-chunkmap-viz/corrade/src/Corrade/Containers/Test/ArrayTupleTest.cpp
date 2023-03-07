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

/* Included as first to check that we *really* don't need the
   [Strided]ArrayView headers for anything. We actually need, but the relevant
   functions are forward-declared so it should work. */
#include "Corrade/Containers/ArrayTuple.h"

#include <sstream>

#include "Corrade/Containers/Array.h"
#include "Corrade/Containers/StridedArrayView.h"
#include "Corrade/Containers/String.h"
#include "Corrade/TestSuite/Tester.h"
#include "Corrade/TestSuite/Compare/Container.h"
#include "Corrade/TestSuite/Compare/Numeric.h"
#include "Corrade/Utility/DebugStl.h"
#include "Corrade/Utility/Format.h"

namespace Corrade { namespace Containers { namespace Test { namespace {

struct ArrayTupleTest: TestSuite::Tester {
    explicit ArrayTupleTest();

    void constructEmpty();
    void constructEmptyArrays();
    void construct();
    void constructNoInit();

    void constructCustomAllocatorDefaultDeleter();
    void constructStatelessDeleter();
    void constructStatefulAlignedNonTriviallyDestructibleDeleter();

    void constructTriviallyDestructible();
    void constructTriviallyDestructibleCustomAllocatorDefaultDeleter();
    void constructTriviallyDestructibleStatelessDeleter();
    void constructTriviallyDestructibleStatefulAlignedNonTriviallyDestructibleDeleter();

    void constructCopy();
    void constructMove();

    void constructBig();

    void allocatorAlignmentEmpty();
    template<int a> void allocatorAlignmentFromItems();
    template<int a> void allocatorAlignmentFromDeleter();

    void convertArray();
    void convertArrayInvalid();

    void release();

    void emplaceConstructItemExplicitInCopyInitialization();
    void copyConstructPlainDeleterStruct();
};

ArrayTupleTest::ArrayTupleTest() {
    addTests({&ArrayTupleTest::constructEmpty,
              &ArrayTupleTest::constructEmptyArrays,
              &ArrayTupleTest::construct,
              &ArrayTupleTest::constructNoInit,

              &ArrayTupleTest::constructCustomAllocatorDefaultDeleter,
              &ArrayTupleTest::constructStatelessDeleter,
              &ArrayTupleTest::constructStatefulAlignedNonTriviallyDestructibleDeleter,

              &ArrayTupleTest::constructTriviallyDestructible,
              &ArrayTupleTest::constructTriviallyDestructibleCustomAllocatorDefaultDeleter,
              &ArrayTupleTest::constructTriviallyDestructibleStatelessDeleter,
              &ArrayTupleTest::constructTriviallyDestructibleStatefulAlignedNonTriviallyDestructibleDeleter,

              &ArrayTupleTest::constructCopy,
              &ArrayTupleTest::constructMove,

              &ArrayTupleTest::constructBig,

              &ArrayTupleTest::allocatorAlignmentEmpty,
              &ArrayTupleTest::allocatorAlignmentFromItems<1>,
              &ArrayTupleTest::allocatorAlignmentFromItems<16>,
              &ArrayTupleTest::allocatorAlignmentFromDeleter<1>,
              &ArrayTupleTest::allocatorAlignmentFromDeleter<16>,

              &ArrayTupleTest::convertArray,
              &ArrayTupleTest::convertArrayInvalid,

              &ArrayTupleTest::release,

              &ArrayTupleTest::emplaceConstructItemExplicitInCopyInitialization,
              &ArrayTupleTest::copyConstructPlainDeleterStruct});
}

void ArrayTupleTest::constructEmpty() {
    /* Const in order to verify const access */
    const ArrayTuple data;
    CORRADE_COMPARE(data.size(), 0);
    CORRADE_VERIFY(!data.data());
    CORRADE_VERIFY(!data.deleter());
}

struct NonCopyable {
    NonCopyable(const NonCopyable&) = delete;
    NonCopyable(NonCopyable&&) = delete;
    NonCopyable& operator=(const NonCopyable&) = delete;
    NonCopyable& operator=(NonCopyable&&) = delete;
    NonCopyable() {
        ++constructed;
        thisPointerSuffix = reinterpret_cast<std::uintptr_t>(this) & 0xff;
    }

    ~NonCopyable() {
        /* Just a check that the deleter is really called on a correct address.
           If it's not, the memory will contain something different and this
           won't match. */
        if(thisPointerSuffix == (reinterpret_cast<std::uintptr_t>(this) & 0xff))
            ++destructed;
    }

    unsigned char thisPointerSuffix;

    static int constructed;
    static int destructed;
};

int NonCopyable::constructed = 0;
int NonCopyable::destructed = 0;

void ArrayTupleTest::constructEmptyArrays() {
    NonCopyable::constructed = 0;
    NonCopyable::destructed = 0;

    {
        ArrayView<char> chars{reinterpret_cast<char*>(1337), 3};
        ArrayView<NonCopyable> noncopyable{reinterpret_cast<NonCopyable*>(1337), 3};
        ArrayView<int> ints{reinterpret_cast<int*>(1337), 3};
        StridedArrayView1D<double> strided{ArrayView<double>{reinterpret_cast<double*>(1337), 3}};
        StridedArrayView3D<float> strided3D{ArrayView<float>{reinterpret_cast<float*>(1337), 24}, {2, 3, 4}};
        StridedArrayView2D<char> stridedErased{ArrayView<char>{reinterpret_cast<char*>(1337), 3}, {1, 3}};
        ArrayTuple data{
            {0, chars},
            {0, noncopyable},
            {0, ints},
            {0, strided},
            {{0, 0, 0}, strided3D},
            {Corrade::NoInit, 0, 4, 2, stridedErased}
        };

        CORRADE_COMPARE(data.size(), 0);
        CORRADE_VERIFY(!data.data());
        CORRADE_VERIFY(!data.deleter());

        CORRADE_COMPARE(NonCopyable::constructed, 0);
        CORRADE_COMPARE(NonCopyable::destructed, 0);

        /* Even though this is basically a no-op, all views should be reset to
           empty nullptr ones */
        CORRADE_COMPARE(chars.size(), 0);
        CORRADE_VERIFY(!chars.data());
        CORRADE_COMPARE(noncopyable.size(), 0);
        CORRADE_VERIFY(!noncopyable.data());
        CORRADE_COMPARE(ints.size(), 0);
        CORRADE_VERIFY(!ints.data());

        /* last dimension stride is always set for strided views */
        CORRADE_COMPARE(strided.size(), 0);
        CORRADE_VERIFY(!strided.data());
        CORRADE_COMPARE(strided.stride(), 8);
        CORRADE_COMPARE(strided3D.size(), (Containers::StridedArrayView3D<float>::Size{0, 0, 0}));
        CORRADE_COMPARE(strided3D.stride(),  (Containers::StridedArrayView3D<float>::Stride{0, 0, 4}));
        CORRADE_VERIFY(!strided3D.data());

        /* second dimension size and stride is always set for erased views */
        CORRADE_COMPARE(stridedErased.size(), (Containers::StridedArrayView2D<char>::Size{0, 4}));
        CORRADE_COMPARE(stridedErased.stride(),  (Containers::StridedArrayView2D<char>::Stride{4, 1}));
        CORRADE_VERIFY(!stridedErased.data());
    }

    CORRADE_COMPARE(NonCopyable::constructed, 0);
    CORRADE_COMPARE(NonCopyable::destructed, 0);
}

template<int align> struct alignas(align) Aligned {
    Aligned() {
        ++constructed;
    }

    ~Aligned() {
        ++destructed;
    }

    static int constructed;
    static int destructed;
};

template<int align> int Aligned<align>::constructed = 0;
template<int align> int Aligned<align>::destructed = 0;

void ArrayTupleTest::construct() {
    NonCopyable::constructed = 0;
    NonCopyable::destructed = 0;
    Aligned<16>::constructed = 0;
    Aligned<16>::destructed = 0;

    {
        ArrayView<char> chars;
        ArrayView<NonCopyable> noncopyable;
        ArrayView<int> ints;
        ArrayView<Aligned<16>> aligned;
        StridedArrayView1D<double> strided;
        StridedArrayView3D<float> strided3D;
        StridedArrayView2D<char> stridedErased;
        ArrayTuple data{
            {17, chars},
            {4, noncopyable},
            {7, ints},
            {3, aligned},
            {5, strided},
            {{2, 1, 4}, strided3D},
            /* There's no ValueInit alternative for the type-erased variant */
            {Corrade::NoInit, 3, 4, 16, stridedErased}
        };

        /* Check base properties */
        CORRADE_COMPARE(data.size(),
            sizeof(void*) +         /* destructible item count */
            3*(4*sizeof(void*)) +   /* two destructible items + deleter */
            17 +                    /* chars, no padding */
            4 + 3 +                 /* noncopyable + padding to align ints */
            7*4 +                   /* ints + padding to align aligned */
                (sizeof(void*) == 4 ? 8 : 4) +
            3*16 +                  /* aligned */
            5*8 +                   /* strided, right after overaligned so no
                                       padding */
            8*4 +                   /* strided 3D, right after a field with
                                       higher padding */
                8 +                 /* padding to align the next to 16 again */
            3*4                     /* 12 bytes, but aligned to 16 */
        );
        CORRADE_VERIFY(data.data());
        /* Custom deleter to call the destructors */
        CORRADE_VERIFY(data.deleter());

        /* Check array sizes, strides and offsets */
        CORRADE_COMPARE(chars.size(), 17);
        CORRADE_COMPARE(noncopyable.size(), 4);
        CORRADE_COMPARE(ints.size(), 7);
        CORRADE_COMPARE(aligned.size(), 3);
        CORRADE_COMPARE(strided.size(), 5);
        CORRADE_COMPARE(strided.stride(), 8);
        CORRADE_COMPARE(strided3D.size(), (Containers::StridedArrayView3D<float>::Size{2, 1, 4}));
        CORRADE_COMPARE(strided3D.stride(),  (Containers::StridedArrayView3D<float>::Stride{16, 16, 4}));
        CORRADE_COMPARE(stridedErased.size(), (Containers::StridedArrayView2D<char>::Size{3, 4}));
        CORRADE_COMPARE(stridedErased.stride(),  (Containers::StridedArrayView2D<char>::Stride{4, 1}));
        CORRADE_COMPARE(static_cast<void*>(chars.data()), data.data() +
            sizeof(void*) + 3*(4*sizeof(void*)));
        CORRADE_COMPARE(static_cast<void*>(noncopyable.data()), data.data() +
            sizeof(void*) + 3*(4*sizeof(void*)) + 17);
        CORRADE_COMPARE(static_cast<void*>(ints.data()), data.data() +
            sizeof(void*) + 3*(4*sizeof(void*)) + 17 + 4 + 3);
        CORRADE_COMPARE(static_cast<void*>(aligned.data()), data.data() +
            sizeof(void*) + 3*(4*sizeof(void*)) + 17 + 4 + 3 + 7*4 +
                (sizeof(void*) == 4 ? 8 : 4));
        CORRADE_COMPARE(strided.data(), data.data() +
            sizeof(void*) + 3*(4*sizeof(void*)) + 17 + 4 + 3 + 7*4 +
                (sizeof(void*) == 4 ? 8 : 4) + 3*16);
        CORRADE_COMPARE(strided3D.data(), data.data() +
            sizeof(void*) + 3*(4*sizeof(void*)) + 17 + 4 + 3 + 7*4 +
                (sizeof(void*) == 4 ? 8 : 4) + 3*16 + 5*8);
        CORRADE_COMPARE(stridedErased.data(), data.data() +
            sizeof(void*) + 3*(4*sizeof(void*)) + 17 + 4 + 3 + 7*4 +
                (sizeof(void*) == 4 ? 8 : 4) + 3*16 + 5*8 + 8*4 + 8);

        /* Check that trivial types are zero-init'd and nontrivial had their
           constructor called */
        for(char i: chars) CORRADE_COMPARE(i, 0);
        CORRADE_COMPARE(NonCopyable::constructed, 4);
        CORRADE_COMPARE(NonCopyable::destructed, 0);
        for(char i: ints) CORRADE_COMPARE(i, 0);
        CORRADE_COMPARE(Aligned<16>::constructed, 3);
        CORRADE_COMPARE(Aligned<16>::destructed, 0);
        for(double i: strided) CORRADE_COMPARE(i, 0.0);
        /* MSVC 2015 needs the {}s, FFS */
        for(auto i: strided3D) {
            for(auto j: i) {
                for(float k: j) {
                    CORRADE_COMPARE(k, 0.0f);
                }
            }
        }
    }

    /* Check that non-trivial destructors were called */
    CORRADE_COMPARE(NonCopyable::constructed, 4);
    CORRADE_COMPARE(NonCopyable::destructed, 4);
    CORRADE_COMPARE(Aligned<16>::constructed, 3);
    CORRADE_COMPARE(Aligned<16>::destructed, 3);
}

void ArrayTupleTest::constructNoInit() {
    NonCopyable::constructed = 0;
    NonCopyable::destructed = 0;

    char storage[276];
    for(char& i: storage) i = '\xce';

    CORRADE_VERIFY(true); /* to capture correct function name */

    {
        ArrayView<char> chars;
        ArrayView<char> initializedChars;
        ArrayView<NonCopyable> noncopyable;
        ArrayView<NonCopyable> initializedNoncopyable;
        StridedArrayView1D<double> strided;
        StridedArrayView1D<double> initializedStrided;
        StridedArrayView3D<float> strided3D;
        StridedArrayView2D<float> initializedStrided2D;
        StridedArrayView2D<char> stridedErased;
        ArrayTuple data{
            {{Corrade::NoInit, 15, chars},
             {Corrade::ValueInit, 15, initializedChars},
             {Corrade::NoInit, 3, noncopyable},
             {Corrade::ValueInit, 2, initializedNoncopyable},
             {Corrade::NoInit, 5, strided},
             {Corrade::ValueInit, 4, initializedStrided},
             {Corrade::NoInit, {1, 2, 3}, strided3D},
             {Corrade::ValueInit, {3, 2}, initializedStrided2D},
             {Corrade::NoInit, 3, 4, 4, stridedErased}},
            [&](std::size_t size, std::size_t) -> std::pair<char*, void(*)(char*, std::size_t)> {
                CORRADE_COMPARE_AS(size, Containers::arraySize(storage),
                    TestSuite::Compare::LessOrEqual);
                return {storage, [](char*, std::size_t) {}};
            }
        };

        /* Verify that NoInit stayed at 0xce, while the ValueInit are 0x0 and
           only the constructors for the ValueInit'd view were called */
        for(char i: chars) CORRADE_COMPARE(i, '\xce');
        for(char i: initializedChars) CORRADE_COMPARE(i, 0);
        for(auto i: Containers::arrayCast<2, char>(strided))
            CORRADE_COMPARE_AS(i, Containers::stridedArrayView({
                '\xce', '\xce', '\xce', '\xce', '\xce', '\xce', '\xce', '\xce'
            }), TestSuite::Compare::Container);
        for(double i: initializedStrided) CORRADE_COMPARE(i, 0.0);
        /* MSVC 2015 needs the {}s, FFS */
        for(auto i: Containers::arrayCast<4, char>(strided3D)) {
            for(auto j: i) {
                for(auto k: j) {
                    CORRADE_COMPARE_AS(k, Containers::stridedArrayView({
                        '\xce', '\xce', '\xce', '\xce'
                    }), TestSuite::Compare::Container);
                }
            }
        }
        /* MSVC 2015 needs the {}s, FFS */
        for(auto i: initializedStrided2D) {
            for(float j: i) {
                CORRADE_COMPARE(j, 0.0f);
            }
        }
        for(auto i: stridedErased)
            CORRADE_COMPARE_AS(i, Containers::stridedArrayView({
                '\xce', '\xce', '\xce', '\xce'
            }), TestSuite::Compare::Container);
        CORRADE_COMPARE(NonCopyable::constructed, 2);
        CORRADE_COMPARE(NonCopyable::destructed, 0);

        /* Construct the remaining NonCopyables, so their destruction is
           correctly reported */
        for(NonCopyable& i: noncopyable)
            new(&i) NonCopyable{};
    }

    /* All destructors are called on destruction */
    CORRADE_COMPARE(NonCopyable::constructed, 5);
    CORRADE_COMPARE(NonCopyable::destructed, 5);
}

void ArrayTupleTest::constructCustomAllocatorDefaultDeleter() {
    NonCopyable::constructed = 0;
    NonCopyable::destructed = 0;

    char* preallocated = new char[256];

    {
        ArrayView<char> chars;
        ArrayView<NonCopyable> noncopyable;
        ArrayTuple data{
            {{15, chars},
             {3, noncopyable}},
            [&](std::size_t, std::size_t) -> std::pair<char*, std::nullptr_t> {
                return {preallocated, nullptr};
            }
        };

        /* The preallocated memory should get used and later deleted using
           the default delete[] */
        CORRADE_COMPARE(data.data(), static_cast<void*>(preallocated));

        /* But the deleter needs to wrap the destructor calls, so it's not
           stored directly as nullptr */
        CORRADE_VERIFY(data.deleter());

        CORRADE_COMPARE(NonCopyable::constructed, 3);
        CORRADE_COMPARE(NonCopyable::destructed, 0);
    }

    CORRADE_COMPARE(NonCopyable::constructed, 3);
    CORRADE_COMPARE(NonCopyable::destructed, 3);
}

char* globalUsedDeleterPointer = nullptr;
std::size_t globalUsedDeleterSize = 0;

void ArrayTupleTest::constructStatelessDeleter() {
    NonCopyable::constructed = 0;
    NonCopyable::destructed = 0;
    globalUsedDeleterPointer = nullptr;
    globalUsedDeleterSize = 0;

    char preallocated[256];
    void(*deleter)(char*, std::size_t) = [](char* data, std::size_t size) {
        globalUsedDeleterPointer = data;
        globalUsedDeleterSize = size;
    };

    const std::size_t expectedSize =
        sizeof(void*) +         /* destructible item count */
        2*(4*sizeof(void*)) +   /* one destructible item + deleter */
        15 + 3;                 /* chars and noncopyable data */

    {
        ArrayView<char> chars;
        ArrayView<NonCopyable> noncopyable;
        ArrayTuple data{
            {{15, chars},
             {3, noncopyable}},
            [&](std::size_t, std::size_t) -> std::pair<char*, void(*)(char*, std::size_t)> {
                return {preallocated, deleter};
            }
        };

        /* The preallocated memory should get used */
        CORRADE_COMPARE(data.data(), static_cast<void*>(preallocated));
        CORRADE_COMPARE(data.size(), expectedSize);

        /* But the deleter needs to wrap the destructor calls, so it's not
           stored directly */
        CORRADE_VERIFY(data.deleter() != deleter);

        CORRADE_COMPARE(NonCopyable::constructed, 3);
        CORRADE_COMPARE(NonCopyable::destructed, 0);
    }

    /* On deletion, correct parameters should get passed to the deleter */
    CORRADE_COMPARE(globalUsedDeleterPointer, static_cast<void*>(preallocated));
    CORRADE_COMPARE(globalUsedDeleterSize, expectedSize);

    CORRADE_COMPARE(NonCopyable::constructed, 3);
    CORRADE_COMPARE(NonCopyable::destructed, 3);
}

struct alignas(16) StatefulAlignedNonTriviallyDestructibleDeleter {
    explicit StatefulAlignedNonTriviallyDestructibleDeleter(void*& thisPointer, char*& usedDeleterPointer, std::size_t& usedDeleterSize, int& copyConstructorCallCount, int& destructorCallCount): _usedThisPointer{&thisPointer}, _usedDeleterPointer{&usedDeleterPointer}, _usedDeleterSize{&usedDeleterSize}, _copyConstructorCallCount{&copyConstructorCallCount}, _destructorCallCount{&destructorCallCount} {}

    void operator()(char* data, std::size_t size) {
        *_usedThisPointer = this;
        *_usedDeleterPointer = data;
        *_usedDeleterSize = size;
    }

    StatefulAlignedNonTriviallyDestructibleDeleter(const StatefulAlignedNonTriviallyDestructibleDeleter& other): _usedThisPointer{other._usedThisPointer}, _usedDeleterPointer{other._usedDeleterPointer}, _usedDeleterSize{other._usedDeleterSize}, _copyConstructorCallCount{other._copyConstructorCallCount}, _destructorCallCount{other._destructorCallCount} {
        ++*_copyConstructorCallCount;
    }

    ~StatefulAlignedNonTriviallyDestructibleDeleter() {
        ++*_destructorCallCount;
    }

    private:
        void** _usedThisPointer;
        char** _usedDeleterPointer;
        std::size_t* _usedDeleterSize;
        int* _copyConstructorCallCount;
        int* _destructorCallCount;
};

void ArrayTupleTest::constructStatefulAlignedNonTriviallyDestructibleDeleter() {
    NonCopyable::constructed = 0;
    NonCopyable::destructed = 0;

    /* StatefulAlignedNonTriviallyDestructibleDeleter is 16-byte aligned and
       if this array is not aligned (like on Android x86), it may cause nasty
       crashes when calling the deleter */
    alignas(16) char preallocated[256];
    void* usedThisPointer;
    char* usedDeleterPointer;
    std::size_t usedDeleterSize;
    int copyConstructorCallCount{}, destructorCallCount{};

    const std::size_t expectedSize =
        sizeof(void*) +         /* destructible item count */
        2*(4*sizeof(void*)) +   /* one destructible item + deleter */
        15 + 3 +                /* chars and noncopyable data */
                                /* padding to align the deleter to 16 bytes */
        (sizeof(void*) == 4 ? 10 : 6) +
        sizeof(StatefulAlignedNonTriviallyDestructibleDeleter);
                                /* and the deleter state */

    {
        ArrayView<char> chars;
        ArrayView<NonCopyable> noncopyable;
        ArrayTuple data{
            {{15, chars},
             {3, noncopyable}},
            [&](std::size_t, std::size_t) -> std::pair<char*, StatefulAlignedNonTriviallyDestructibleDeleter> {
                return {preallocated, StatefulAlignedNonTriviallyDestructibleDeleter{usedThisPointer, usedDeleterPointer, usedDeleterSize, copyConstructorCallCount, destructorCallCount}};
            }
        };

        /* The preallocated memory should get used */
        CORRADE_COMPARE(data.data(), static_cast<void*>(preallocated));
        CORRADE_COMPARE(data.size(), expectedSize);

        /* The deleter is anything but the default one */
        CORRADE_VERIFY(data.deleter());

        CORRADE_COMPARE(NonCopyable::constructed, 3);
        CORRADE_COMPARE(NonCopyable::destructed, 0);
    }

    /* Correct pointer + size should be passed to the deleter */
    CORRADE_COMPARE(usedDeleterPointer, static_cast<void*>(preallocated));
    CORRADE_COMPARE(usedDeleterSize, expectedSize);

    /* The deleter should be copied out of the allocation to prevent it from
       unknowingly accessing gone memory after it frees it */
    CORRADE_VERIFY(usedThisPointer < preallocated || usedThisPointer > preallocated);

    /* Apart from all destructions coming from the copies, one extra destructor
       should be called at the end to match the initial construction */
    CORRADE_COMPARE(destructorCallCount, copyConstructorCallCount + 1);

    CORRADE_COMPARE(NonCopyable::constructed, 3);
    CORRADE_COMPARE(NonCopyable::destructed, 3);
}

void ArrayTupleTest::constructTriviallyDestructible() {
    ArrayView<int> ints;
    ArrayView<char> chars;
    ArrayView<double> doubles;
    ArrayTuple data{
        {3, ints},
        {13, chars},
        {2, doubles}
    };

    CORRADE_VERIFY(data.data());
    CORRADE_COMPARE(data.size(),
        3*4 +
        13 +
        /* 7 / 3 bytes padding after the chars to align doubles. Android x86 is
           the one with 4-byte-aligned doubles. Wonderful, eh? */
        (alignof(double) == 8 ? 7 : 3) +
        2*8
    );

    /* The default deleter is used, as there's nothing to non-trivially
       destruct */
    CORRADE_VERIFY(!data.deleter());

    /* Check array sizes and offsets. No metadata should be anywhere. */
    CORRADE_COMPARE(ints.size(), 3);
    CORRADE_COMPARE(chars.size(), 13);
    CORRADE_COMPARE(doubles.size(), 2);
    CORRADE_COMPARE(static_cast<void*>(ints.data()), data.data());
    CORRADE_COMPARE(static_cast<void*>(chars.data()), data.data() + 3*4);
    CORRADE_COMPARE(static_cast<void*>(doubles.data()), data.data() + 3*4 + 13 + (alignof(double) == 8 ? 7 : 3));
}

void ArrayTupleTest::constructTriviallyDestructibleCustomAllocatorDefaultDeleter() {
    char* preallocated = new char[256];

    ArrayView<int> ints;
    ArrayView<char> chars;
    ArrayView<double> doubles;
    ArrayTuple data{
        {{3, ints},
         {13, chars},
         {2, doubles}},
        [&](std::size_t, std::size_t) -> std::pair<char*, std::nullptr_t> {
            return {preallocated, nullptr};
        }
    };

    CORRADE_VERIFY(data.data());
    CORRADE_COMPARE(data.size(),
        3*4 +
        13 +
        /* 7 / 3 bytes padding after the chars to align doubles. Android x86 is
           the one with 4-byte-aligned doubles. Wonderful, eh? */
        (alignof(double) == 8 ? 7 : 3) +
        2*8
    );

    /* The default deleter is used, as there's nothing to non-trivially
       destruct */
    CORRADE_VERIFY(!data.deleter());

    /* And no metadata at the front here either */
    CORRADE_COMPARE(static_cast<void*>(ints.data()), data.data());
}

void ArrayTupleTest::constructTriviallyDestructibleStatelessDeleter() {
    globalUsedDeleterPointer = nullptr;
    globalUsedDeleterSize = 0;

    char preallocated[256];
    void(*deleter)(char*, std::size_t) = [](char* data, std::size_t size) {
        globalUsedDeleterPointer = data;
        globalUsedDeleterSize = size;
    };

    const std::size_t expectedSize =
        3*4 +
        13 +
        /* 7 / 3 bytes padding after the chars to align doubles. Android x86 is
           the one with 4-byte-aligned doubles. Wonderful, eh? */
        (alignof(double) == 8 ? 7 : 3) +
        2*8;

    {
        ArrayView<int> ints;
        ArrayView<char> chars;
        ArrayView<double> doubles;
        ArrayTuple data{
            {{3, ints},
             {13, chars},
             {2, doubles}},
            [&](std::size_t, std::size_t) -> std::pair<char*, void(*)(char*, std::size_t)> {
                return {preallocated, deleter};
            }
        };

        CORRADE_VERIFY(data.data());
        CORRADE_COMPARE(data.size(), expectedSize);

        /* The stateless deleter is used directly, as there's nothing to
           non-trivially destruct */
        CORRADE_VERIFY(data.deleter() == deleter);

        /* And no metadata at the front here either */
        CORRADE_COMPARE(static_cast<void*>(ints.data()), data.data());
    }

    /* On deletion, correct parameters should get passed to the deleter */
    CORRADE_COMPARE(globalUsedDeleterPointer, static_cast<void*>(preallocated));
    CORRADE_COMPARE(globalUsedDeleterSize, expectedSize);
}

void ArrayTupleTest::constructTriviallyDestructibleStatefulAlignedNonTriviallyDestructibleDeleter() {
    /* StatefulAlignedNonTriviallyDestructibleDeleter is 16-byte aligned and
       if this array is not aligned (like on Android x86), it may cause nasty
       crashes when calling the deleter */
    alignas(16) char preallocated[256];
    void* usedThisPointer;
    char* usedDeleterPointer;
    std::size_t usedDeleterSize;
    int copyConstructorCallCount{}, destructorCallCount{};

    const std::size_t expectedSize =
        sizeof(void*) +     /* destructible item count */
        4*sizeof(void*) +   /* just one destructible item for the deleter */
        3*4 +
        13 +                /* padding after chars to align doubles */
            (sizeof(void*) == 4 ? 3 : 7) +
        2*8 +               /* padding to align the deleter to 16 bytes */
            (sizeof(void*) == 4 ? 0 : 8) +
        sizeof(StatefulAlignedNonTriviallyDestructibleDeleter);
                            /* and the deleter state */

    {
        ArrayView<int> ints;
        ArrayView<char> chars;
        ArrayView<double> doubles;
        ArrayTuple data{
            {{3, ints},
             {13, chars},
             {2, doubles}},
            [&](std::size_t, std::size_t) -> std::pair<char*, StatefulAlignedNonTriviallyDestructibleDeleter> {
                return {preallocated, StatefulAlignedNonTriviallyDestructibleDeleter{usedThisPointer, usedDeleterPointer, usedDeleterSize, copyConstructorCallCount, destructorCallCount}};
            }
        };

        /* The preallocated memory should get used */
        CORRADE_COMPARE(data.data(), static_cast<void*>(preallocated));
        CORRADE_COMPARE(data.size(), expectedSize);

        /* The deleter is anything but the default one */
        CORRADE_VERIFY(data.deleter());
    }

    /* Correct pointer + size should be passed to the deleter */
    CORRADE_COMPARE(usedDeleterPointer, static_cast<void*>(preallocated));
    CORRADE_COMPARE(usedDeleterSize, expectedSize);

    /* The deleter should be copied out of the allocation to prevent it from
       unknowingly accessing gone memory after it frees it */
    CORRADE_VERIFY(usedThisPointer < preallocated || usedThisPointer > preallocated);

    /* Apart from all destructions coming from the copies, one extra destructor
       should be called at the end to match the initial construction */
    CORRADE_COMPARE(destructorCallCount, copyConstructorCallCount + 1);
}

void ArrayTupleTest::constructCopy() {
    CORRADE_VERIFY(!std::is_copy_constructible<ArrayTuple>::value);
    CORRADE_VERIFY(!std::is_copy_constructible<ArrayTuple>::value);
}

void ArrayTupleTest::constructMove() {
    char preallocated[256];
    void(*deleter)(char*, std::size_t) = [](char*, std::size_t) {};

    ArrayView<int> ints;
    ArrayTuple a{
        {{5, ints}},
        [&](std::size_t, std::size_t) -> std::pair<char*, void(*)(char*, std::size_t)> {
            return {preallocated, deleter};
        }
    };

    ArrayTuple b{Utility::move(a)};
    CORRADE_VERIFY(!a.data());
    CORRADE_VERIFY(!a.size());
    CORRADE_VERIFY(!a.deleter());
    CORRADE_COMPARE(b.data(), static_cast<void*>(preallocated));
    CORRADE_COMPARE(b.size(), 20);
    CORRADE_VERIFY(b.deleter() == deleter);

    ArrayTuple c;
    c = Utility::move(b);
    CORRADE_VERIFY(!b.data());
    CORRADE_VERIFY(!b.size());
    CORRADE_VERIFY(!b.deleter());
    CORRADE_COMPARE(c.data(), static_cast<void*>(preallocated));
    CORRADE_COMPARE(c.size(), 20);
    CORRADE_VERIFY(c.deleter() == deleter);

    CORRADE_VERIFY(std::is_nothrow_move_constructible<ArrayTuple>::value);
    CORRADE_VERIFY(std::is_nothrow_move_assignable<ArrayTuple>::value);
}

struct Big {
    Big(const Big&) = delete;
    Big(Big&&) = delete;
    Big& operator=(const Big&) = delete;
    Big& operator=(Big&&) = delete;
    Big() {
        ++constructed;
        thisPointer = this;
        thisPointer2 = this;
    }

    ~Big() {
        /* Just a check that the deleter is really called on a correct address.
           If it's not, the memory will contain something different and this
           won't match. */
        if(thisPointer == this && thisPointer2 == this)
            ++destructed;
    }

    Big* thisPointer;
    Big* thisPointer2;

    static int constructed;
    static int destructed;
};

int Big::constructed = 0;
int Big::destructed = 0;

void ArrayTupleTest::constructBig() {
    Big::constructed = 0;
    Big::destructed = 0;

    {
        ArrayView<char> chars;
        ArrayView<Big> bigs;
        ArrayTuple data{
            {17, chars},
            {7, bigs}
        };

        /* Check base properties */
        CORRADE_COMPARE(data.size(),
            sizeof(void*) +         /* destructible item count */
            2*(4*sizeof(void*)) +   /* one destructible item + deleter */
            17 +                    /* chars, padding */
                (sizeof(void*) == 4 ? 3 : 7) +
            7*sizeof(Big)           /* bigs */
        );
        CORRADE_VERIFY(data.data());
        /* Custom deleter to call the destructors */
        CORRADE_VERIFY(data.deleter());

        /* Check array sizes and offsets */
        CORRADE_COMPARE(chars.size(), 17);
        CORRADE_COMPARE(bigs.size(), 7);
        CORRADE_COMPARE(static_cast<void*>(chars.data()), data.data() +
            sizeof(void*) + 2*(4*sizeof(void*)));
        CORRADE_COMPARE(static_cast<void*>(bigs.data()), data.data() +
            sizeof(void*) + 2*(4*sizeof(void*)) + 17 + (sizeof(void*) == 4 ? 3 : 7));

        /* Check that trivial types are zero-init'd and nontrivial had their
           constructor called */
        for(char i: chars) CORRADE_COMPARE(i, 0);
        CORRADE_COMPARE(Big::constructed, 7);
        CORRADE_COMPARE(Big::destructed, 0);
    }

    /* Check that non-trivial destructors were called */
    CORRADE_COMPARE(Big::constructed, 7);
    CORRADE_COMPARE(Big::destructed, 7);
}

void ArrayTupleTest::allocatorAlignmentEmpty() {
    std::size_t alignmentRequirement = ~std::size_t{};

    CORRADE_VERIFY(true); /* Just to register correct function name */

    ArrayTuple data{
        {},
        [&](std::size_t size, std::size_t alignment) -> std::pair<char*, std::nullptr_t> {
            CORRADE_COMPARE(size, 0);
            CORRADE_COMPARE(alignment, 1);
            alignmentRequirement = alignment;
            return {};
        }
    };

    /* Check again to verify the allocator actually got called */
    CORRADE_COMPARE(alignmentRequirement, 1);
}

template<int a> void ArrayTupleTest::allocatorAlignmentFromItems() {
    setTestCaseTemplateName(Utility::format("{}", a));

    CORRADE_VERIFY(true); /* Just to register correct function name */

    ArrayView<Aligned<a>> view;
    ArrayTuple data{
        {{3, view}},
        [&](std::size_t size, std::size_t alignment) -> std::pair<char*, std::nullptr_t> {
            CORRADE_COMPARE(alignment, a);
            return {new char[size], nullptr};
        }
    };

    CORRADE_COMPARE(view.size(), 3);
}

template<int a> void ArrayTupleTest::allocatorAlignmentFromDeleter() {
    setTestCaseTemplateName(Utility::format("{}", a));

    CORRADE_VERIFY(true); /* Just to register correct function name */

    struct alignas(a) Deleter {
        #ifdef CORRADE_MSVC2015_COMPATIBILITY
        /* If this is not present on MSVC2015, the test segfaults. HEH */
        char aehhhhh;
        #endif

        void operator()(char* data, std::size_t) {
            delete[] data;
        }
    };

    ArrayView<char> view;
    ArrayTuple data{
        {{3, view}},
        [&](std::size_t size, std::size_t alignment) -> std::pair<char*, Deleter> {
            CORRADE_COMPARE(alignment, a);
            return {new char[size], {}};
        }
    };

    CORRADE_COMPARE(view.size(), 3);
}

void ArrayTupleTest::convertArray() {
    char preallocated[256]{};
    void(*deleter)(char*, std::size_t) = [](char* data, std::size_t) {
        ++data[255];
    };

    {
        ArrayView<int> ints;
        ArrayView<char> chars;
        ArrayView<double> doubles;
        Array<char> data = ArrayTuple{
            {{3, ints},
            {13, chars},
            {2, doubles}},
            [&](std::size_t, std::size_t) -> std::pair<char*, void(*)(char*, std::size_t)> {
                return {preallocated, deleter};
            }
        };

        CORRADE_VERIFY(data.data());
        CORRADE_COMPARE(data.size(),
            3*4 +
            13 +
            /* 7 / 3 bytes padding after the chars to align doubles. Android
               x86 is the one with 4-byte-aligned doubles. Wonderful, eh? */
            (alignof(double) == 8 ? 7 : 3) +
            2*8
        );

        /* The stateless deleter is used directly, as there's nothing to
        non-trivially destruct */
        CORRADE_VERIFY(data.deleter() == deleter);
    }

    /* Check the deleter was called just once */
    CORRADE_COMPARE(preallocated[255], 1);
}

void ArrayTupleTest::convertArrayInvalid() {
    #ifdef CORRADE_NO_ASSERT
    CORRADE_SKIP("CORRADE_NO_ASSERT defined, can't test assertions");
    #endif

    ArrayView<NonCopyable> noncopyable;
    ArrayTuple nonTrivialData{
        {5, noncopyable}
    };

    struct Deleter {
        int state;
        void operator()(char* data, std::size_t) {
            delete[] data;
        }
    };
    ArrayTuple nonTrivialDeleter{
        {},
        [&](std::size_t size, std::size_t) -> std::pair<char*, Deleter> {
            return {new char[size], {}};
        }
    };

    std::ostringstream out;
    Error redirectError{&out};
    Array<char> a = Utility::move(nonTrivialData);
    Array<char> b = Utility::move(nonTrivialDeleter);
    CORRADE_COMPARE(out.str(),
        "Containers::ArrayTuple: conversion to Array allowed only with trivially destructible types and a stateless destructor\n"
        "Containers::ArrayTuple: conversion to Array allowed only with trivially destructible types and a stateless destructor\n");
}

void ArrayTupleTest::release() {
    NonCopyable::constructed = 0;
    NonCopyable::destructed = 0;

    ArrayView<NonCopyable> noncopyable;
    ArrayTuple data{
        {5, noncopyable}
    };

    std::size_t size = data.size();
    char* pointer = data.data();
    ArrayTuple::Deleter deleter = data.deleter();
    char* released = data.release();
    deleter(released, size);

    /* Not comparing pointers directly because then Clang Analyzer complains
       that printing the value of `released` is use-after-free. Um. */
    CORRADE_COMPARE(reinterpret_cast<std::intptr_t>(pointer), reinterpret_cast<std::intptr_t>(released));
    CORRADE_VERIFY(!data.data());
    CORRADE_COMPARE(data.size(), 0);
    CORRADE_VERIFY(!data.deleter());
}

void ArrayTupleTest::emplaceConstructItemExplicitInCopyInitialization() {
    /* See constructHelpers.h for details about this compiler-specific issue */
    struct ExplicitDefault {
        explicit ExplicitDefault() = default;
    };

    struct ContainingExplicitDefaultWithImplicitConstructor {
        ExplicitDefault a;
    };

    /* This alone works */
    ContainingExplicitDefaultWithImplicitConstructor a;
    static_cast<void>(a);

    /* So this should too */
    ArrayView<ContainingExplicitDefaultWithImplicitConstructor> view;
    ArrayTuple data{
        {3, view}
    };
    CORRADE_COMPARE(data.size(), 3);
}

void ArrayTupleTest::copyConstructPlainDeleterStruct() {
    struct ExtremelyTrivialDeleter {
        int a;
        char b;

        void operator()(char*, std::size_t) {}
    };

    char storage[256];
    ArrayView<int> view;

    /* This needs special handling on GCC 4.8, where T{b} (copy-construction)
       attempts to convert ExtremelyTrivial to int to initialize the first
       argument and fails miserably. */
    ArrayTuple data{
        {{5, view}},
        [&](std::size_t, std::size_t) -> std::pair<char*, ExtremelyTrivialDeleter> {
            return {storage, ExtremelyTrivialDeleter{}};
        }
    };

    CORRADE_COMPARE(view.size(), 5);
}

}}}}

CORRADE_TEST_MAIN(Corrade::Containers::Test::ArrayTupleTest)
