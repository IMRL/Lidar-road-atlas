/*
    This file is part of Magnum.

    Copyright © 2010, 2011, 2012, 2013, 2014, 2015, 2016, 2017, 2018, 2019,
                2020, 2021, 2022 Vladimír Vondruš <mosra@centrum.cz>
    Copyright © 2013 Jan Dupal <dupal.j@gmail.com>
    Copyright © 2016 Jonathan Hale <squareys@googlemail.com>

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

#include <sstream>
#include <Corrade/TestSuite/Tester.h>
#include <Corrade/Utility/DebugStl.h>

#include "Magnum/BulletIntegration/DebugDraw.h"

namespace Magnum { namespace BulletIntegration { namespace Test { namespace {

struct DebugDrawTest: TestSuite::Tester {
    explicit DebugDrawTest();

    void constructNoInit();
    void constructCopy();

    void debugMode();
};

DebugDrawTest::DebugDrawTest() {
    addTests({&DebugDrawTest::constructNoInit,
              &DebugDrawTest::constructCopy,
              &DebugDrawTest::debugMode});
}

void DebugDrawTest::constructNoInit() {
    {
        DebugDraw debugDraw{NoCreate};
    }

    CORRADE_VERIFY(true);
}

void DebugDrawTest::constructCopy() {
    CORRADE_VERIFY(!std::is_constructible<DebugDraw, const DebugDraw&>{});
    CORRADE_VERIFY(!std::is_assignable<DebugDraw, const DebugDraw&>{});
}

void DebugDrawTest::debugMode() {
    std::ostringstream out;

    Debug(&out) << DebugDraw::Mode::DrawAabb << DebugDraw::Mode(0xbaadcafe);
    CORRADE_COMPARE(out.str(), "BulletIntegration::DebugDraw::Mode::DrawAabb BulletIntegration::DebugDraw::Mode(0xbaadcafe)\n");
}

}}}}

CORRADE_TEST_MAIN(Magnum::BulletIntegration::Test::DebugDrawTest)
