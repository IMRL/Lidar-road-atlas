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

#include "Corrade/PluginManager/Manager.hpp"
#include "Corrade/PluginManager/PluginMetadata.h"
#include "Corrade/TestSuite/Tester.h"
#include "Corrade/TestSuite/Compare/Container.h"
#include "Corrade/TestSuite/Compare/Numeric.h"
#include "Corrade/Utility/ConfigurationGroup.h"
#include "Corrade/Utility/DebugStl.h"

#include "animals/Canary.h"

static void importPlugin() {
    CORRADE_PLUGIN_IMPORT(Canary)
}

namespace Corrade { namespace PluginManager { namespace Test { namespace {

struct AbstractPluginTest: TestSuite::Tester {
    explicit AbstractPluginTest();

    void constructCopy();
    void constructMove();

    #ifndef CORRADE_PLUGINMANAGER_NO_DYNAMIC_PLUGIN_SUPPORT
    void implicitPluginSearchPaths();
    void implicitPluginSearchPathsGlobalViews();
    void implicitPluginSearchPathsNoLibraryLocation();
    void implicitPluginSearchPathsNoAbsolutePath();
    #endif
};

using namespace Containers::Literals;

AbstractPluginTest::AbstractPluginTest() {
    addTests({&AbstractPluginTest::constructCopy,
              &AbstractPluginTest::constructMove,

              #ifndef CORRADE_PLUGINMANAGER_NO_DYNAMIC_PLUGIN_SUPPORT
              &AbstractPluginTest::implicitPluginSearchPaths,
              &AbstractPluginTest::implicitPluginSearchPathsGlobalViews,
              &AbstractPluginTest::implicitPluginSearchPathsNoLibraryLocation,
              &AbstractPluginTest::implicitPluginSearchPathsNoAbsolutePath
              #endif
              });

    importPlugin();
}

void AbstractPluginTest::constructCopy() {
    CORRADE_VERIFY(!std::is_copy_constructible<Canary>::value);
    CORRADE_VERIFY(!std::is_copy_assignable<Canary>::value);
}

void AbstractPluginTest::constructMove() {
    /* Only move construction is allowed */
    CORRADE_VERIFY(std::is_move_constructible<Canary>::value);
    CORRADE_VERIFY(!std::is_move_assignable<Canary>::value);

    PluginManager::Manager<AbstractAnimal> manager;

    /* Without plugin manager -- shouldn't crash or do any other weird stuff */
    {
        Canary a;
        CORRADE_VERIFY(!a.metadata());
        Canary b{std::move(a)};
        CORRADE_VERIFY(!b.metadata());
    }

    /* With plugin manager -- should properly reregister and not fail during
       destruction */
    CORRADE_COMPARE(manager.loadState("Canary"), LoadState::Static);
    {
        auto a = Containers::pointerCast<Canary>(manager.instantiate("Canary"));
        CORRADE_VERIFY(a);
        CORRADE_COMPARE(a->plugin(), "Canary");
        CORRADE_VERIFY(a->metadata());
        CORRADE_COMPARE(a->metadata()->name(), "Canary");
        CORRADE_COMPARE(a->configuration().value("name"), "Achoo");

        Canary b{std::move(*a)};
        CORRADE_COMPARE(b.plugin(), "Canary");
        CORRADE_VERIFY(b.metadata());
        CORRADE_COMPARE(b.metadata()->name(), "Canary");
        CORRADE_COMPARE(b.configuration().value("name"), "Achoo");
    }
}

#ifndef CORRADE_PLUGINMANAGER_NO_DYNAMIC_PLUGIN_SUPPORT
void AbstractPluginTest::implicitPluginSearchPaths() {
    Containers::StringView hardcodedPath = "/usr/lib/64/corrade/foobars";
    Containers::StringView relativePath = "corrade/foobars";
    /** @todo StringView once Compare::Container can compare different types */
    const Containers::String expected[]{
        hardcodedPath,
        #ifdef CORRADE_TARGET_APPLE
        "../PlugIns/corrade/foobars",
        #endif
        "/usr/lib/corrade/foobars",
        #ifndef CORRADE_TARGET_WINDOWS
        "../lib/corrade/foobars",
        #endif
        relativePath
    };
    Containers::Array<Containers::String> paths = PluginManager::implicitPluginSearchPaths(
        "/usr/lib/CorradeFooBar.so",
        hardcodedPath,
        relativePath
    );
    CORRADE_COMPARE_AS(paths,
        Containers::arrayView(expected),
        TestSuite::Compare::Container);
    CORRADE_COMPARE_AS(paths[0].data(),
        static_cast<const void*>(hardcodedPath.data()),
        TestSuite::Compare::NotEqual);
    CORRADE_COMPARE_AS(paths.back().data(),
        static_cast<const void*>(relativePath.data()),
        TestSuite::Compare::NotEqual);
}

void AbstractPluginTest::implicitPluginSearchPathsGlobalViews() {
    Containers::StringView hardcodedPath = "/usr/lib/64/corrade/foobars"_s;
    Containers::StringView relativePath = "corrade/foobars"_s;
    /** @todo StringView once Compare::Container can compare different types */
    const Containers::String expected[]{
        hardcodedPath,
        #ifdef CORRADE_TARGET_APPLE
        "../PlugIns/corrade/foobars",
        #endif
        "/usr/lib/corrade/foobars",
        #ifndef CORRADE_TARGET_WINDOWS
        "../lib/corrade/foobars",
        #endif
        relativePath
    };
    Containers::Array<Containers::String> paths = PluginManager::implicitPluginSearchPaths(
        "/usr/lib/CorradeFooBar.so",
        hardcodedPath,
        relativePath
    );
    CORRADE_COMPARE_AS(paths, Containers::arrayView(expected), TestSuite::Compare::Container);
    CORRADE_COMPARE(paths[0].data(), static_cast<const void*>(hardcodedPath.data()));
    CORRADE_COMPARE(paths.back().data(), static_cast<const void*>(relativePath.data()));
}

void AbstractPluginTest::implicitPluginSearchPathsNoLibraryLocation() {
    /** @todo StringView once Compare::Container can compare different types */
    const Containers::String expected[]{
        "/usr/lib/64/corrade/foobars",
        #ifdef CORRADE_TARGET_APPLE
        "../PlugIns/corrade/foobars",
        #endif
        #ifndef CORRADE_TARGET_WINDOWS
        "../lib/corrade/foobars",
        #endif
        "corrade/foobars"
    };
    CORRADE_COMPARE_AS(PluginManager::implicitPluginSearchPaths(
        {},
        "/usr/lib/64/corrade/foobars",
        "corrade/foobars"
    ), Containers::arrayView(expected), TestSuite::Compare::Container);
}

void AbstractPluginTest::implicitPluginSearchPathsNoAbsolutePath() {
    /** @todo StringView once Compare::Container can compare different types */
    const Containers::String expected[]{
        #ifdef CORRADE_TARGET_APPLE
        "../PlugIns/corrade/foobars",
        #endif
        "/usr/lib/corrade/foobars",
        #ifndef CORRADE_TARGET_WINDOWS
        "../lib/corrade/foobars",
        #endif
        "corrade/foobars"
    };
    CORRADE_COMPARE_AS(PluginManager::implicitPluginSearchPaths(
        "/usr/lib/CorradeFooBar.so",
        {},
        "corrade/foobars"
    ), Containers::arrayView(expected), TestSuite::Compare::Container);
}
#endif

}}}}

CORRADE_TEST_MAIN(Corrade::PluginManager::Test::AbstractPluginTest)
