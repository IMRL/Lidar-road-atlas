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

#include <sstream>
#include <Corrade/Containers/Array.h>
#include <Corrade/Containers/Optional.h>
#include <Corrade/Containers/StringStl.h> /** @todo remove once AbstractFont is <string>-free */
#include <Corrade/TestSuite/Tester.h>
#include <Corrade/TestSuite/Compare/String.h>
#include <Corrade/Utility/DebugStl.h>
#include <Corrade/Utility/Path.h>

#include "Magnum/FileCallback.h"
#include "Magnum/Text/AbstractFont.h"
#include "Magnum/Text/AbstractGlyphCache.h"
#include "Magnum/Trade/AbstractImporter.h"

#include "configure.h"

namespace Magnum { namespace Text { namespace Test { namespace {

struct MagnumFontTest: TestSuite::Tester {
    explicit MagnumFontTest();

    void nonexistent();
    void properties();
    void layout();

    void fileCallbackImage();
    void fileCallbackImageNotFound();

    /* Explicitly forbid system-wide plugin dependencies */
    PluginManager::Manager<Trade::AbstractImporter> _importerManager{"nonexistent"};
    PluginManager::Manager<AbstractFont> _fontManager{"nonexistent"};
};

MagnumFontTest::MagnumFontTest() {
    addTests({&MagnumFontTest::nonexistent,
              &MagnumFontTest::properties,
              &MagnumFontTest::layout,

              &MagnumFontTest::fileCallbackImage,
              &MagnumFontTest::fileCallbackImageNotFound});

    /* Load the plugins directly from the build tree. Otherwise they're static
       and already loaded. */
    _fontManager.registerExternalManager(_importerManager);
    #if defined(TGAIMPORTER_PLUGIN_FILENAME) && defined(MAGNUMFONT_PLUGIN_FILENAME)
    CORRADE_INTERNAL_ASSERT_OUTPUT(_importerManager.load(TGAIMPORTER_PLUGIN_FILENAME) & PluginManager::LoadState::Loaded);
    CORRADE_INTERNAL_ASSERT_OUTPUT(_fontManager.load(MAGNUMFONT_PLUGIN_FILENAME) & PluginManager::LoadState::Loaded);
    #endif
}

void MagnumFontTest::nonexistent() {
    Containers::Pointer<AbstractFont> font = _fontManager.instantiate("MagnumFont");

    std::ostringstream out;
    Error redirectError{&out};
    CORRADE_VERIFY(!font->openFile("nonexistent.conf", 0.0f));
    /* There's an error message from Path::read() before */
    CORRADE_COMPARE_AS(out.str(),
        "\nText::AbstractFont::openFile(): cannot open file nonexistent.conf\n",
        TestSuite::Compare::StringHasSuffix);
}

void MagnumFontTest::properties() {
    Containers::Pointer<AbstractFont> font = _fontManager.instantiate("MagnumFont");

    CORRADE_VERIFY(font->openFile(Utility::Path::join(MAGNUMFONT_TEST_DIR, "font.conf"), 0.0f));
    CORRADE_COMPARE(font->size(), 16.0f);
    CORRADE_COMPARE(font->ascent(), 25.0f);
    CORRADE_COMPARE(font->descent(), -10.0f);
    CORRADE_COMPARE(font->lineHeight(), 39.7333f);
    CORRADE_COMPARE(font->glyphAdvance(font->glyphId(U'W')), Vector2(23.0f, 0.0f));
}

void MagnumFontTest::layout() {
    Containers::Pointer<AbstractFont> font = _fontManager.instantiate("MagnumFont");

    CORRADE_VERIFY(font->openFile(Utility::Path::join(MAGNUMFONT_TEST_DIR, "font.conf"), 0.0f));

    /* Fill the cache with some fake glyphs */
    struct DummyGlyphCache: AbstractGlyphCache {
        using AbstractGlyphCache::AbstractGlyphCache;

        GlyphCacheFeatures doFeatures() const override { return {}; }
        void doSetImage(const Vector2i&, const ImageView2D&) override {}
    } cache{Vector2i{256}};
    cache.insert(font->glyphId(U'W'), {25, 34}, {{0, 8}, {16, 128}});
    cache.insert(font->glyphId(U'e'), {25, 12}, {{16, 4}, {64, 32}});

    auto layouter = font->layout(cache, 0.5f, "Wave");
    CORRADE_VERIFY(layouter);
    CORRADE_COMPARE(layouter->glyphCount(), 4);

    Range2D rectangle;
    Range2D position;
    Range2D textureCoordinates;

    /* 'W' */
    Vector2 cursorPosition;
    std::tie(position, textureCoordinates) = layouter->renderGlyph(0, cursorPosition = {}, rectangle);
    CORRADE_COMPARE(position, Range2D({0.78125f, 1.0625f}, {1.28125f, 4.8125f}));
    CORRADE_COMPARE(textureCoordinates, Range2D({0, 0.03125f}, {0.0625f, 0.5f}));
    CORRADE_COMPARE(cursorPosition, Vector2(0.71875f, 0.0f));

    /* 'a' (not found) */
    std::tie(position, textureCoordinates) = layouter->renderGlyph(1, cursorPosition = {}, rectangle);
    CORRADE_COMPARE(position, Range2D());
    CORRADE_COMPARE(textureCoordinates, Range2D());
    CORRADE_COMPARE(cursorPosition, Vector2(0.25f, 0.0f));

    /* 'v' (not found) */
    std::tie(position, textureCoordinates) = layouter->renderGlyph(2, cursorPosition = {}, rectangle);
    CORRADE_COMPARE(position, Range2D());
    CORRADE_COMPARE(textureCoordinates, Range2D());
    CORRADE_COMPARE(cursorPosition, Vector2(0.25f, 0.0f));

    /* 'e' */
    std::tie(position, textureCoordinates) = layouter->renderGlyph(3, cursorPosition = {}, rectangle);
    CORRADE_COMPARE(position, Range2D({0.78125f, 0.375f}, {2.28125f, 1.25f}));
    CORRADE_COMPARE(textureCoordinates, Range2D({0.0625f, 0.015625f}, {0.25f, 0.125f}));
    CORRADE_COMPARE(cursorPosition, Vector2(0.375f, 0.0f));
}

void MagnumFontTest::fileCallbackImage() {
    Containers::Pointer<AbstractFont> font = _fontManager.instantiate("MagnumFont");
    CORRADE_VERIFY(font->features() & FontFeature::FileCallback);

    std::unordered_map<std::string, Containers::Array<char>> files;
    Containers::Optional<Containers::Array<char>> conf = Utility::Path::read(Utility::Path::join(MAGNUMFONT_TEST_DIR, "font.conf"));
    Containers::Optional<Containers::Array<char>> tga =
    Utility::Path::read(Utility::Path::join(MAGNUMFONT_TEST_DIR, "font.tga"));
    CORRADE_VERIFY(conf);
    CORRADE_VERIFY(tga);
    files["not/a/path/font.conf"] = *std::move(conf);
    files["not/a/path/font.tga"] = *std::move(tga);
    font->setFileCallback([](const std::string& filename, InputFileCallbackPolicy policy,
        std::unordered_map<std::string, Containers::Array<char>>& files) {
            Debug{} << "Loading" << filename << "with" << policy;
            return Containers::optional(Containers::ArrayView<const char>(files.at(filename)));
        }, files);

    CORRADE_VERIFY(font->openFile("not/a/path/font.conf", 13.0f));
    CORRADE_COMPARE(font->size(), 16.0f);
    CORRADE_COMPARE(font->ascent(), 25.0f);
    CORRADE_COMPARE(font->descent(), -10.0f);
    CORRADE_COMPARE(font->lineHeight(), 39.7333f);
    CORRADE_COMPARE(font->glyphAdvance(font->glyphId(U'W')), Vector2(23.0f, 0.0f));
}

void MagnumFontTest::fileCallbackImageNotFound() {
    Containers::Pointer<AbstractFont> font = _fontManager.instantiate("MagnumFont");
    CORRADE_VERIFY(font->features() & FontFeature::FileCallback);

    font->setFileCallback([](const std::string&, InputFileCallbackPolicy, void*) {
            return Containers::Optional<Containers::ArrayView<const char>>{};
        });

    std::ostringstream out;
    Error redirectError{&out};
    Containers::Optional<Containers::Array<char>> conf = Utility::Path::read(Utility::Path::join(MAGNUMFONT_TEST_DIR, "font.conf"));
    CORRADE_VERIFY(conf);
    CORRADE_VERIFY(!font->openData(*conf, 13.0f));
    CORRADE_COMPARE(out.str(), "Trade::AbstractImporter::openFile(): cannot open file font.tga\n");
}

}}}}

CORRADE_TEST_MAIN(Magnum::Text::Test::MagnumFontTest)
