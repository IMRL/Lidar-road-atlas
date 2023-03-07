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

#include "AbstractImporter.h"

#include <string>
#include <Corrade/Containers/Array.h>
#include <Corrade/Containers/EnumSet.hpp>
#include <Corrade/Containers/Optional.h>
#include <Corrade/Containers/String.h>
#include <Corrade/Containers/StringStl.h> /** @todo remove once AbstractImporter is <string>-free */
#include <Corrade/PluginManager/Manager.hpp>
#include <Corrade/Utility/Assert.h>
#include <Corrade/Utility/DebugStl.h> /** @todo remove once AbstractImporter is <string>-free */
#include <Corrade/Utility/Path.h>

#ifndef CORRADE_PLUGINMANAGER_NO_DYNAMIC_PLUGIN_SUPPORT
#include "Magnum/Audio/configure.h"
#endif

namespace Corrade { namespace PluginManager {

/* On non-MinGW Windows the instantiations are already marked with extern
   template. However Clang-CL doesn't propagate the export from the extern
   template, it seems. */
#if !defined(CORRADE_TARGET_WINDOWS) || defined(CORRADE_TARGET_MINGW) || defined(CORRADE_TARGET_CLANG_CL)
#define MAGNUM_AUDIO_EXPORT_HPP MAGNUM_AUDIO_EXPORT
#else
#define MAGNUM_AUDIO_EXPORT_HPP
#endif
template class MAGNUM_AUDIO_EXPORT_HPP Manager<Magnum::Audio::AbstractImporter>;

}}

namespace Magnum { namespace Audio {

using namespace Containers::Literals;

Containers::StringView AbstractImporter::pluginInterface() {
    return
/* [interface] */
"cz.mosra.magnum.Audio.AbstractImporter/0.1"_s
/* [interface] */
    ;
}

#ifndef CORRADE_PLUGINMANAGER_NO_DYNAMIC_PLUGIN_SUPPORT
Containers::Array<Containers::String> AbstractImporter::pluginSearchPaths() {
    const Containers::Optional<Containers::String> libraryLocation = Utility::Path::libraryLocation(&pluginInterface);
    return PluginManager::implicitPluginSearchPaths(
        #ifndef MAGNUM_BUILD_STATIC
        libraryLocation ? *libraryLocation : Containers::String{},
        #else
        {},
        #endif
        #ifdef CORRADE_IS_DEBUG_BUILD
        MAGNUM_PLUGINS_AUDIOIMPORTER_DEBUG_DIR,
        #else
        MAGNUM_PLUGINS_AUDIOIMPORTER_DIR,
        #endif
        #ifdef CORRADE_IS_DEBUG_BUILD
        "magnum-d/"
        #else
        "magnum/"
        #endif
        "audioimporters"_s);
}
#endif

AbstractImporter::AbstractImporter() = default;

AbstractImporter::AbstractImporter(PluginManager::Manager<AbstractImporter>& manager): PluginManager::AbstractManagingPlugin<AbstractImporter>{manager} {}

AbstractImporter::AbstractImporter(PluginManager::AbstractManager& manager, const Containers::StringView& plugin): PluginManager::AbstractManagingPlugin<AbstractImporter>{manager, plugin} {}

bool AbstractImporter::openData(Containers::ArrayView<const void> data) {
    CORRADE_ASSERT(features() & ImporterFeature::OpenData,
        "Audio::AbstractImporter::openData(): feature not supported", {});

    close();
    doOpenData(Containers::arrayCast<const char>(data));
    return isOpened();
}

void AbstractImporter::doOpenData(Containers::ArrayView<const char>) {
    CORRADE_ASSERT_UNREACHABLE("Audio::AbstractImporter::openData(): feature advertised but not implemented", );
}

bool AbstractImporter::openFile(const std::string& filename) {
    close();
    doOpenFile(filename);
    return isOpened();
}

void AbstractImporter::doOpenFile(const std::string& filename) {
    CORRADE_ASSERT(features() & ImporterFeature::OpenData, "Audio::AbstractImporter::openFile(): not implemented", );

    /* Open file */
    const Containers::Optional<Containers::Array<char>> data = Utility::Path::read(filename);
    if(!data) {
        Error() << "Audio::AbstractImporter::openFile(): cannot open file" << filename;
        return;
    }

    doOpenData(*data);
}

void AbstractImporter::close() {
    if(isOpened()) {
        doClose();
        CORRADE_INTERNAL_ASSERT(!isOpened());
    }
}

BufferFormat AbstractImporter::format() const {
    CORRADE_ASSERT(isOpened(), "Audio::AbstractImporter::format(): no file opened", {});
    return doFormat();
}

UnsignedInt AbstractImporter::frequency() const {
    CORRADE_ASSERT(isOpened(), "Audio::AbstractImporter::frequency(): no file opened", {});
    return doFrequency();
}

Containers::Array<char> AbstractImporter::data() {
    CORRADE_ASSERT(isOpened(), "Audio::AbstractImporter::data(): no file opened", nullptr);

    Containers::Array<char> out = doData();
    CORRADE_ASSERT(!out.deleter(), "Audio::AbstractImporter::data(): implementation is not allowed to use a custom Array deleter", {});
    return out;
}

Debug& operator<<(Debug& debug, const ImporterFeature value) {
    debug << "Audio::ImporterFeature" << Debug::nospace;

    switch(value) {
        /* LCOV_EXCL_START */
        #define _c(v) case ImporterFeature::v: return debug << "::" #v;
        _c(OpenData)
        #undef _c
        /* LCOV_EXCL_STOP */
    }

    return debug << "(" << Debug::nospace << reinterpret_cast<void*>(UnsignedByte(value)) << Debug::nospace << ")";
}

Debug& operator<<(Debug& debug, const ImporterFeatures value) {
    return Containers::enumSetDebugOutput(debug, value, "Audio::ImporterFeatures{}", {
        ImporterFeature::OpenData});
}

}}
