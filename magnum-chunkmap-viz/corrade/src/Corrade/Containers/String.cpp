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

#include "String.h"
#include "StringStl.h"

#include <string>
#include <cstring>

#include "Corrade/Containers/Array.h"
#include "Corrade/Containers/Pair.h"
#include "Corrade/Containers/StaticArray.h"

namespace Corrade { namespace Containers {

namespace {
    enum: std::size_t {
        SmallSize = 0x80,
        SmallSizeMask = 0xc0,
        LargeSizeMask = SmallSizeMask << (sizeof(std::size_t) - 1)*8
    };
}

static_assert(std::size_t(LargeSizeMask) == Implementation::StringViewSizeMask,
    "reserved bits should be the same in String and StringView");

String String::nullTerminatedView(StringView view) {
    if(view.flags() & StringViewFlag::NullTerminated)
        return String{view.data(), view.size(), [](char*, std::size_t){}};
    return String{view};
}

String String::nullTerminatedView(AllocatedInitT, StringView view) {
    if(view.flags() & StringViewFlag::NullTerminated)
        return String{view.data(), view.size(), [](char*, std::size_t){}};
    return String{AllocatedInit, view};
}

String String::nullTerminatedGlobalView(StringView view) {
    if(view.flags() >= (StringViewFlag::NullTerminated|StringViewFlag::Global))
        return String{view.data(), view.size(), [](char*, std::size_t){}};
    return String{view};
}

String String::nullTerminatedGlobalView(AllocatedInitT, StringView view) {
    if(view.flags() >= (StringViewFlag::NullTerminated|StringViewFlag::Global))
        return String{view.data(), view.size(), [](char*, std::size_t){}};
    return String{AllocatedInit, view};
}

inline void String::construct(Corrade::NoInitT, const std::size_t size) {
    if(size < Implementation::SmallStringSize) {
        _small.data[size] = '\0';
        _small.size = size | SmallSize;
    } else {
        _large.data = new char[size + 1];
        _large.data[size] = '\0';
        _large.size = size;
        _large.deleter = nullptr;
    }
}

inline void String::construct(const char* const data, const std::size_t size) {
    construct(Corrade::NoInit, size);

    /* If the size is small enough for SSO, use that. Not using <= because we
       need to store the null terminator as well. */
    if(size < Implementation::SmallStringSize) {
        /* Apparently memcpy() can't be called with null pointers, even if size
           is zero. I call that bullying. */
        if(size) std::memcpy(_small.data, data, size);

    /* Otherwise allocate. Assuming the size is small enough -- this should
       have been checked in the caller already. */
    } else {
        std::memcpy(_large.data, data, size);
    }
}

inline void String::destruct() {
    /* If not SSO, delete the data */
    if(_small.size & 0x80) return;
    if(_large.deleter) _large.deleter(_large.data, _large.size);
    else delete[] _large.data;
}

inline Containers::Pair<const char*, std::size_t> String::dataInternal() const {
    if(_small.size & 0x80)
        return {_small.data, _small.size & ~SmallSizeMask};
    return {_large.data, _large.size & ~LargeSizeMask};
}

String::String() noexcept {
    /* Create a zero-size small string to fullfil the guarantee of data() being
       always non-null and null-terminated */
    _small.data[0] = '\0';
    _small.size = SmallSize;
}

String::String(const StringView view): String{view._data, view._sizePlusFlags & ~Implementation::StringViewSizeMask} {}

String::String(const MutableStringView view): String{view._data, view._sizePlusFlags & ~Implementation::StringViewSizeMask} {}

String::String(const ArrayView<const char> view): String{view.data(), view.size()} {}

String::String(const ArrayView<char> view): String{view.data(), view.size()} {}

String::String(const char* const data): String{data, data ? std::strlen(data) : 0} {}

String::String(const char* const data, const std::size_t size)
    #ifdef CORRADE_GRACEFUL_ASSERT
    /* Zero-init the contents so the destructor doesn't crash if we assert here */
    : _large{}
    #endif
{
    CORRADE_ASSERT(size < std::size_t{1} << (sizeof(std::size_t)*8 - 2),
        "Containers::String: string expected to be smaller than 2^" << Utility::Debug::nospace << sizeof(std::size_t)*8 - 2 << "bytes, got" << size, );
    CORRADE_ASSERT(data || !size,
        "Containers::String: received a null string of size" << size, );

    construct(data, size);
}

String::String(AllocatedInitT, const StringView view): String{AllocatedInit, view._data, view._sizePlusFlags & ~Implementation::StringViewSizeMask} {}

String::String(AllocatedInitT, const MutableStringView view): String{AllocatedInit, view._data, view._sizePlusFlags & ~Implementation::StringViewSizeMask} {}

String::String(AllocatedInitT, const ArrayView<const char> view): String{AllocatedInit, view.data(), view.size()} {}

String::String(AllocatedInitT, const ArrayView<char> view): String{AllocatedInit, view.data(), view.size()} {}

String::String(AllocatedInitT, const char* const data): String{AllocatedInit, data, data ? std::strlen(data) : 0} {}

String::String(AllocatedInitT, const char* const data, const std::size_t size)
    #ifdef CORRADE_GRACEFUL_ASSERT
    /* Zero-init the contents so the destructor doesn't crash if we assert here */
    : _large{}
    #endif
{
    CORRADE_ASSERT(size < std::size_t{1} << (sizeof(std::size_t)*8 - 2),
        "Containers::String: string expected to be smaller than 2^" << Utility::Debug::nospace << sizeof(std::size_t)*8 - 2 << "bytes, got" << size, );
    CORRADE_ASSERT(data || !size,
        "Containers::String: received a null string of size" << size, );

    _large.data = new char[size + 1];
    /* Apparently memcpy() can't be called with null pointers, even if size is
       zero. I call that bullying. */
    if(size) std::memcpy(_large.data, data, size);
    _large.data[size] = '\0';
    _large.size = size;
    _large.deleter = nullptr;
}

String::String(AllocatedInitT, String&& other) {
    /* Allocate a copy if the other is a SSO */
    if(other.isSmall()) {
        const std::size_t sizePlusOne = (other._small.size & ~SmallSizeMask) + 1;
        _large.data = new char[sizePlusOne];
        /* Copies also the null terminator */
        std::memcpy(_large.data, other._small.data, sizePlusOne);
        _large.size = other._small.size & ~SmallSizeMask;
        _large.deleter = nullptr;

    /* Otherwise take over the data */
    } else {
        _large.data = other._large.data;
        _large.size = other._large.size;
        _large.deleter = other._large.deleter;
    }

    /* Move-out the other instance in both cases */
    other._large.data = nullptr;
    other._large.size = 0;
    other._large.deleter = nullptr;
}

String::String(AllocatedInitT, const String& other) {
    const Containers::Pair<const char*, std::size_t> data = other.dataInternal();
    const std::size_t sizePlusOne = data.second() + 1;
    _large.size = data.second();
    _large.data = new char[sizePlusOne];
    /* Copies also the null terminator */
    std::memcpy(_large.data, data.first(), sizePlusOne);
    _large.deleter = nullptr;
}

String::String(char* const data, const std::size_t size, void(*deleter)(char*, std::size_t)) noexcept
    #ifdef CORRADE_GRACEFUL_ASSERT
    /* Zero-init the contents so the destructor doesn't crash if we assert here */
    : _large{}
    #endif
{
    CORRADE_ASSERT(size < std::size_t{1} << (sizeof(std::size_t)*8 - 2),
        "Containers::String: string expected to be smaller than 2^" << Utility::Debug::nospace << sizeof(std::size_t)*8 - 2 << "bytes, got" << size, );
    CORRADE_ASSERT(data && !data[size],
        "Containers::String: can only take ownership of a non-null null-terminated array", );

    _large.data = data;
    _large.size = size;
    _large.deleter = deleter;
}

String::String(void(*deleter)(char*, std::size_t), std::nullptr_t, char* const data) noexcept: String{
    data,
    /* If data is null, strlen() would crash before reaching our assert inside
       the delegated-to constructor */
    #ifndef CORRADE_NO_ASSERT
    data ? std::strlen(data) : 0,
    #else
    std::strlen(data),
    #endif
    deleter
} {}

String::String(Corrade::ValueInitT, const std::size_t size): _large{} {
    CORRADE_ASSERT(size < std::size_t{1} << (sizeof(std::size_t)*8 - 2),
        "Containers::String: string expected to be smaller than 2^" << Utility::Debug::nospace << sizeof(std::size_t)*8 - 2 << "bytes, got" << size, );

    if(size < Implementation::SmallStringSize) {
        /* Everything already zero-init'd in the constructor init list */
        _small.size = size | SmallSize;
    } else {
        _large.data = new char[size + 1]{};
        _large.size = size;
        _large.deleter = nullptr;
    }
}

String::String(Corrade::DirectInitT, const std::size_t size, const char c): String{Corrade::NoInit, size} {
    #ifdef CORRADE_GRACEFUL_ASSERT
    /* If the NoInit constructor asserted, don't attempt to memset */
    if(size >= Implementation::SmallStringSize && !_large.data) return;
    #endif

    std::memset(size < Implementation::SmallStringSize ? _small.data : _large.data, c, size);
}

String::String(Corrade::NoInitT, const std::size_t size)
    #ifdef CORRADE_GRACEFUL_ASSERT
    /* Zero-init the contents so the destructor doesn't crash if we assert here */
    : _large{}
    #endif
{
    CORRADE_ASSERT(size < std::size_t{1} << (sizeof(std::size_t)*8 - 2),
        "Containers::String: string expected to be smaller than 2^" << Utility::Debug::nospace << sizeof(std::size_t)*8 - 2 << "bytes, got" << size, );

    construct(Corrade::NoInit, size);
}

String::~String() { destruct(); }

String::String(const String& other) {
    const Containers::Pair<const char*, std::size_t> data = other.dataInternal();
    construct(data.first(), data.second());
}

String::String(String&& other) noexcept {
    /* Similarly as in operator=(String&&), the following works also in case of
       SSO, as for small string we would be doing a copy of _small.data and
       then also a copy of _small.size *including* the two highest bits */
    _large.data = other._large.data;
    _large.size = other._large.size;
    _large.deleter = other._large.deleter;
    other._large.data = nullptr;
    other._large.size = 0;
    other._large.deleter = nullptr;
}

String& String::operator=(const String& other) {
    destruct();

    const Containers::Pair<const char*, std::size_t> data = other.dataInternal();
    construct(data.first(), data.second());
    return *this;
}

String& String::operator=(String&& other) noexcept {
    /* Simply swap the contents, which will do the right thing always:

       - If both are allocated, swapping just swaps the pointers and sizes,
         and each instance will later correctly delete its own.
       - If the other is allocated and ours is small, the other gets our small
         string and we get the pointer and deleter in exchange. We'll delete
         the newly acquired pointer and the other will do nothing as it has our
         small data.
       - If we're allocated and the other is small, it's just the inverse of
         the above.
       - If both are small, there's just data exchange, with neither instance
         deleting anything. */
    using std::swap;
    swap(other._large.data, _large.data);
    swap(other._large.size, _large.size);
    swap(other._large.deleter, _large.deleter);
    return *this;
}

String::operator ArrayView<const char>() const noexcept {
    const Containers::Pair<const char*, std::size_t> data = dataInternal();
    return {data.first(), data.second()};
}

String::operator ArrayView<const void>() const noexcept {
    const Containers::Pair<const char*, std::size_t> data = dataInternal();
    return {data.first(), data.second()};
}

String::operator ArrayView<char>() noexcept {
    const Containers::Pair<const char*, std::size_t> data = dataInternal();
    return {const_cast<char*>(data.first()), data.second()};
}

String::operator ArrayView<void>() noexcept {
    const Containers::Pair<const char*, std::size_t> data = dataInternal();
    return {const_cast<char*>(data.first()), data.second()};
}

String::operator Array<char>() && {
    Array<char> out;
    if(_small.size & 0x80) {
        const std::size_t size = _small.size & ~SmallSizeMask;
        /* Allocate the output including a null terminator at the end, but
           don't include it in the size */
        out = Array<char>{Array<char>{Corrade::NoInit, size + 1}.release(), size};
        out[size] = '\0';
        std::memcpy(out.data(), _small.data, size);
    } else {
        out = Array<char>{_large.data, _large.size, deleter()};
    }

    /* Same as in release(). Create a zero-size small string to fullfil the
       guarantee of data() being always non-null and null-terminated. Since
       this makes the string switch to SSO, we also clear the deleter this
       way. */
    _small.data[0] = '\0';
    _small.size = SmallSize;

    return out;
}

String::operator bool() const {
    /* The data pointer is guaranteed to be non-null, so no need to check it */
    if(_small.size & 0x80) return _small.size & ~SmallSizeMask;
    return _large.size;
}

const char* String::data() const {
    if(_small.size & 0x80) return _small.data;
    return _large.data;
}

char* String::data() {
    if(_small.size & 0x80) return _small.data;
    return _large.data;
}

bool String::isEmpty() const {
    if(_small.size & 0x80) return !(_small.size & ~SmallSizeMask);
    return !_large.size;
}

auto String::deleter() const -> Deleter {
    CORRADE_ASSERT(!(_small.size & 0x80),
        "Containers::String::deleter(): cannot call on a SSO instance", {});
    return _large.deleter;
}

std::size_t String::size() const {
    if(_small.size & 0x80) return _small.size & ~SmallSizeMask;
    return _large.size;
}

char* String::begin() {
    if(_small.size & 0x80) return _small.data;
    return _large.data;
}

const char* String::begin() const {
    if(_small.size & 0x80) return _small.data;
    return _large.data;
}

const char* String::cbegin() const {
    if(_small.size & 0x80) return _small.data;
    return _large.data;
}

char* String::end() {
    if(_small.size & 0x80) return _small.data + (_small.size & ~SmallSizeMask);
    return _large.data + _large.size;
}

const char* String::end() const {
    if(_small.size & 0x80) return _small.data + (_small.size & ~SmallSizeMask);
    return _large.data + _large.size;
}

const char* String::cend() const {
    if(_small.size & 0x80) return _small.data + (_small.size & ~SmallSizeMask);
    return _large.data + _large.size;
}

/** @todo does it make a practical sense (debug perf) to rewrite these two
    directly without delegating to size()/begin()/end()? i don't think so */

char& String::front() {
    CORRADE_ASSERT(size(), "Containers::String::front(): string is empty", *begin());
    return *begin();
}

char String::front() const {
    return const_cast<String&>(*this).front();
}

char& String::back() {
    CORRADE_ASSERT(size(), "Containers::String::back(): string is empty", *(end() - 1));
    return *(end() - 1);
}

char String::back() const {
    return const_cast<String&>(*this).back();
}

char& String::operator[](std::size_t i) {
    if(_small.size & 0x80) return _small.data[i];
    return _large.data[i];
}

char String::operator[](std::size_t i) const {
    if(_small.size & 0x80) return _small.data[i];
    return _large.data[i];
}

MutableStringView String::slice(char* const begin, char* const end) {
    return MutableStringView{*this}.slice(begin, end);
}

StringView String::slice(const char* const begin, const char* const end) const {
    return StringView{*this}.slice(begin, end);
}

MutableStringView String::slice(const std::size_t begin, const std::size_t end) {
    return MutableStringView{*this}.slice(begin, end);
}

StringView String::slice(const std::size_t begin, const std::size_t end) const {
    return StringView{*this}.slice(begin, end);
}

MutableStringView String::prefix(char* const end) {
    return MutableStringView{*this}.prefix(end);
}

StringView String::prefix(const char* const end) const {
    return StringView{*this}.prefix(end);
}

MutableStringView String::suffix(char* const begin) {
    return MutableStringView{*this}.suffix(begin);
}

StringView String::suffix(const char* const begin) const {
    return StringView{*this}.suffix(begin);
}

MutableStringView String::prefix(const std::size_t count) {
    return MutableStringView{*this}.prefix(count);
}

StringView String::prefix(const std::size_t count) const {
    return StringView{*this}.prefix(count);
}

MutableStringView String::exceptPrefix(const std::size_t begin) {
    return MutableStringView{*this}.exceptPrefix(begin);
}

StringView String::exceptPrefix(const std::size_t begin) const {
    return StringView{*this}.exceptPrefix(begin);
}

#ifdef CORRADE_BUILD_DEPRECATED
MutableStringView String::suffix(const std::size_t begin) {
    return MutableStringView{*this}.exceptPrefix(begin);
}

StringView String::suffix(const std::size_t begin) const {
    return StringView{*this}.exceptPrefix(begin);
}
#endif

MutableStringView String::exceptSuffix(const std::size_t count) {
    return MutableStringView{*this}.exceptSuffix(count);
}

StringView String::exceptSuffix(const std::size_t count) const {
    return StringView{*this}.exceptSuffix(count);
}

#ifdef CORRADE_BUILD_DEPRECATED
MutableStringView String::except(const std::size_t count) {
    return MutableStringView{*this}.exceptSuffix(count);
}

StringView String::except(const std::size_t count) const {
    return StringView{*this}.exceptSuffix(count);
}
#endif

Array<MutableStringView> String::split(const char delimiter) {
    return MutableStringView{*this}.split(delimiter);
}

Array<StringView> String::split(const char delimiter) const {
    return StringView{*this}.split(delimiter);
}

Array<MutableStringView> String::splitWithoutEmptyParts(const char delimiter) {
    return MutableStringView{*this}.splitWithoutEmptyParts(delimiter);
}

Array<StringView> String::splitWithoutEmptyParts(const char delimiter) const {
    return StringView{*this}.splitWithoutEmptyParts(delimiter);
}

Array<MutableStringView> String::splitOnAnyWithoutEmptyParts(const StringView delimiters) {
    return MutableStringView{*this}.splitOnAnyWithoutEmptyParts(delimiters);
}

Array<StringView> String::splitOnAnyWithoutEmptyParts(const StringView delimiters) const {
    return StringView{*this}.splitOnAnyWithoutEmptyParts(delimiters);
}

#ifdef CORRADE_BUILD_DEPRECATED
Array<MutableStringView> String::splitWithoutEmptyParts(const StringView delimiters) {
    return splitOnAnyWithoutEmptyParts(delimiters);
}

Array<StringView> String::splitWithoutEmptyParts(const StringView delimiters) const {
    return splitOnAnyWithoutEmptyParts(delimiters);
}
#endif

Array<MutableStringView> String::splitOnWhitespaceWithoutEmptyParts() {
    return MutableStringView{*this}.splitOnWhitespaceWithoutEmptyParts();
}

Array<StringView> String::splitOnWhitespaceWithoutEmptyParts() const {
    return StringView{*this}.splitOnWhitespaceWithoutEmptyParts();
}

#ifdef CORRADE_BUILD_DEPRECATED
Array<MutableStringView> String::splitWithoutEmptyParts() {
    return splitOnWhitespaceWithoutEmptyParts();
}

Array<StringView> String::splitWithoutEmptyParts() const {
    return splitOnWhitespaceWithoutEmptyParts();
}
#endif

Array3<MutableStringView> String::partition(const char separator) {
    return MutableStringView{*this}.partition(separator);
}

Array3<StringView> String::partition(const char separator) const {
    return StringView{*this}.partition(separator);
}

String String::join(const ArrayView<const StringView> strings) const {
    return StringView{*this}.join(strings);
}

String String::join(const std::initializer_list<StringView> strings) const {
    /* Doing it this way instead of calling directly into StringView to have
       the above overload implicitly covered */
    return join(arrayView(strings));
}

String String::joinWithoutEmptyParts(const ArrayView<const StringView> strings) const {
    return StringView{*this}.joinWithoutEmptyParts(strings);
}

String String::joinWithoutEmptyParts(const std::initializer_list<StringView> strings) const {
    /* Doing it this way instead of calling directly into StringView to have
       the above overload implicitly covered */
    return joinWithoutEmptyParts(arrayView(strings));
}

bool String::hasPrefix(const StringView prefix) const {
    return StringView{*this}.hasPrefix(prefix);
}

bool String::hasPrefix(const char prefix) const {
    return StringView{*this}.hasPrefix(prefix);
}

bool String::hasSuffix(const StringView suffix) const {
    return StringView{*this}.hasSuffix(suffix);
}

bool String::hasSuffix(const char suffix) const {
    return StringView{*this}.hasSuffix(suffix);
}

MutableStringView String::exceptPrefix(const StringView prefix) {
    return MutableStringView{*this}.exceptPrefix(prefix);
}

StringView String::exceptPrefix(const StringView prefix) const {
    return StringView{*this}.exceptPrefix(prefix);
}

MutableStringView String::exceptSuffix(const StringView suffix) {
    return MutableStringView{*this}.exceptSuffix(suffix);
}

StringView String::exceptSuffix(const StringView suffix) const {
    return StringView{*this}.exceptSuffix(suffix);
}

MutableStringView String::trimmed(const StringView characters) {
    return MutableStringView{*this}.trimmed(characters);
}

StringView String::trimmed(const StringView characters) const {
    return StringView{*this}.trimmed(characters);
}

MutableStringView String::trimmed() {
    return MutableStringView{*this}.trimmed();
}

StringView String::trimmed() const {
    return StringView{*this}.trimmed();
}

MutableStringView String::trimmedPrefix(const StringView characters) {
    return MutableStringView{*this}.trimmedPrefix(characters);
}

StringView String::trimmedPrefix(const StringView characters) const {
    return StringView{*this}.trimmedPrefix(characters);
}

MutableStringView String::trimmedPrefix() {
    return MutableStringView{*this}.trimmedPrefix();
}

StringView String::trimmedPrefix() const {
    return StringView{*this}.trimmedPrefix();
}

MutableStringView String::trimmedSuffix(const StringView characters) {
    return MutableStringView{*this}.trimmedSuffix(characters);
}

StringView String::trimmedSuffix(const StringView characters) const {
    return StringView{*this}.trimmedSuffix(characters);
}

MutableStringView String::trimmedSuffix() {
    return MutableStringView{*this}.trimmedSuffix();
}

StringView String::trimmedSuffix() const {
    return StringView{*this}.trimmedSuffix();
}

MutableStringView String::find(const StringView substring) {
    /* Calling straight into the concrete implementation to reduce call stack
       depth */
    return MutableStringView{*this}.findOr(substring, nullptr);
}

StringView String::find(const StringView substring) const {
    /* Calling straight into the concrete implementation to reduce call stack
       depth */
    return StringView{*this}.findOr(substring, nullptr);
}

MutableStringView String::find(const char character) {
    /* Calling straight into the concrete implementation to reduce call stack
       depth */
    return MutableStringView{*this}.findOr(character, nullptr);
}

StringView String::find(const char character) const {
    /* Calling straight into the concrete implementation to reduce call stack
       depth */
    return StringView{*this}.findOr(character, nullptr);
}

MutableStringView String::findOr(const StringView substring, char* const fail) {
    return MutableStringView{*this}.findOr(substring, fail);
}

StringView String::findOr(const StringView substring, const char* const fail) const {
    return StringView{*this}.findOr(substring, fail);
}

MutableStringView String::findOr(const char character, char* const fail) {
    return MutableStringView{*this}.findOr(character, fail);
}

StringView String::findOr(const char character, const char* const fail) const {
    return StringView{*this}.findOr(character, fail);
}

MutableStringView String::findLast(const StringView substring) {
    /* Calling straight into the concrete implementation to reduce call stack
       depth */
    return MutableStringView{*this}.findLastOr(substring, nullptr);
}

StringView String::findLast(const StringView substring) const {
    /* Calling straight into the concrete implementation to reduce call stack
       depth */
    return StringView{*this}.findLastOr(substring, nullptr);
}

MutableStringView String::findLast(const char character) {
    /* Calling straight into the concrete implementation to reduce call stack
       depth */
    return MutableStringView{*this}.findLastOr(character, nullptr);
}

StringView String::findLast(const char character) const {
    /* Calling straight into the concrete implementation to reduce call stack
       depth */
    return StringView{*this}.findLastOr(character, nullptr);
}

MutableStringView String::findLastOr(const StringView substring, char* const fail) {
    return MutableStringView{*this}.findLastOr(substring, fail);
}

StringView String::findLastOr(const StringView substring, const char* const fail) const {
    return StringView{*this}.findLastOr(substring, fail);
}

MutableStringView String::findLastOr(const char character, char* const fail) {
    return MutableStringView{*this}.findLastOr(character, fail);
}

StringView String::findLastOr(const char character, const char* const fail) const {
    return StringView{*this}.findLastOr(character, fail);
}

bool String::contains(const StringView substring) const {
    return StringView{*this}.contains(substring);
}

bool String::contains(const char character) const {
    return StringView{*this}.contains(character);
}

MutableStringView String::findAny(const StringView characters) {
    return MutableStringView{*this}.findAny(characters);
}

StringView String::findAny(const StringView characters) const {
    return StringView{*this}.findAny(characters);
}

MutableStringView String::findAnyOr(const StringView characters, char* fail) {
    return MutableStringView{*this}.findAnyOr(characters, fail);
}

StringView String::findAnyOr(const StringView characters, const char* fail) const {
    return StringView{*this}.findAnyOr(characters, fail);
}

MutableStringView String::findLastAny(const StringView characters) {
    return MutableStringView{*this}.findLastAny(characters);
}

StringView String::findLastAny(const StringView characters) const {
    return StringView{*this}.findLastAny(characters);
}

MutableStringView String::findLastAnyOr(const StringView characters, char* fail) {
    return MutableStringView{*this}.findLastAnyOr(characters, fail);
}

StringView String::findLastAnyOr(const StringView characters, const char* fail) const {
    return StringView{*this}.findLastAnyOr(characters, fail);
}

bool String::containsAny(const StringView substring) const {
    return StringView{*this}.containsAny(substring);
}

char* String::release() {
    CORRADE_ASSERT(!(_small.size & 0x80),
        "Containers::String::release(): cannot call on a SSO instance", {});
    char* data = _large.data;

    /* Create a zero-size small string to fullfil the guarantee of data() being
       always non-null and null-terminated. Since this makes the string switch
       to SSO, we also clear the deleter this way. */
    _small.data[0] = '\0';
    _small.size = SmallSize;
    return data;
}

namespace Implementation {

String StringConverter<std::string>::from(const std::string& other) {
    return String{other.data(), other.size()};
}

std::string StringConverter<std::string>::to(const String& other) {
    return std::string{other.data(), other.size()};
}

}

}}
