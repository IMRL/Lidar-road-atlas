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

#include "Format.h"

#include <cstring>

#include "Corrade/Containers/ArrayView.h"
#include "Corrade/Containers/StringView.h"
#include "Corrade/Utility/Assert.h"
#include "Corrade/Utility/TypeTraits.h"

namespace Corrade { namespace Utility { namespace Implementation {

enum class FormatType: unsigned char {
    Unspecified,
    Character,
    Octal,
    Decimal,
    Hexadecimal,
    HexadecimalUppercase,
    Float,
    FloatUppercase,
    FloatExponent,
    FloatExponentUppercase,
    FloatFixed,
    FloatFixedUppercase
};

template<class> char formatTypeChar(FormatType type);

template<> char formatTypeChar<int>(FormatType type) {
    switch(type) {
        case FormatType::Character: return 'c';
        case FormatType::Unspecified:
        case FormatType::Decimal: return 'i';
        case FormatType::Octal: return 'o';
        case FormatType::Hexadecimal: return 'x';
        case FormatType::HexadecimalUppercase: return 'X';

        case FormatType::Float:
        case FormatType::FloatUppercase:
        case FormatType::FloatExponent:
        case FormatType::FloatExponentUppercase:
        case FormatType::FloatFixed:
        case FormatType::FloatFixedUppercase:
            /* Return some reasonable default so we can test for the assert */
            CORRADE_ASSERT_UNREACHABLE("Utility::format(): floating-point type used for an integral value", 'i');
    }

    CORRADE_INTERNAL_ASSERT_UNREACHABLE(); /* LCOV_EXCL_LINE */
}

template<> char formatTypeChar<long long>(const FormatType type) {
    /* Return some reasonable default so we can test for the assert */
    CORRADE_ASSERT(type != FormatType::Character,
        "Utility::format(): character type used for a 64-bit value", 'i');
    return formatTypeChar<int>(type);
}

template<> char formatTypeChar<unsigned int>(FormatType type) {
    switch(type) {
        case FormatType::Character: return 'c';
        case FormatType::Unspecified:
        case FormatType::Decimal: return 'u';
        case FormatType::Octal: return 'o';
        case FormatType::Hexadecimal: return 'x';
        case FormatType::HexadecimalUppercase: return 'X';

        case FormatType::Float:
        case FormatType::FloatUppercase:
        case FormatType::FloatExponent:
        case FormatType::FloatExponentUppercase:
        case FormatType::FloatFixed:
        case FormatType::FloatFixedUppercase:
            /* Return some reasonable default so we can test for the assert */
            CORRADE_ASSERT_UNREACHABLE("Utility::format(): floating-point type used for an integral value", 'u');
    }

    CORRADE_INTERNAL_ASSERT_UNREACHABLE(); /* LCOV_EXCL_LINE */
}

template<> char formatTypeChar<unsigned long long>(const FormatType type) {
    /* Return some reasonable default so we can test for the assert */
    CORRADE_ASSERT(type != FormatType::Character,
        "Utility::format(): character type used for a 64-bit value", 'i');
    return formatTypeChar<unsigned int>(type);
}

template<> char formatTypeChar<float>(FormatType type) {
    switch(type) {
        case FormatType::Unspecified:
        case FormatType::Float: return 'g';
        case FormatType::FloatUppercase: return 'G';
        case FormatType::FloatExponent: return 'e';
        case FormatType::FloatExponentUppercase: return 'E';
        case FormatType::FloatFixed: return 'f';
        case FormatType::FloatFixedUppercase: return 'F';

        case FormatType::Character:
            /* Return some reasonable default so we can test for the assert */
            CORRADE_ASSERT_UNREACHABLE("Utility::format(): character type used for a floating-point value", 'g');

        case FormatType::Decimal:
        case FormatType::Octal:
        case FormatType::Hexadecimal:
        case FormatType::HexadecimalUppercase:
            /* Return some reasonable default so we can test for the assert */
            CORRADE_ASSERT_UNREACHABLE("Utility::format(): integral type used for a floating-point value", 'g');
    }

    CORRADE_INTERNAL_ASSERT_UNREACHABLE(); /* LCOV_EXCL_LINE */
}

std::size_t Formatter<int>::format(const Containers::MutableStringView& buffer, const int value, int precision, const FormatType type) {
    if(precision == -1) precision = 1;
    const char format[]{ '%', '.', '*', formatTypeChar<int>(type), 0 };
    return std::snprintf(buffer.data(), buffer.size(), format, precision, value);
    return {};
}
void Formatter<int>::format(std::FILE* const file, const int value, int precision, FormatType type) {
    if(precision == -1) precision = 1;
    const char format[]{ '%', '.', '*', formatTypeChar<int>(type), 0 };
    std::fprintf(file, format, precision, value);
}
std::size_t Formatter<unsigned int>::format(const Containers::MutableStringView& buffer, const unsigned int value, int precision, const FormatType type) {
    if(precision == -1) precision = 1;
    const char format[]{ '%', '.', '*', formatTypeChar<unsigned int>(type), 0 };
    return std::snprintf(buffer.data(), buffer.size(), format, precision, value);
}
void Formatter<unsigned int>::format(std::FILE* const file, const unsigned int value, int precision, const FormatType type) {
    if(precision == -1) precision = 1;
    const char format[]{ '%', '.', '*', formatTypeChar<unsigned int>(type), 0 };
    std::fprintf(file, format, precision, value);
}
std::size_t Formatter<long long>::format(const Containers::MutableStringView& buffer, const long long value, int precision, const FormatType type) {
    if(precision == -1) precision = 1;
    const char format[]{ '%', '.', '*', 'l', 'l', formatTypeChar<long long>(type), 0 };
    return std::snprintf(buffer.data(), buffer.size(), format, precision, value);
}
void Formatter<long long>::format(std::FILE* const file, const long long value, int precision, const FormatType type) {
    if(precision == -1) precision = 1;
    const char format[]{ '%', '.', '*', 'l', 'l', formatTypeChar<long long>(type), 0 };
    std::fprintf(file, format, precision, value);
}
std::size_t Formatter<unsigned long long>::format(const Containers::MutableStringView& buffer, const unsigned long long value, int precision, const FormatType type) {
    if(precision == -1) precision = 1;
    const char format[]{ '%', '.', '*', 'l', 'l', formatTypeChar<unsigned long long>(type), 0 };
    return std::snprintf(buffer.data(), buffer.size(), format, precision, value);
}
void Formatter<unsigned long long>::format(std::FILE* const file, const unsigned long long value, int precision, const FormatType type) {
    if(precision == -1) precision = 1;
    const char format[]{ '%', '.', '*', 'l', 'l', formatTypeChar<unsigned long long>(type), 0 };
    std::fprintf(file, format, precision, value);
}

std::size_t Formatter<float>::format(const Containers::MutableStringView& buffer, const float value, int precision, const FormatType type) {
    if(precision == -1) precision = Implementation::FloatPrecision<float>::Digits;
    const char format[]{ '%', '.', '*', formatTypeChar<float>(type), 0 };
    return std::snprintf(buffer.data(), buffer.size(), format, precision, double(value));
}
void Formatter<float>::format(std::FILE* const file, const float value, int precision, const FormatType type) {
    if(precision == -1) precision = Implementation::FloatPrecision<float>::Digits;
    const char format[]{ '%', '.', '*', formatTypeChar<float>(type), 0 };
    std::fprintf(file, format, precision, double(value));
}

std::size_t Formatter<double>::format(const Containers::MutableStringView& buffer, const double value, int precision, const FormatType type) {
    if(precision == -1) precision = Implementation::FloatPrecision<double>::Digits;
    const char format[]{ '%', '.', '*', formatTypeChar<float>(type), 0 };
    return std::snprintf(buffer.data(), buffer.size(), format, precision, value);
}
void Formatter<double>::format(std::FILE* const file, const double value, int precision, const FormatType type) {
    if(precision == -1) precision = Implementation::FloatPrecision<double>::Digits;
    const char format[]{ '%', '.', '*', formatTypeChar<float>(type), 0 };
    std::fprintf(file, format, precision, value);
}

std::size_t Formatter<long double>::format(const Containers::MutableStringView& buffer, const long double value, int precision, const FormatType type) {
    if(precision == -1) precision = Implementation::FloatPrecision<long double>::Digits;
    const char format[]{ '%', '.', '*', 'L', formatTypeChar<float>(type), 0 };
    return std::snprintf(buffer.data(), buffer.size(), format, precision, value);
}
void Formatter<long double>::format(std::FILE* const file, const long double value, int precision, const FormatType type) {
    if(precision == -1) precision = Implementation::FloatPrecision<long double>::Digits;
    const char format[]{ '%', '.', '*', 'L', formatTypeChar<float>(type), 0 };
    std::fprintf(file, format, precision, value);
}

std::size_t Formatter<Containers::StringView>::format(const Containers::MutableStringView& buffer, const Containers::StringView value, const int precision, const FormatType type) {
    std::size_t size = value.size();
    if(std::size_t(precision) < size) size = precision;
    CORRADE_ASSERT(type == FormatType::Unspecified,
        "Utility::format(): type specifier can't be used for a string value", {});
    #ifdef CORRADE_NO_ASSERT
    static_cast<void>(type);
    #endif
    /* strncpy() would stop on \0 characters */
    /* Apparently memcpy() can't be called with null pointers, even if size is
       zero. I call that bullying. */
    if(buffer.data() && size) std::memcpy(buffer.data(), value.data(), size);
    return size;
}
void Formatter<Containers::StringView>::format(std::FILE* const file, const Containers::StringView value, const int precision, const FormatType type) {
    std::size_t size = value.size();
    if(std::size_t(precision) < size) size = precision;
    CORRADE_ASSERT(type == FormatType::Unspecified,
        "Utility::format(): type specifier can't be used for a string value", );
    #ifdef CORRADE_NO_ASSERT
    static_cast<void>(type);
    #endif
    std::fwrite(value.data(), size, 1, file);
}
std::size_t Formatter<const char*>::format(const Containers::MutableStringView& buffer, const char* value, const int precision, const FormatType type) {
    return Formatter<Containers::StringView>::format(buffer, value, precision, type);
}
void Formatter<const char*>::format(std::FILE* const file, const char* value, const int precision, const FormatType type) {
    Formatter<Containers::StringView>::format(file, value, precision, type);
}
#ifdef CORRADE_BUILD_DEPRECATED
std::size_t Formatter<Containers::ArrayView<const char>>::format(const Containers::MutableStringView& buffer, const Containers::ArrayView<const char> value, const int precision, const FormatType type) {
    return Formatter<Containers::StringView>::format(buffer, value, precision, type);
}
void Formatter<Containers::ArrayView<const char>>::format(std::FILE* const file, const Containers::ArrayView<const char> value, const int precision, const FormatType type) {
    Formatter<Containers::StringView>::format(file, value, precision, type);
}
#endif

namespace {

int parseNumber(const Containers::StringView format, std::size_t& formatOffset) {
    int number = -1;
    while(formatOffset < format.size() && format[formatOffset] >= '0' && format[formatOffset] <= '9') {
        if(number == -1) number = 0;
        else number *= 10;
        number += (format[formatOffset] - '0');
        ++formatOffset;
    }
    return number;
}

template<class Writer, class FormattedWriter, class Formatter> void formatWith(const Writer writer, const FormattedWriter formattedWriter, const Containers::StringView format, const Containers::ArrayView<Formatter> formatters) {
    bool inPlaceholder = false;
    std::size_t placeholderOffset = 0;
    std::size_t formatterToGo = 0;
    int placeholderIndex = -1;
    int precision = -1;
    FormatType type = FormatType::Unspecified;
    for(std::size_t formatOffset = 0; formatOffset != format.size(); ) {
        /* Placeholder begin (or escaped {) */
        if(format[formatOffset] == '{') {
            if(formatOffset + 1 < format.size() && format[formatOffset+1] == '{') {
                writer(format.slice(formatOffset, formatOffset + 1));
                formatOffset += 2;
                continue;
            }

            CORRADE_INTERNAL_ASSERT(!inPlaceholder);
            inPlaceholder = true;
            placeholderOffset = formatOffset;
            placeholderIndex = -1;
            precision = -1;
            type = FormatType::Unspecified;

            ++formatOffset;
            continue;
        }

        /* Placeholder end (or escaped }) */
        if(format[formatOffset] == '}') {
            if(!inPlaceholder && formatOffset + 1 < format.size() && format[formatOffset+1] == '}') {
                writer(format.slice(formatOffset, formatOffset + 1));
                formatOffset += 2;
                continue;
            }

            CORRADE_ASSERT(inPlaceholder, "Utility::format(): mismatched }", );
            inPlaceholder = false;

            /* If the placeholder was numbered, use that number, otherwise
               just use the formatter that's next */
            if(placeholderIndex != -1) formatterToGo = placeholderIndex;

            /* Formatter index is in bounds, write */
            if(formatterToGo < formatters.size())
                formattedWriter(formatters[formatterToGo], precision, type);

            /* Otherwise just verbatim copy the placeholder (including }) */
            else writer(format.slice(placeholderOffset, formatOffset + 1));

            /* Next time we see an unnumbered placeholder, take the next
               formatter */
            ++formatterToGo;

            ++formatOffset;
            continue;
        }

        /* Placeholder contents */
        if(inPlaceholder) {
            /* Placeholder index */
            placeholderIndex = parseNumber(format, formatOffset);

            /* Formatting options */
            if(formatOffset < format.size() && format[formatOffset] == ':') {
                ++formatOffset;

                /* Precision */
                if(formatOffset + 1 < format.size() && format[formatOffset] == '.') {
                    ++formatOffset;
                    precision = parseNumber(format, formatOffset);
                    CORRADE_ASSERT(precision != -1,
                        "Utility::format(): invalid character in precision specifier:" << format.slice(formatOffset, formatOffset + 1), );
                }

                /* Type */
                if(formatOffset < format.size() && format[formatOffset] != '}') {
                    switch(format[formatOffset]) {
                        /** @todo binary */
                        case 'c':
                            type = FormatType::Character;
                            break;
                        case 'o':
                            type = FormatType::Octal;
                            break;
                        case 'd':
                            type = FormatType::Decimal;
                            break;
                        case 'x':
                            type = FormatType::Hexadecimal;
                            break;
                        case 'X':
                            type = FormatType::HexadecimalUppercase;
                            break;
                        case 'g':
                            type = FormatType::Float;
                            break;
                        case 'G':
                            type = FormatType::FloatUppercase;
                            break;
                        case 'e':
                            type = FormatType::FloatExponent;
                            break;
                        case 'E':
                            type = FormatType::FloatExponentUppercase;
                            break;
                        case 'f':
                            type = FormatType::FloatFixed;
                            break;
                        case 'F':
                            type = FormatType::FloatFixedUppercase;
                            break;
                        default:
                            CORRADE_ASSERT(false,
                                "Utility::format(): invalid type specifier:" << format.slice(formatOffset, formatOffset + 1), );
                    }
                    ++formatOffset;
                }
            }

            /* Unexpected end, break -- the assert at the end of function
               takes care of this */
            if(formatOffset == format.size()) break;

            /* Next should be the placeholder end */
            CORRADE_ASSERT(format[formatOffset] == '}',
                "Utility::format(): unknown placeholder content:" << format.slice(formatOffset, formatOffset + 1), );
            continue;
        }

        /* Other things, just copy. Grab as much as I can to avoid calling
           functions on single bytes. */
        std::size_t next = formatOffset;
        while(next < format.size() && format[next] != '{' && format[next] != '}')
            ++next;
        writer(format.slice(formatOffset, next));
        formatOffset = next;
    }

    CORRADE_ASSERT(!inPlaceholder, "Utility::format(): unexpected end of format string", );
}

}

std::size_t formatInto(const Containers::MutableStringView& buffer, const char* const format, BufferFormatter* const formatters, std::size_t formatterCount) {
    std::size_t bufferOffset = 0;
    formatWith([&buffer, &bufferOffset](Containers::StringView data) {
        if(buffer.data()) {
            CORRADE_ASSERT(data.size() <= buffer.size(),
                "Utility::formatInto(): buffer too small, expected at least" << bufferOffset + data.size() << "but got" << bufferOffset + buffer.size(), );
            /* strncpy() would stop on \0 characters */
            /* data.size() can't be 0 because that would make the above assert
               fail, thus data can't be nullptr either and so we don't need to
               check anything to avoid calling memcpy() with a null pointer */
            std::memcpy(buffer.data() + bufferOffset, data.data(), data.size());
        }
        bufferOffset += data.size();
    }, [&buffer, &bufferOffset](BufferFormatter& formatter, int precision, FormatType type) {
        if(buffer.data()) {
            formatter.size = formatter(buffer.exceptPrefix(bufferOffset), precision, type);
            CORRADE_ASSERT(bufferOffset + formatter.size <= buffer.size(),
                "Utility::formatInto(): buffer too small, expected at least" << bufferOffset + formatter.size << "but got" << buffer.size(), );
        } else if(formatter.size == ~std::size_t{})
            formatter.size = formatter(nullptr, precision, type);
        bufferOffset += formatter.size;
    }, format, Containers::arrayView(formatters, formatterCount));
    return bufferOffset;
}

void formatInto(std::FILE* const file, const char* format, FileFormatter* const formatters, std::size_t formatterCount) {
    formatWith([&file](Containers::StringView data) {
        fwrite(data.data(), data.size(), 1, file);
    }, [&file](const FileFormatter& formatter, int precision, FormatType type) {
        formatter(file, precision, type);
    }, format, Containers::arrayView(formatters, formatterCount));
}

}

}}
