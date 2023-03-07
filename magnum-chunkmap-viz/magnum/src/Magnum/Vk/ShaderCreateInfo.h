#ifndef Magnum_Vk_ShaderCreateInfo_h
#define Magnum_Vk_ShaderCreateInfo_h
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
 * @brief Class @ref Magnum::Vk::ShaderCreateInfo
 * @m_since_latest
 */

#include <Corrade/Containers/EnumSet.h>

#include "Magnum/Magnum.h"
#include "Magnum/Tags.h"
#include "Magnum/Vk/Vk.h"
#include "Magnum/Vk/Vulkan.h"
#include "Magnum/Vk/visibility.h"

namespace Magnum { namespace Vk {

/**
@brief Shader creation info
@m_since_latest

Wraps a @type_vk_keyword{ShaderModuleCreateInfo}. See
@ref Vk-Shader-creation "Shader creation" for usage information.
*/
class MAGNUM_VK_EXPORT ShaderCreateInfo {
    public:
        /**
         * @brief Shader creation flag
         *
         * Wraps @type_vk_keyword{ShaderModuleCreateFlagBits}.
         * @see @ref Flags, @ref ShaderCreateInfo()
         * @m_enum_values_as_keywords
         */
        enum class Flag: UnsignedInt {};

        /**
         * @brief Shader creation flags
         *
         * Type-safe wrapper for @type_vk_keyword{ShaderModuleCreateFlags}.
         * @see @ref ShaderCreateInfo()
         */
        typedef Containers::EnumSet<Flag> Flags;

        /**
         * @brief Constructor
         * @param code      Shader code
         * @param flags     Shader creation flags
         *
         * The following @type_vk{ShaderModuleCreateInfo} fields are pre-filled
         * in addition to `sType`, everything else is zero-filled:
         *
         * -    `flags`
         * -    `pCode` and `codeSize` to @p code
         *
         * @attention The class doesn't make any copy of @p code, so you either
         *      have to ensure it stays in scope until @ref Shader is
         *      constructed, or instantiate a temporary @ref ShaderCreateInfo
         *      directly in @ref Shader constructor call expression, as shown
         *      in its usage docs. If you have the data in
         *      @ref Corrade::Containers::Array, there's also
         *      @ref ShaderCreateInfo(Containers::Array<T>&&, Flags).
         */
        explicit ShaderCreateInfo(Containers::ArrayView<const void> code, Flags flags = {});

        /**
         * @brief Construct taking ownership of a r-value array instance
         *
         * Behaves like @ref ShaderCreateInfo(Containers::ArrayView<const void>, Flags)
         * but in addition ensures @p code stays in scope until @ref Shader is
         * created, deleting it on destruction. The cleanup relies on the
         * pointer and size stored in @type_vk{ShaderModuleCreateInfo},
         * changing the `pCode` and `codeSize` members afterwards may result in
         * memory corruption.
         */
        template<class T> explicit ShaderCreateInfo(Containers::Array<T>&& code, Flags flags = {});

        /**
         * @overload
         *
         * Sorry, custom @ref Corrade::Containers::Array deleter types can't be
         * taken over.
         */
        template<class T, class Deleter> explicit ShaderCreateInfo(Containers::Array<T, Deleter>&& code, Flags flags = {}) = delete;

        /**
         * @brief Construct without initializing the contents
         *
         * Note that not even the `sType` field is set --- the structure has to
         * be fully initialized afterwards in order to be usable.
         */
        explicit ShaderCreateInfo(NoInitT) noexcept;

        /**
         * @brief Construct from existing data
         *
         * Copies the existing values verbatim, pointers are kept unchanged
         * without taking over the ownership. Modifying the newly created
         * instance will not modify the original data nor the pointed-to data.
         */
        explicit ShaderCreateInfo(const VkShaderModuleCreateInfo& info);

        /** @brief Copying is not allowed */
        ShaderCreateInfo(const ShaderCreateInfo&) = delete;

        /** @brief Move constructor */
        ShaderCreateInfo(ShaderCreateInfo&& other) noexcept;

        /**
         * @brief Destructor
         *
         * If the @ref ShaderCreateInfo(Containers::Array<T>&&, Flags)
         * constructor was used, calls deleter on the stored array.
         */
        ~ShaderCreateInfo();

        /** @brief Copying is not allowed */
        ShaderCreateInfo& operator=(const ShaderCreateInfo&) = delete;

        /** @brief Move assignment */
        ShaderCreateInfo& operator=(ShaderCreateInfo&& other) noexcept;

        /** @brief Underlying @type_vk{ShaderModuleCreateInfo} structure */
        VkShaderModuleCreateInfo& operator*() { return _info; }
        /** @overload */
        const VkShaderModuleCreateInfo& operator*() const { return _info; }
        /** @overload */
        VkShaderModuleCreateInfo* operator->() { return &_info; }
        /** @overload */
        const VkShaderModuleCreateInfo* operator->() const { return &_info; }
        /** @overload */
        operator const VkShaderModuleCreateInfo*() const { return &_info; }

    private:
        VkShaderModuleCreateInfo _info;

        /* Used by the Array&& constructor. Instead of wrapping an Array of an
           arbitrary type we just take its deleter, the pointer + size pair is
           stored in _info already. */
        void(*_originalDeleter)(){};
        void(*_deleter)(void(*)(), const void*, std::size_t){};
};

CORRADE_ENUMSET_OPERATORS(ShaderCreateInfo::Flags)

template<class T> /*implicit*/ ShaderCreateInfo::ShaderCreateInfo(Containers::Array<T>&& code, Flags flags): ShaderCreateInfo{code, flags} {
    /* Remember the deleter. The pointer and size is stored in
       VkShaderModuleCreateInfo and we assume it won't get stomped on
       afterwards */
    _originalDeleter = reinterpret_cast<void(*)()>(code.deleter() ? code.deleter() : static_cast<void(*)(T*, std::size_t)>([](T* data, std::size_t) { delete[] data; }));
    _deleter = [](void(*originalDeleter)(), const void* data, std::size_t size) {
        /* VkShaderModuleCreateInfo stores the size in bytes, convert it back
           to the element count for the deleter */
        reinterpret_cast<void(*)(T*, std::size_t)>(originalDeleter)(reinterpret_cast<T*>(const_cast<void*>(data)), size/sizeof(T));
    };

    /* Release the original array so the deleter isn't called too early */
    code.release();
}

}}

/* Make the definition complete -- it doesn't make sense to have a CreateInfo
   without the corresponding object anyway. */
#include "Magnum/Vk/Shader.h"

#endif
