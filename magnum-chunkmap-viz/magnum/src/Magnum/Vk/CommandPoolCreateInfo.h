#ifndef Magnum_Vk_CommandPoolCreateInfo_h
#define Magnum_Vk_CommandPoolCreateInfo_h
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
 * @brief Class @ref Magnum::Vk::CommandPoolCreateInfo
 * @m_since_latest
 */

#include <Corrade/Containers/EnumSet.h>

#include "Magnum/Tags.h"
#include "Magnum/Magnum.h"
#include "Magnum/Vk/Vk.h"
#include "Magnum/Vk/Vulkan.h"
#include "Magnum/Vk/visibility.h"

namespace Magnum { namespace Vk {

/**
@brief Command pool creation info
@m_since_latest

Wraps a @type_vk_keyword{CommandPoolCreateInfo}. See
@ref Vk-CommandPool-creation "Command pool creation" for usage information.
*/
class MAGNUM_VK_EXPORT CommandPoolCreateInfo {
    public:
        /**
         * @brief Command pool creation flag
         *
         * Wraps @type_vk_keyword{CommandPoolCreateFlagBits}.
         * @see @ref Flags, @ref CommandPoolCreateInfo(UnsignedInt, Flags)
         * @m_enum_values_as_keywords
         */
        enum class Flag: UnsignedInt {
            /** Command buffers allocated from this pool will be short-lived */
            Transient = VK_COMMAND_POOL_CREATE_TRANSIENT_BIT,

            /**
             * Allow individual command buffers to be reset to initial state
             * using @ref CommandBuffer::reset() instead of just the whole pool
             * using @ref CommandPool::reset().
             *
             * @m_class{m-note m-success}
             *
             * @par
             *      Not using this flag may help the driver to use simpler
             *      per-pool allocators instead of per-buffer.
             */
            ResetCommandBuffer = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT
        };

        /**
         * @brief Command pool creation flags
         *
         * Type-safe wrapper for @type_vk_keyword{CommandPoolCreateFlags}.
         * @see @ref CommandPoolCreateInfo(UnsignedInt, Flags)
         */
        typedef Containers::EnumSet<Flag> Flags;

        /**
         * @brief Constructor
         * @param queueFamilyIndex  Queue family index
         * @param flags             Command pool creation flags
         *
         * The following @type_vk{CommandPoolCreateInfo} fields are pre-filled
         * in addition to `sType`, everything else is zero-filled:
         *
         * -    `flags`
         * -    `queueFamilyIndex`
         *
         * @see @ref DeviceProperties::pickQueueFamily()
         */
        explicit CommandPoolCreateInfo(UnsignedInt queueFamilyIndex, Flags flags = {});

        /**
         * @brief Construct without initializing the contents
         *
         * Note that not even the `sType` field is set --- the structure has to
         * be fully initialized afterwards in order to be usable.
         */
        explicit CommandPoolCreateInfo(NoInitT) noexcept;

        /**
         * @brief Construct from existing data
         *
         * Copies the existing values verbatim, pointers are kept unchanged
         * without taking over the ownership. Modifying the newly created
         * instance will not modify the original data nor the pointed-to data.
         */
        explicit CommandPoolCreateInfo(const VkCommandPoolCreateInfo& info);

        /** @brief Underlying @type_vk{CommandPoolCreateInfo} structure */
        VkCommandPoolCreateInfo& operator*() { return _info; }
        /** @overload */
        const VkCommandPoolCreateInfo& operator*() const { return _info; }
        /** @overload */
        VkCommandPoolCreateInfo* operator->() { return &_info; }
        /** @overload */
        const VkCommandPoolCreateInfo* operator->() const { return &_info; }
        /** @overload */
        operator const VkCommandPoolCreateInfo*() const { return &_info; }

    private:
        VkCommandPoolCreateInfo _info;
};

CORRADE_ENUMSET_OPERATORS(CommandPoolCreateInfo::Flags)

}}

/* Make the definition complete -- it doesn't make sense to have a CreateInfo
   without the corresponding object anyway. */
#include "Magnum/Vk/CommandPool.h"

#endif
