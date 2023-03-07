#ifndef Magnum_Mesh_h
#define Magnum_Mesh_h
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
 * @brief Enum @ref Magnum::MeshPrimitive, @ref Magnum::MeshIndexType, function @ref Magnum::isMeshPrimitiveImplementationSpecific(), @ref Magnum::meshPrimitiveWrap(), @ref Magnum::meshPrimitiveUnwrap(), @ref Magnum::meshIndexTypeSize()
 */

#include <Corrade/Utility/Assert.h>
#include <Corrade/Utility/StlForwardString.h>

#include "Magnum/Magnum.h"
#include "Magnum/visibility.h"

namespace Magnum {

/**
@brief Mesh primitive type

Can act also as a wrapper for implementation-specific mesh primitive types
using @ref meshPrimitiveWrap() and @ref meshPrimitiveUnwrap(). Distinction
between generic and implementation-specific primitive types can be done using
@ref isMeshPrimitiveImplementationSpecific().

In case of OpenGL, corresponds to @ref GL::MeshPrimitive and is convertible to
it using @ref GL::meshPrimitive(). See documentation of each value for more
information about the mapping.

In case of Vulkan, corresponds to @type_vk_keyword{PrimitiveTopology} and is
convertible to it using @ref Vk::vkPrimitiveTopology(). See documentation of
each value for more information about the mapping. Note that not every mode is available there, use @ref Vk::hasVkPrimitiveTopology() to check for its
presence.

For D3D, corresponds to @m_class{m-doc-external} [D3D_PRIMITIVE_TOPOLOGY](https://docs.microsoft.com/en-us/windows/win32/api/d3dcommon/ne-d3dcommon-d3d_primitive_topology);
for Metal, corresponds to @m_class{m-doc-external} [MTLPrimitiveType](https://developer.apple.com/documentation/metal/mtlprimitivetype?language=objc).
See documentation of each value for more information about the mapping.

@see @ref MeshTools::primitiveCount(MeshPrimitive, UnsignedInt)
*/
enum class MeshPrimitive: UnsignedInt {
    /* Zero reserved for an invalid type (but not being a named value) */

    /**
     * Single points.
     *
     * Corresponds to @ref GL::MeshPrimitive::Points;
     * @val_vk_keyword{PRIMITIVE_TOPOLOGY_POINT_LIST,PrimitiveTopology};
     * @m_class{m-doc-external} [D3D_PRIMITIVE_TOPOLOGY_POINTLIST](https://docs.microsoft.com/en-us/windows/win32/api/d3dcommon/ne-d3dcommon-d3d_primitive_topology)
     * or @m_class{m-doc-external} [MTLPrimitiveTypePoint](https://developer.apple.com/documentation/metal/mtlprimitivetype/mtlprimitivetypepoint?language=objc).
     * @m_keywords{D3D_PRIMITIVE_TOPOLOGY_POINTLIST MTLPrimitiveTypePoint}
     */
    Points = 1,

    /**
     * Each pair of vertices defines a single line, lines aren't
     * connected together.
     *
     * Corresponds to @ref GL::MeshPrimitive::Lines /
     * @val_vk_keyword{PRIMITIVE_TOPOLOGY_LINE_LIST,PrimitiveTopology};
     * @m_class{m-doc-external} [D3D_PRIMITIVE_TOPOLOGY_LINELIST](https://docs.microsoft.com/en-us/windows/win32/api/d3dcommon/ne-d3dcommon-d3d_primitive_topology)
     * or @m_class{m-doc-external} [MTLPrimitiveTypeLine](https://developer.apple.com/documentation/metal/mtlprimitivetype/mtlprimitivetypeline?language=objc).
     * @m_keywords{D3D_PRIMITIVE_TOPOLOGY_LINELIST MTLPrimitiveTypeLine}
     */
    Lines,

    /**
     * Like @ref MeshPrimitive::LineStrip, but with last and first vertex
     * connected together.
     *
     * Corresponds to @ref GL::MeshPrimitive::LineLoop. Not supported on
     * Vulkan, D3D or Metal.
     * @see @ref MeshTools::generateLineLoopIndices()
     */
    LineLoop,

    /**
     * First two vertices define first line segment, each following
     * vertex defines another segment.
     *
     * Corresponds to @ref GL::MeshPrimitive::LineStrip /
     * @val_vk_keyword{PRIMITIVE_TOPOLOGY_LINE_STRIP,PrimitiveTopology};
     * @m_class{m-doc-external} [D3D_PRIMITIVE_TOPOLOGY_LINESTRIP](https://docs.microsoft.com/en-us/windows/win32/api/d3dcommon/ne-d3dcommon-d3d_primitive_topology)
     * or @m_class{m-doc-external} [MTLPrimitiveTypeLineStrip](https://developer.apple.com/documentation/metal/mtlprimitivetype/mtlprimitivetypelinestrip?language=objc).
     * @m_keywords{D3D_PRIMITIVE_TOPOLOGY_LINESTRIP MTLPrimitiveTypeLineStrip}
     * @see @ref MeshTools::generateLineStripIndices()
     */
    LineStrip,

    /**
     * Each three vertices define one triangle.
     *
     * Corresponds to @ref GL::MeshPrimitive::Triangles /
     * @val_vk_keyword{PRIMITIVE_TOPOLOGY_TRIANGLE_LIST,PrimitiveTopology};
     * @m_class{m-doc-external} [D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST](https://docs.microsoft.com/en-us/windows/win32/api/d3dcommon/ne-d3dcommon-d3d_primitive_topology)
     * or @m_class{m-doc-external} [MTLPrimitiveTypeTriangle](https://developer.apple.com/documentation/metal/mtlprimitivetype/mtlprimitivetypetriangle?language=objc).
     * @m_keywords{D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST MTLPrimitiveTypeTriangle}
     */
    Triangles,

    /**
     * First three vertices define first triangle, each following
     * vertex defines another triangle.
     *
     * Corresponds to @ref GL::MeshPrimitive::TriangleStrip /
     * @val_vk_keyword{PRIMITIVE_TOPOLOGY_TRIANGLE_STRIP,PrimitiveTopology} or
     * @m_class{m-doc-external} [D3D_PRIMITIVE_TOPOLOGY_TRIANGLESTRIP](https://docs.microsoft.com/en-us/windows/win32/api/d3dcommon/ne-d3dcommon-d3d_primitive_topology)
     * or @m_class{m-doc-external} [MTLPrimitiveTypeTriangleStrip](https://developer.apple.com/documentation/metal/mtlprimitivetype/mtlprimitivetypetrianglestrip?language=objc).
     * @m_keywords{D3D_PRIMITIVE_TOPOLOGY_TRIANGLESTRIP MTLPrimitiveTypeTriangleStrip}
     * @see @ref MeshTools::generateTriangleStripIndices()
     */
    TriangleStrip,

    /**
     * First vertex is center, each following vertex is connected to
     * previous and center vertex.
     *
     * Corresponds to @ref GL::MeshPrimitive::TriangleFan /
     * @val_vk_keyword{PRIMITIVE_TOPOLOGY_TRIANGLE_FAN,PrimitiveTopology}. Not
     * supported on D3D or Metal.
     * @see @ref MeshTools::generateTriangleFanIndices()
     */
    TriangleFan,

    /**
     * Per-instance data.
     * @m_since{2020,06}
     *
     * Has no direct mapping to GPU APIs, but can be used to annotate
     * @ref Trade::MeshData containing per-instance data (such as colors,
     * transformations or texture layers) and then used to populate an instance
     * buffer. Index buffer has no defined meaning for instance data.
     */
    Instances,

    /**
     * Per-face data.
     * @m_since{2020,06}
     *
     * Can be used to annotate @ref Trade::MeshData containing data that are
     * per-face, as opposed to per-vertex. Has no direct mapping to common GPU
     * APIs, there it either has to be converted to per-vertex (which usually
     * involves slightly duplicating the original per-vertex data) or accessed
     * via a direct buffer/texture fetch from a shader using e.g.
     * @glsl gl_VertexID @ce. Index buffer can be used to deduplicate per-face
     * data.
     */
    Faces,

    /**
     * Per-edge data.
     * @m_since{2020,06}
     *
     * Can be used to annotate @ref Trade::MeshData containing data that are
     * per-edge, as opposed to per-vertex. This is different from
     * @ref MeshPrimitive::Lines as it has just one entry per line segment,
     * instead of two. Has no direct mapping to common GPU APIs, there it has
     * to be converted to per-vertex (which usually involves slightly
     * duplicating the original per-vertex data). Index buffer can be used to
     * deduplicate per-face data. Can also be used for example to describe a
     * half-edge mesh representation.
     * @see @ref Trade::meshAttributeCustom()
     */
    Edges,

    /**
     * Meshlet data.
     * @m_since_latest
     *
     * Can be used to annotate @ref Trade::MeshData containing meshlet chunks,
     * i.e. groups of vertex references together with per-meshlet culling
     * information such as a bounding sphere or visibility cone.
     */
    Meshlets
};

/** @debugoperatorenum{MeshPrimitive} */
MAGNUM_EXPORT Debug& operator<<(Debug& debug, MeshPrimitive value);

/**
@brief Whether a @ref MeshPrimitive value wraps an implementation-specific identifier
@m_since{2020,06}

Returns @cpp true @ce if value of @p primitive has its highest bit set,
@cpp false @ce otherwise. Use @ref meshPrimitiveWrap() and @ref meshPrimitiveUnwrap()
to wrap/unwrap an implementation-specific indentifier to/from
@ref MeshPrimitive.
*/
constexpr bool isMeshPrimitiveImplementationSpecific(MeshPrimitive primitive) {
    return UnsignedInt(primitive) & (1u << 31);
}

/**
@brief Wrap an implementation-specific mesh primitive identifier in @ref MeshPrimitive
@m_since{2020,06}

Sets the highest bit on @p implementationSpecific to mark it as
implementation-specific. Expects that @p implementationSpecific fits into the
remaining 31 bits. Use @ref meshPrimitiveUnwrap() for the inverse operation.
@see @ref isMeshPrimitiveImplementationSpecific()
*/
template<class T> constexpr MeshPrimitive meshPrimitiveWrap(T implementationSpecific) {
    static_assert(sizeof(T) <= 4, "types larger than 32bits are not supported");
    return CORRADE_CONSTEXPR_ASSERT(!(UnsignedInt(implementationSpecific) & (1u << 31)),
        "meshPrimitiveWrap(): implementation-specific value" << reinterpret_cast<void*>(implementationSpecific) << "already wrapped or too large"),
        MeshPrimitive((1u << 31)|UnsignedInt(implementationSpecific));
}

/**
@brief Unwrap an implementation-specific mesh primitive identifier from @ref MeshPrimitive
@m_since{2020,06}

Unsets the highest bit from @p primitive to extract the implementation-specific
value. Expects that @p primitive has it set. Use @ref meshPrimitiveWrap() for
the inverse operation.
@see @ref isMeshPrimitiveImplementationSpecific()
*/
template<class T = UnsignedInt> constexpr T meshPrimitiveUnwrap(MeshPrimitive primitive) {
    return CORRADE_CONSTEXPR_ASSERT(UnsignedInt(primitive) & (1u << 31),
        "meshPrimitiveUnwrap():" << primitive << "isn't a wrapped implementation-specific value"),
        T(UnsignedInt(primitive) & ~(1u << 31));
}

/**
@brief Mesh index type

A counterpart to @ref VertexFormat describing a mesh index type. Can act also
as a wrapper for implementation-specific mesh index type valueus uing
@ref meshIndexTypeWrap() and @ref meshIndexTypeUnwrap(). Distinction between
generic and implementation-specific types can be done using
@ref isMeshIndexTypeImplementationSpecific().

In case of OpenGL, corresponds to @ref GL::MeshIndexType and is convertible to
it using @ref GL::meshIndexType(). See documentation of each value for more
information about the mapping.

In case of Vulkan, corresponds to @ref Vk::MeshIndexType and is convertible to
it using @ref Vk::meshIndexType(). See documentation of each value for more
information about the mapping.

For D3D, corresponds to @m_class{m-doc-external} [DXGI_FORMAT](https://docs.microsoft.com/en-us/windows/win32/api/dxgiformat/ne-dxgiformat-dxgi_format);
for Metal, corresponds to @m_class{m-doc-external} [MTLIndexType](https://developer.apple.com/documentation/metal/mtlindextype?language=objc).
See documentation of each value for more information about the mapping.
@see @ref meshIndexTypeSize()
*/
enum class MeshIndexType: UnsignedInt {
    /* Zero reserved for an invalid type (but not being a named value) */

    /**
     * @relativeref{Magnum,UnsignedByte}.
     *
     * Corresponds to @ref GL::MeshIndexType::UnsignedByte or
     * @ref Vk::MeshIndexType::UnsignedByte. No D3D or Metal equivalent. Even
     * though OpenGL supports this type and Vulkan can as well via an
     * extension, using this type is discouraged on contemporary GPU
     * architectures. Prefer using 16-bit indices instead.
     */
    UnsignedByte = 1,

    /**
     * @relativeref{Magnum,UnsignedShort}.
     *
     * Corresponds to @ref GL::MeshIndexType::UnsignedShort /
     * @ref Vk::MeshIndexType::UnsignedShort;
     * @m_class{m-doc-external} [DXGI_FORMAT_R16_UINT](https://docs.microsoft.com/en-us/windows/win32/api/dxgiformat/ne-dxgiformat-dxgi_format)
     * or @m_class{m-doc-external} [MTLIndexTypeUInt16](https://developer.apple.com/documentation/metal/mtlindextype/mtlindextypeuint16?language=objc).
     * @m_keywords{DXGI_FORMAT_R16_UINT MTLIndexTypeUInt16}
     */
    UnsignedShort,

    /**
     * @relativeref{Magnum,UnsignedInt}.
     *
     * Corresponds to @ref GL::MeshIndexType::UnsignedInt /
     * @ref Vk::MeshIndexType::UnsignedInt;
     * @m_class{m-doc-external} [DXGI_FORMAT_R32_UINT](https://docs.microsoft.com/en-us/windows/win32/api/dxgiformat/ne-dxgiformat-dxgi_format)
     * or @m_class{m-doc-external} [MTLIndexTypeUInt32](https://developer.apple.com/documentation/metal/mtlindextype/mtlindextypeuint32?language=objc).
     * @m_keywords{DXGI_FORMAT_R32_UINT MTLIndexTypeUInt32}
     */
    UnsignedInt
};

/** @debugoperatorenum{MeshIndexType} */
MAGNUM_EXPORT Debug& operator<<(Debug& debug, MeshIndexType value);

/**
@brief Whether a @ref MeshIndexType value wraps an implementation-specific identifier
@m_since_latest

Returns @cpp true @ce if value of @p type has its highest bit set,
@cpp false @ce otherwise. Use @ref meshIndexTypeWrap() and
@ref meshIndexTypeUnwrap() to wrap/unwrap an implementation-specific
indentifier to/from @ref MeshIndexType.
*/
constexpr bool isMeshIndexTypeImplementationSpecific(MeshIndexType type) {
    return UnsignedInt(type) & (1u << 31);
}

/**
@brief Wrap an implementation-specific mesh index type identifier in @ref MeshIndexType
@m_since_latest

Sets the highest bit on @p implementationSpecific to mark it as
implementation-specific. Expects that @p implementationSpecific fits into the
remaining 31 bits. Use @ref meshIndexTypeUnwrap() for the inverse operation.
@see @ref isMeshIndexTypeImplementationSpecific()
*/
template<class T> constexpr MeshIndexType meshIndexTypeWrap(T implementationSpecific) {
    static_assert(sizeof(T) <= 4, "types larger than 32bits are not supported");
    return CORRADE_CONSTEXPR_ASSERT(!(UnsignedInt(implementationSpecific) & (1u << 31)),
        "meshIndexTypeWrap(): implementation-specific value" << reinterpret_cast<void*>(implementationSpecific) << "already wrapped or too large"),
        MeshIndexType((1u << 31)|UnsignedInt(implementationSpecific));
}

/**
@brief Unwrap an implementation-specific mesh index type identifier from @ref MeshIndexType
@m_since_latest

Unsets the highest bit from @p type to extract the implementation-specific
value. Expects that @p type has it set. Use @ref meshIndexTypeWrap() for
the inverse operation.
@see @ref isMeshIndexTypeImplementationSpecific()
*/
template<class T = UnsignedInt> constexpr T meshIndexTypeUnwrap(MeshIndexType type) {
    return CORRADE_CONSTEXPR_ASSERT(UnsignedInt(type) & (1u << 31),
        "meshIndexTypeUnwrap():" << type << "isn't a wrapped implementation-specific value"),
        T(UnsignedInt(type) & ~(1u << 31));
}

/** @brief Size of given mesh index type */
MAGNUM_EXPORT UnsignedInt meshIndexTypeSize(MeshIndexType type);

}

namespace Corrade { namespace Utility {

/** @configurationvalue{Magnum::MeshPrimitive} */
template<> struct MAGNUM_EXPORT ConfigurationValue<Magnum::MeshPrimitive> {
    ConfigurationValue() = delete;

    /**
     * @brief Writes enum value as string
     *
     * If the value is invalid, returns empty string.
     */
    static std::string toString(Magnum::MeshPrimitive value, ConfigurationValueFlags);

    /**
     * @brief Reads enum value as string
     *
     * If the value is invalid, returns a zero (invalid) primitive.
     */
    static Magnum::MeshPrimitive fromString(const std::string& stringValue, ConfigurationValueFlags);
};

/** @configurationvalue{Magnum::MeshIndexType} */
template<> struct MAGNUM_EXPORT ConfigurationValue<Magnum::MeshIndexType> {
    ConfigurationValue() = delete;

    /**
     * @brief Write enum value as string
     *
     * If the value is invalid, returns empty string.
     */
    static std::string toString(Magnum::MeshIndexType value, ConfigurationValueFlags);

    /**
     * @brief Read enum value as string
     *
     * If the value is invalid, returns a zero (invalid) type.
     */
    static Magnum::MeshIndexType fromString(const std::string& stringValue, ConfigurationValueFlags);
};

}}

#endif
