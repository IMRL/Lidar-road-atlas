#ifndef _FONT_H_
#define _FONT_H_

#if defined(_WIN32)
#define SYS_FONT "%WINDIR%\\fonts"
// #define NOTO_CJK ""
#elif defined(_APPLE_) && defined(_MACH_)
#define SYS_FONT "/System/Library/Fonts"
// #define NOTO_CJK ""
#elif defined(linux) || defined(__linux)
#define SYS_FONT "/usr/share/fonts"
#define NOTO_CJK "opentype/noto/NotoSansCJK-Regular.ttc"
#else
#define SYS_FONT "."
// #define NOTO_CJK ""
#endif

#endif
