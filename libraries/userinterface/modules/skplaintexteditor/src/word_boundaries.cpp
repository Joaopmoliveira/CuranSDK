// Copyright 2019 Google LLC.
// Use of this source code is governed by a BSD-style license that can be found in the LICENSE file.

#include "include/core/SkTypes.h"
#include "word_boundaries.h"
#include "modules/skunicode/include/SkUnicode.h"
#include "modules/skunicode/include/SkUnicode_icu.h"
#include <memory>

#if defined(SK_UNICODE_ICU_IMPLEMENTATION)
#include "modules/skunicode/include/SkUnicode_icu.h"
#endif

#if defined(SK_UNICODE_LIBGRAPHEME_IMPLEMENTATION)
#include "modules/skunicode/include/SkUnicode_libgrapheme.h"
#endif

#if defined(SK_UNICODE_ICU4X_IMPLEMENTATION)
#include "modules/skunicode/include/SkUnicode_icu4x.h"
#endif

std::vector<bool> GetUtf8WordBoundaries(const char* begin, size_t byteCount, const char* locale) {
    
    
    auto unicode = SkUnicodes::ICU::Make();
    if (nullptr == unicode) {
        return {};
    }
    std::vector<SkUnicode::Position> positions;
    if (!unicode->getWords(begin, byteCount, locale, &positions) || byteCount == 0) {
        return {};
    }
    std::vector<bool> result;
    result.resize(byteCount);
    for (auto& pos : positions) {
        result[pos] = true;
    }
    return result;
}
