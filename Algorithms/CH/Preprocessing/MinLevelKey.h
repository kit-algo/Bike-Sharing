/**********************************************************************************

 Copyright (c) 2020 Tobias Zündorf

 MIT License

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation
 files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy,
 modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
 is furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
 IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

**********************************************************************************/

#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "CHData.h"
#include "KeyFunction.h"
#include "../../PathFinder.h"

namespace CH {

template<typename WITNESS_SEARCH, typename KEY_FUNCTION>
class MinLevelKey {

public:
    using WitnessSearch = WITNESS_SEARCH;
    using KeyFunction = KEY_FUNCTION;
    using Type = MinLevelKey<WitnessSearch, KeyFunction>;

    using KeyType = typename KeyFunction::KeyType;
    constexpr static int offset = (1 << 30) - 1;

public:
    MinLevelKey(const std::vector<uint16_t>& minLevel, const KeyFunction& keyFunction = KeyFunction()) :
        data(nullptr),
        keyFunction(keyFunction),
        minLevel(minLevel) {
    }

    inline KeyType operator() (Vertex vertex) {
        int key = keyFunction(vertex);
        if (data->level[vertex] < minLevel[vertex]) {
            key = std::min(key, offset);
            key += offset;
        }
        return key;
    }

    template<typename T> inline void update(T&) noexcept {}

    inline void initialize(const Data* data, WitnessSearch* witnessSearch) noexcept {
        this->data = data;
        keyFunction.initialize(data, witnessSearch);
    }

private:
    const Data* data;
    KeyFunction keyFunction;
    std::vector<uint16_t> minLevel;

};

}
