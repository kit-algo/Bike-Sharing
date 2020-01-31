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

#include "RouteSegment.h"

#include "../../../Helpers/Types.h"
#include "../../../Helpers/IO/Serialization.h"

namespace RAPTOR {

class Transfer {

public:
    Transfer(const RouteId routeId = -1, const StopIndex stopIndex = -1, const size_t stopEventIndex = -1) :
        routeId(routeId),
        stopIndex(stopIndex),
        stopEventIndex(stopEventIndex) {
    }
    Transfer(IO::Deserialization& deserialize) {
        this->deserialize(deserialize);
    }

    friend std::ostream& operator<<(std::ostream& out, const Transfer& t) {
        return out << "Transfer{" << t.routeId << ", " << t.stopIndex << ", " << t.stopEventIndex << "}";
    }

    inline void serialize(IO::Serialization& serialize) const noexcept {
        serialize(routeId, stopIndex, stopEventIndex);
    }

    inline void deserialize(IO::Deserialization& deserialize) noexcept {
        deserialize(routeId, stopIndex, stopEventIndex);
    }

public:
    RouteId routeId{noRouteId};
    StopIndex stopIndex{noStopIndex};
    size_t stopEventIndex{(size_t)-1};

};

}

