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

#include <limits>
#include <vector>
#include <string>

#include "TaggedInteger.h"

template<int TAG, typename VALUE_TYPE>
using TaggedIntegerX = TaggedInteger<TAG, typename VALUE_TYPE::ValueType, VALUE_TYPE::InvalidValue, VALUE_TYPE::DefaultValue, VALUE_TYPE>;

using Vertex = TaggedInteger<0, u_int32_t, -u_int32_t(1)>;
constexpr Vertex noVertex(Vertex::InvalidValue);

using Edge = TaggedInteger<1, u_int32_t, -u_int32_t(1)>;
constexpr Edge noEdge(Edge::InvalidValue);

using StopId = DependentTaggedInteger<2, Vertex>;
constexpr StopId noStop(StopId::InvalidValue);

using RouteId = TaggedInteger<3, u_int32_t, -u_int32_t(1)>;
constexpr RouteId noRouteId(RouteId::InvalidValue);

using TripId = TaggedInteger<4, u_int32_t, -u_int32_t(1)>;
constexpr TripId noTripId(TripId::InvalidValue);

using StopIndex = TaggedInteger<5, u_int32_t, -u_int32_t(1)>;
constexpr StopIndex noStopIndex(StopIndex::InvalidValue);

using ConnectionId = TaggedInteger<6, u_int32_t, -u_int32_t(1)>;
constexpr ConnectionId noConnection(ConnectionId::InvalidValue);

inline constexpr int intMax = std::numeric_limits<int>::max();
inline constexpr double doubleMax = std::numeric_limits<double>::max();
inline constexpr int never = intMax;

inline constexpr int EARTH_RADIUS_IN_CENTIMETRE = 637813700;
inline constexpr double PI = 3.141592653589793;

inline constexpr int INFTY = std::numeric_limits<int>::max() / 2;
inline constexpr int DAY = 24 * 60 * 60;
inline constexpr int VOID = -32768;

inline constexpr int FORWARD = 0;
inline constexpr int BACKWARD = 1;

inline constexpr int DEPARTURE = 0;
inline constexpr int ARRIVAL = 1;

struct NO_OPERATION {
    template<typename... ARGS>
    constexpr inline bool operator() (ARGS...) const noexcept {return false;}
};

NO_OPERATION NoOperation;

namespace Vehicle {
    namespace Type {
        constexpr int Tram = 0;
        constexpr int Subway = 1;
        constexpr int Rail = 2;
        constexpr int Bus = 3;
        constexpr int Ferry = 4;
        constexpr int CableCar = 5;
        constexpr int Gondola = 6;
        constexpr int Funicular = 7;
        constexpr int Undefined = -1;
    }
}
