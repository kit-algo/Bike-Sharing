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

#include "OperatorDependentQueries.h"

#include "../Debugger.h"
#include "../ULTRARAPTOR.h"
#include "../InitialTransfers.h"

#include "../../../DataStructures/BikeSharing/ExtendedNetwork.h"

namespace RAPTOR {

class BikeSharingDebugger : public NoDebugger {

public:
    BikeSharingDebugger() :
        stopCount(0),
        vertexCount(0),
        routeCount(0),
        roundCount(0) {
    }

public:
    inline void start() noexcept {
        roundCount++;
    }

    inline void newRound() noexcept {
        roundCount++;
    }

    inline void scanRoute(const RouteId) noexcept {
        routeCount++;
    }

    inline void updateStopByRoute(const StopId, const int) noexcept {
        stopCount++;
    }

    inline void updateStopByTransfer(const StopId, const int) noexcept {
        stopCount++;
    }

    inline void settleVertex(const Vertex) noexcept {
        vertexCount++;
    }

    inline void printData(const double f = 1.0) noexcept {
        std::cout << "Number of scanned routes: " << String::prettyDouble(routeCount / f, 0) << std::endl;
        std::cout << "Number of settled vertices: " << String::prettyDouble(vertexCount / f, 0) << std::endl;
        std::cout << "Number of rounds: " << String::prettyDouble(roundCount / f, 2) << std::endl;
        stopCount = 0;
        vertexCount = 0;
        routeCount = 0;
        roundCount = 0;
    }

public:
    size_t stopCount;
    size_t vertexCount;
    size_t routeCount;
    size_t roundCount;

};

template<bool DEBUG = false>
class OperatorExpandedULTRA {

public:
    static constexpr bool Debug = DEBUG;
    using Type = OperatorExpandedULTRA<Debug>;

public:
    OperatorExpandedULTRA(const BikeSharing::ExtendedNetwork& extendedNetwork) :
        extendedNetwork(extendedNetwork),
        ultra(extendedNetwork.data, extendedNetwork.ch) {
    }

public:
    inline void run(const Vertex source, const int departureTime, const Vertex target) noexcept {
        ultra.run(extendedNetwork.oldToNewVertex[source], departureTime, extendedNetwork.oldToNewVertex[target]);
    }

    inline int getEarliestArrivalTime() const noexcept {
        return ultra.getEarliestArrivalTime();
    }

    inline int getEarliestArrivalNumerOfTrips() const noexcept {
        return ultra.getEarliestArrivalNumerOfTrips();
    }

    inline void debug(const double f = 1.0) noexcept {
        ultra.getDebugger().printData(f);
    }

private:
    const BikeSharing::ExtendedNetwork& extendedNetwork;

    ULTRARAPTOR<true, BucketCHInitialTransfers, BikeSharingDebugger> ultra;

};

template<bool DEBUG = false>
class OperatorExpandedOperatorDependentRAPTOR {

public:
    static constexpr bool Debug = DEBUG;
    using Type = OperatorExpandedULTRA<Debug>;

public:
    OperatorExpandedOperatorDependentRAPTOR(const BikeSharing::ExtendedNetwork& extendedNetwork) :
        extendedNetwork(extendedNetwork),
        extendedData(extendedNetwork.data, extendedNetwork.graph, extendedNetwork.coreCH),
        raptor(extendedData) {
    }

public:
    inline void run(const Vertex source, const int departureTime, const Vertex target) noexcept {
        raptor.run(extendedNetwork.oldToNewVertex[source], departureTime, extendedNetwork.oldToNewVertex[target]);
    }

    inline int getEarliestArrivalTime() const noexcept {
        return raptor.getEarliestArrivalTime();
    }

    inline int getEarliestArrivalNumerOfTrips() const noexcept {
        return raptor.getEarliestArrivalNumerOfTrips();
    }

    inline void debug(const double f = 1.0) noexcept {
        raptor.debug(f);
    }

    inline void printPath() const noexcept {
        raptor.printPath();
    }

private:
    const BikeSharing::ExtendedNetwork& extendedNetwork;
    const BikeSharing::Data extendedData;

    OperatorDependentRAPTOR<false, Debug, false> raptor;

};

}
