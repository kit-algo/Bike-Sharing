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

#include "../../../Helpers/Types.h"
#include "../../../Helpers/Vector/Vector.h"

#include "../../../DataStructures/RAPTOR/Data.h"
#include "../../../DataStructures/Intermediate/Data.h"
#include "../../../DataStructures/Container/Set.h"
#include "../../../DataStructures/Container/Map.h"
#include "../../../DataStructures/Container/ExternalKHeap.h"
#include "../../../DataStructures/BikeSharing/Data.h"

namespace RAPTOR {

template<bool USE_REACHABILITY_FLAGS = false, bool DEBUG = false, bool TRIP_PRUNING = false>
class OperatorDependentRAPTOR {

// index ranking for multi dimensional vectors: roundID > operatorID > stopID
// roundID == number of public transit vehicles used

public:
    static constexpr bool UseReachabilityFlags = USE_REACHABILITY_FLAGS;
    static constexpr bool Debug = DEBUG;
    static constexpr bool TripPruning = TRIP_PRUNING;
    using Type = OperatorDependentRAPTOR<UseReachabilityFlags, Debug, TripPruning>;

public:
    struct EarliestArrivalLabel {
        EarliestArrivalLabel() : arrivalTime(never), parentInfo(never), parent(noStop), routeId(noRouteId) {}
        static inline size_t UpdateCount{0};
        int arrivalTime;
        int parentInfo;
        StopId parent;
        RouteId routeId;
        inline void update(const int at, const int pi, const StopId p, const RouteId r) noexcept {
            if constexpr (Debug) UpdateCount++;
            arrivalTime = at;
            parentInfo = pi;
            parent = p;
            routeId = r;
        }
    };
    using Round = std::vector<std::vector<EarliestArrivalLabel>>;

    struct DijkstraLabel : public ExternalKHeapElement {
        DijkstraLabel(const uint32_t bikeOperator = -1) : arrivalTime(never), originStop(noStop), originOperator(-1), timeStamp(1), bikeOperator(bikeOperator) {}
        int arrivalTime;
        StopId originStop;
        int originOperator;
        int timeStamp;
        const uint32_t bikeOperator;
        inline void update(const int at, const StopId os, const int oo, ExternalKHeap<2, DijkstraLabel>& q) noexcept {
            arrivalTime = at;
            originStop = os;
            originOperator = oo;
            q.update(this);
        }
        inline bool hasSmallerKey(const DijkstraLabel* const other) const noexcept {
            return arrivalTime < other->arrivalTime;
        }
    };

    struct TargetLabel : public ExternalKHeapElement {
        TargetLabel() : earliestArrival(never) {}
        int earliestArrival;
        inline bool hasSmallerKey(const TargetLabel* const other) const noexcept {
            return earliestArrival > other->earliestArrival;
        }
    };

public:
    OperatorDependentRAPTOR(const BikeSharing::Data& data) :
        data(data),
        earliestArrivalPerOperator(data.numberOfOperators(),  std::vector<int>(data.numberOfStops() + 2, never)),
        finalTransferPerOperator(data.numberOfOperators(), std::vector<int>(data.numberOfStops() + 2, never)),
        updatedStopsPerOperator(data.numberOfOperators(), data.numberOfStops()),
        sourceStop(noStop),
        targetStop(noStop),
        sourceDepartureTime(intMax) {
        for (size_t bikeOperator = 0; bikeOperator < data.numberOfOperators(); bikeOperator++) {
            labelPerOperator.emplace_back(std::vector<DijkstraLabel>(data.walkingGraph.numVertices(), bikeOperator));
        }
        routesServingUpdatedStopsPerOperator.emplace_back(data.walkingNetwork.numberOfRoutes());
        routesServingUpdatedStopsPerOperator.resize(data.numberOfOperators(), data.cyclingNetwork.numberOfRoutes());
        if constexpr (TripPruning) {
            reachableRoutesOfOperator.resize(data.numberOfOperators());
            for (size_t bikeOperator = 1; bikeOperator < data.numberOfOperators(); bikeOperator++) {
                for (const std::vector<bool>& reachableTrips : data.reachableTripsOfOperator[bikeOperator]) {
                    reachableRoutesOfOperator[bikeOperator].emplace_back(false);
                    for (const bool trip : reachableTrips) {
                        if (trip) {
                            reachableRoutesOfOperator[bikeOperator].back() = true;
                            break;
                        }
                    }
                }
            }
        }
    }

    inline void run(const Vertex source, const int departureTime, const Vertex target = noStop) noexcept {
        clear();
        initialize(source, departureTime, target);
        runRounds();
    }

    inline int getEarliestArrivalTime() const noexcept {
        return earliestArrivalPerOperator[0][targetStop];
    }

    inline int getEarliestArrivalNumerOfTrips() const noexcept {
        const int eat = earliestArrivalPerOperator[0][targetStop];
        for (size_t i = rounds.size() - 1; i < rounds.size(); i--) {
            if (rounds[i][0][targetStop].arrivalTime == eat) return i;
        }
        return -1;
    }

    inline void debug(const double f = 1.0) const noexcept {
        std::cout << "Number of scanned routes: " << String::prettyDouble(RouteCount / f, 0) << std::endl;
        std::cout << "Number of settled vertices: " << String::prettyDouble(VertexCount / f, 0) << std::endl;
        std::cout << "Number of rounds: " << String::prettyDouble(RoundCount / f, 2) << std::endl;
        EarliestArrivalLabel::UpdateCount = 0;
        VertexCount = 0;
        RouteCount = 0;
        RoundCount = 0;
    }

private:
    inline void clear() noexcept {
        sourceStop = noStop;
        targetStop = noStop;
        queue.clear();
        rounds.clear();
        for (std::vector<int>& earliestArrival : earliestArrivalPerOperator) {
            std::fill(earliestArrival.begin(), earliestArrival.end(), never);
        }
        for (std::vector<int>& finalTransfer : finalTransferPerOperator) {
            std::fill(finalTransfer.begin(), finalTransfer.end(), never);
        }
        for (IndexedSet<false, StopId>& updatedStops : updatedStopsPerOperator) {
            updatedStops.clear();
        }
        for (IndexedMap<StopIndex, false, RouteId>& routesServingUpdatedStops : routesServingUpdatedStopsPerOperator) {
            routesServingUpdatedStops.clear();
        }
    }

    inline void initialize(const Vertex source, const int departureTime, const Vertex target) noexcept {
        sourceStop = (data.isStop(source)) ? (StopId(source)) : (StopId(data.numberOfStops()));
        targetStop = (data.isStop(target)) ? (StopId(target)) : (StopId(data.numberOfStops() + 1));
        sourceDepartureTime = departureTime;
        scanFinalTransfers(target);
        startNewRound();
        scanInitialTransfers(source);
    }

    inline void runRounds() noexcept {
        while (rounds.size() < 50) {
            startNewRound();
            collectRoutesServingUpdatedStop();
            scanRoutes();
            if (queue.empty()) break;
            scanIntermediateTransfers();
        }
        if constexpr (Debug) RoundCount += rounds.size();
    }

    // RAPTOR route collection begin
    inline void collectRoutesServingUpdatedStop() noexcept {
        collectRoutesServingUpdatedStop(data.walkingNetwork, 0);
        for (size_t bikeOperator = 1; bikeOperator < data.numberOfOperators(); bikeOperator++) {
            collectRoutesServingUpdatedStop(data.cyclingNetwork, bikeOperator);
        }
    }

    inline void collectRoutesServingUpdatedStop(const RAPTOR::Data& network, const uint32_t bikeOperator) noexcept {
        IndexedMap<StopIndex, false, RouteId>& routesServingUpdatedStops = routesServingUpdatedStopsPerOperator[bikeOperator];
        for (const StopId stop : updatedStopsPerOperator[bikeOperator]) {
            for (const RouteSegment& route : network.routesContainingStop(stop)) {
                AssertMsg(network.isRoute(route.routeId), "Route " << route.routeId << " is out of range!");
                AssertMsg(network.stopIds[network.firstStopIdOfRoute[route.routeId] + route.stopIndex] == stop, "RAPTOR network contains invalid route segments!");
                if (route.stopIndex + 1 == network.numberOfStopsInRoute(route.routeId)) continue;
                if (network.lastTripOfRoute(route.routeId)[route.stopIndex].departureTime < earliestArrivalPerOperator[bikeOperator][stop]) continue;
                if constexpr (TripPruning) {
                    if ((bikeOperator > 0) && (!reachableRoutesOfOperator[bikeOperator][route.routeId])) continue;
                }
                if (routesServingUpdatedStops.contains(route.routeId)) {
                    routesServingUpdatedStops[route.routeId] = std::min(routesServingUpdatedStops[route.routeId], route.stopIndex);
                } else {
                    routesServingUpdatedStops.insert(route.routeId, route.stopIndex);
                }
            }
        }
    }
    // RAPTOR route collection end

    // RAPTOR route scanning begin
    // One scanning pass per operator
    inline void scanRoutes() noexcept {
        AssertMsg(queue.empty(), "The queue still contains labels from the last round!");
        scanRoutes(data.walkingNetwork, 0);
        for (size_t bikeOperator = 1; bikeOperator < data.numberOfOperators(); bikeOperator++) {
            scanRoutes(data.cyclingNetwork, bikeOperator);
        }
    }

    inline void scanRoutes(const RAPTOR::Data& network, const uint32_t bikeOperator) noexcept {
        updatedStopsPerOperator[bikeOperator].clear();
        const std::vector<EarliestArrivalLabel>& previousRound = rounds[rounds.size() - 2][bikeOperator];
        std::vector<EarliestArrivalLabel>& currentRound  = rounds[rounds.size() - 1][bikeOperator];
        for (const RouteId route : routesServingUpdatedStopsPerOperator[bikeOperator].getKeys()) {
            if constexpr (Debug) RouteCount++;
            StopIndex stopIndex = routesServingUpdatedStopsPerOperator[bikeOperator][route];
            const size_t tripSize = network.numberOfStopsInRoute(route);
            AssertMsg(stopIndex < tripSize - 1, "Cannot scan a route starting at/after the last stop (Route: " << route << ", StopIndex: " << stopIndex << ", TripSize: " << tripSize << ")!");

            const StopId* stops = network.stopArrayOfRoute(route);
            const StopEvent* trip = network.lastTripOfRoute(route);
            StopId stop = stops[stopIndex];
            AssertMsg(trip[stopIndex].departureTime >= previousRound[stop].arrivalTime, "Cannot scan a route after the last trip has departed (Route: " << route << ", Stop: " << stop << ", StopIndex: " << stopIndex << ", Time: " << previousRound[stop].arrivalTime << ", LastDaparture: " << trip[stopIndex].departureTime << ")!");

            StopIndex parentIndex = stopIndex;
            const StopEvent* firstTrip = network.firstTripOfRoute(route);
            while (stopIndex < tripSize - 1) {
                while ((trip > firstTrip) && ((trip - tripSize + stopIndex)->departureTime >= previousRound[stop].arrivalTime)) {
                    trip -= tripSize;
                    parentIndex = stopIndex;
                }
                stopIndex++;
                stop = stops[stopIndex];
                if (improveEarliestArrival(bikeOperator, stop, trip[stopIndex].arrivalTime)) {
                    currentRound[stop].update(trip[stopIndex].arrivalTime, trip[parentIndex].departureTime, stops[parentIndex], route);
                    updatedStopsPerOperator[bikeOperator].insert(stop);
                    getLabel(stop, bikeOperator).update(trip[stopIndex].arrivalTime, stop, bikeOperator, queue);
                }
            }
        }
        routesServingUpdatedStopsPerOperator[bikeOperator].clear();
    }
    // RAPTOR route scanning end

    inline bool targetPruning(const int time) noexcept {
        return earliestArrivalPerOperator[0][targetStop] <= time;
    }

    inline bool improveEarliestArrival(const uint32_t bikeOperator, const StopId stop, const int time) noexcept {
        if (targetPruning(time)) return false;
        if constexpr (UseReachabilityFlags) {
            if (!data.reachableVerticesOfOperator[bikeOperator][stop]) return false;
        }
        if (earliestArrivalPerOperator[bikeOperator][stop] > time) {
            earliestArrivalPerOperator[bikeOperator][stop] = time;
            return true;
        } else {
            return false;
        }
    }

    inline void startNewRound() noexcept {
        rounds.emplace_back(data.numberOfOperators(), std::vector<EarliestArrivalLabel>(data.numberOfStops() + 2));
    }

    inline void scanInitialTransfers(const Vertex source) noexcept {
        timeStamp++;
        queue.clear();
        getLabel(source, 0).update(sourceDepartureTime, sourceStop, 0, queue);
        scanIntermediateTransfers();
    }

    inline void scanIntermediateTransfers() noexcept {
        Round& currentRound  = rounds[rounds.size() - 1];
        scanTransfers(data.walkingCoreCH.forward, data.cyclingNetwork.transferGraph, [&](const StopId stop, const DijkstraLabel& stopLabel){
            if (improveEarliestArrival(stopLabel.bikeOperator, stop, stopLabel.arrivalTime)) {
                currentRound[stopLabel.bikeOperator][stop].update(stopLabel.arrivalTime, stopLabel.originOperator, stopLabel.originStop, noRouteId);
                updatedStopsPerOperator[stopLabel.bikeOperator].insert(stop);
            }
            if (finalTransferPerOperator[stopLabel.bikeOperator][stop] >=  never) return;
            if (improveEarliestArrival(stopLabel.bikeOperator, targetStop, stopLabel.arrivalTime + finalTransferPerOperator[stopLabel.bikeOperator][stop])) {
                currentRound[0][targetStop].update(stopLabel.arrivalTime + finalTransferPerOperator[stopLabel.bikeOperator][stop], stopLabel.originOperator, stopLabel.originStop, noRouteId);
            }
        });
    }

    inline void scanFinalTransfers(const Vertex target) noexcept {
        timeStamp++;
        queue.clear();
        getLabel(target, 0).update(0, targetStop, 0, queue);
        scanTransfers(data.walkingCoreCH.backward, data.cyclingNetwork.transferGraph, [&](const StopId stop, const DijkstraLabel& stopLabel){
            finalTransferPerOperator[stopLabel.bikeOperator][stop] = stopLabel.arrivalTime;
        });
    }

    // Dijkstra's Algorithm begin
    // One queue for all labels of all operators
    template<typename WALKING_GRAPH, typename CYCLING_GRAPH, typename SETTLE>
    inline void scanTransfers(const WALKING_GRAPH& walkingGraph, const CYCLING_GRAPH& cyclingGraph, const SETTLE& settle) noexcept {
        while (!queue.empty()) {
            if constexpr (Debug) VertexCount++;
            const DijkstraLabel& currentLabel = *(queue.extractFront());
            const Vertex currentVertex = getVertex(currentLabel);
            if (currentLabel.bikeOperator == 0) {
                relaxEdges(currentLabel, 0, walkingGraph);
                if (data.operatorOfVertex[currentVertex] > 0) {
                    DijkstraLabel& pickUpLabel = getLabel(currentVertex, data.operatorOfVertex[currentVertex]);
                    const int arrivalTime = currentLabel.arrivalTime + data.pickUpTimeOfVertex[currentVertex];
                    if (pickUpLabel.arrivalTime > arrivalTime) {
                        pickUpLabel.update(arrivalTime, currentLabel.originStop, currentLabel.originOperator, queue);
                    }
                }
            } else {
                relaxEdges(currentLabel, currentLabel.bikeOperator, cyclingGraph);
                if (currentLabel.bikeOperator == data.operatorOfVertex[currentVertex]) {
                    DijkstraLabel& dropOffLabel = getLabel(currentVertex, 0);
                    const int arrivalTime = currentLabel.arrivalTime + data.dropOffTimeOfVertex[currentVertex];
                    if (dropOffLabel.arrivalTime > arrivalTime) {
                        dropOffLabel.update(arrivalTime, currentLabel.originStop, currentLabel.originOperator, queue);
                    }
                }
            }
            if (data.isStop(currentVertex)) {
                settle(StopId(currentVertex), currentLabel);
            }
        }
    }

    template<typename GRAPH>
    inline void relaxEdges(const DijkstraLabel& currentLabel, const uint32_t bikeOperator, const GRAPH& graph) noexcept {
        const Vertex currentVertex = getVertex(currentLabel);
        for (const Edge edge : graph.edgesFrom(currentVertex)) {
            const Vertex nextVertex = graph.get(ToVertex, edge);
            if constexpr (UseReachabilityFlags) {
                if (!data.reachableVerticesOfOperator[bikeOperator][nextVertex]) continue;
            }
            DijkstraLabel& nextLabel = getLabel(nextVertex, bikeOperator);
            const int arrivalTime = currentLabel.arrivalTime + getTravelTimeOrWeight(graph, edge);
            if (targetPruning(arrivalTime)) continue;
            if (nextLabel.arrivalTime > arrivalTime) {
                nextLabel.update(arrivalTime, currentLabel.originStop, currentLabel.originOperator, queue);
            }
        }
    }
    // Dijkstra's Algorithm end

    template<typename GRAPH>
    inline static int getTravelTimeOrWeight(const GRAPH& graph, const Edge edge) noexcept {
        if constexpr (GRAPH::HasEdgeAttribute(TravelTime)) {
            return graph.get(TravelTime, edge);;
        } else {
            return graph.get(Weight, edge);;
        }
    }

    inline DijkstraLabel& getLabel(const Vertex& vertex, const uint32_t bikeOperator) noexcept {
        DijkstraLabel& result = labelPerOperator[bikeOperator][vertex];
        result.arrivalTime = (result.timeStamp < timeStamp) ? (never) : (result.arrivalTime);
        result.timeStamp = timeStamp;
        return result;
    }

    inline Vertex getVertex(const DijkstraLabel& currentLabel) noexcept {
        return Vertex(&currentLabel - &(labelPerOperator[currentLabel.bikeOperator][0]));
    }

private:
    const BikeSharing::Data& data;

    std::vector<Round> rounds;                                                                  // rounds[roundID][operatorID][stopID]

    std::vector<std::vector<int>> earliestArrivalPerOperator;                                   // earliestArrivalPerOperator[operatorID][stopID]
    std::vector<std::vector<int>> finalTransferPerOperator;                                     // finalTransferPerOperator[operatorID][stopID]

    std::vector<IndexedSet<false, StopId>> updatedStopsPerOperator;                             // updatedStopsPerOperator[operatorID]
    std::vector<IndexedMap<StopIndex, false, RouteId>> routesServingUpdatedStopsPerOperator;    // routesServingUpdatedStopsPerOperator[operatorID]

    StopId sourceStop;
    StopId targetStop;
    int sourceDepartureTime;

    std::vector<std::vector<DijkstraLabel>> labelPerOperator;                                   // labelPerOperator[operatorID][stopID]
    ExternalKHeap<2, DijkstraLabel> queue;
    int timeStamp;

    std::vector<std::vector<bool>> reachableRoutesOfOperator;

    static inline size_t VertexCount{0};
    static inline size_t RouteCount{0};
    static inline size_t RoundCount{0};

};

}
