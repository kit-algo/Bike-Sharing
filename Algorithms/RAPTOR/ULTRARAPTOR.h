/**********************************************************************************

 Copyright (c) 2020 Jonas Sauer, Tobias Zündorf

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

#include "InitialTransfers.h"

#include "../../DataStructures/RAPTOR/Data.h"
#include "../../DataStructures/RAPTOR/Entities/ArrivalLabel.h"
#include "../../DataStructures/RAPTOR/Entities/EarliestArrivalTime.h"
#include "../../DataStructures/Container/Set.h"
#include "../../DataStructures/Container/Map.h"

#include "Debugger.h"

namespace RAPTOR {

template<bool TARGET_PRUNING, typename INITIAL_TRANSFERS = BucketCHInitialTransfers, typename DEBUGGER = NoDebugger, bool USE_MIN_TRANSFER_TIMES = false, bool PREVENT_DIRECT_WALKING = false>
class ULTRARAPTOR {

public:
    static constexpr bool TargetPruning = TARGET_PRUNING;
    using InitialTransferType = INITIAL_TRANSFERS;
    using InitialTransferGraph = typename InitialTransferType::Graph;
    using Debugger = DEBUGGER;
    static constexpr bool UseMinTransferTimes = USE_MIN_TRANSFER_TIMES;
    static constexpr bool PreventDirectWalking = PREVENT_DIRECT_WALKING;
    static constexpr bool SeparateRouteAndTransferEntries = UseMinTransferTimes | PreventDirectWalking;
    static constexpr int RoundFactor = SeparateRouteAndTransferEntries ? 2 : 1;
    using ArrivalTime = EarliestArrivalTime<SeparateRouteAndTransferEntries>;
    using Type = ULTRARAPTOR<TargetPruning, InitialTransferType, Debugger, UseMinTransferTimes, PreventDirectWalking>;
    using SourceType = Vertex;

private:
    struct EarliestArrivalLabel {
        EarliestArrivalLabel() : arrivalTime(never), parentDepartureTime(never), parent(noVertex), usesRoute(false), routeId(noRouteId) {}
        int arrivalTime;
        int parentDepartureTime;
        Vertex parent;
        bool usesRoute;
        union {
            RouteId routeId;
            Edge transferId;
        };
    };
    using Round = std::vector<EarliestArrivalLabel>;

public:
    template<typename ATTRIBUTE>
    ULTRARAPTOR(const Data& data, const InitialTransferGraph& forwardGraph, const InitialTransferGraph& backwardGraph, const ATTRIBUTE weight, const Debugger& debuggerTemplate = Debugger()) :
        data(data),
        initialTransfers(forwardGraph, backwardGraph, data.numberOfStops(), weight),
        earliestArrival(data.numberOfStops() + 1),
        stopsUpdatedByRoute(data.numberOfStops() + 1),
        stopsUpdatedByTransfer(data.numberOfStops() + 1),
        routesServingUpdatedStops(data.numberOfRoutes()),
        sourceVertex(noVertex),
        targetVertex(noVertex),
        targetStop(noStop),
        sourceDepartureTime(never),
        debugger(debuggerTemplate) {
        if constexpr (UseMinTransferTimes) {
            AssertMsg(!data.hasImplicitBufferTimes(), "Either min transfer times have to be used OR departure buffer times have to be implicit!");
        } else {
            AssertMsg(data.hasImplicitBufferTimes(), "Either min transfer times have to be used OR departure buffer times have to be implicit!");
        }
        debugger.initialize(data);
    }

    template<typename = std::enable_if_t<Meta::Equals<CHGraph, InitialTransferGraph>()>>
    ULTRARAPTOR(const Data& data, const CH::CH& chData, const Debugger& debuggerTemplate = Debugger()) :
        ULTRARAPTOR(data, chData.forward, chData.backward, Weight, debuggerTemplate) {
    }

    template<typename = std::enable_if_t<Meta::Equals<TransferGraph, InitialTransferGraph>()>>
    ULTRARAPTOR(const Data& data, const TransferGraph& forwardGraph, const TransferGraph& backwardGraph, const Debugger& debuggerTemplate = Debugger()) :
        ULTRARAPTOR(data, forwardGraph, backwardGraph, TravelTime, debuggerTemplate) {
    }

    inline void run(const Vertex source, const int departureTime, const Vertex target, const size_t maxRounds = 50) noexcept {
        debugger.start();
        debugger.startInitialization();
        clear();
        initialize(source, departureTime, target);
        debugger.doneInitialization();
        relaxInitialTransfers(departureTime);
        for (size_t i = 0; i < maxRounds; i++) {
            debugger.newRound();
            startNewRound();
            collectRoutesServingUpdatedStops();
            scanRoutes();
            if (stopsUpdatedByRoute.empty()) break;
            if constexpr (SeparateRouteAndTransferEntries) startNewRound();
            relaxIntermediateTransfers();
        }
        debugger.done();
    }

    inline std::vector<ArrivalLabel> getArrivals() const noexcept {
        return getArrivals(targetStop);
    }

    inline std::vector<ArrivalLabel> getArrivals(const Vertex vertex) const noexcept {
        const StopId target = (vertex == targetVertex) ? (targetStop) : (StopId(vertex));
        std::vector<ArrivalLabel> labels;
        for (size_t i = 0; i < rounds.size(); i += RoundFactor) {
            getArrival(labels, i, target);
        }
        return labels;
    }

    inline std::vector<int> getArrivalTimes() const noexcept {
        return getArrivalTimes(targetStop);
    }

    inline std::vector<int> getArrivalTimes(const Vertex vertex) const noexcept {
        const StopId target = (vertex == targetVertex) ? (targetStop) : (StopId(vertex));
        std::vector<int> arrivalTimes;
        for (size_t i = 0; i < rounds.size(); i += RoundFactor) {
            getArrivalTime(arrivalTimes, i, target);
        }
        return arrivalTimes;
    }

    inline bool reachable(const Vertex vertex) const noexcept {
        const StopId target = (vertex == targetVertex) ? (targetStop) : (StopId(vertex));
        return earliestArrival[target].getArrivalTime() < never;
    }

    inline int getEarliestArrivalTime(const Vertex vertex) const noexcept {
        const StopId target = (vertex == targetVertex) ? (targetStop) : (StopId(vertex));
        return earliestArrival[target].getArrivalTime();
    }

    inline int getEarliestArrivalTime() const noexcept {
        return earliestArrival[targetStop].getArrivalTime();
    }

    inline int getEarliestArrivalNumerOfTrips() const noexcept {
        const int eat = earliestArrival[targetStop].getArrivalTime();
        for (size_t i = rounds.size() - 1; i < rounds.size(); i--) {
            if (rounds[i][targetStop].arrivalTime == eat) return i;
        }
        return -1;
    }

    inline int getWalkingArrivalTime() const noexcept {
        return sourceDepartureTime + initialTransfers.getDistance();
    }

    inline int getWalkingArrivalTime(const Vertex vertex) const noexcept {
        return sourceDepartureTime + initialTransfers.getForwardDistance(vertex);
    }

    inline int getWalkingTravelTime() const noexcept {
        return initialTransfers.getDistance();
    }

    inline int getWalkingTravelTime(const Vertex vertex) const noexcept {
        return initialTransfers.getDistance(vertex);
    }

    inline int getDirectTransferTime() const noexcept {
        return initialTransfers.getDistance();
    }

    inline const Debugger& getDebugger() const noexcept {
        return debugger;
    }

    inline Debugger& getDebugger() noexcept {
        return debugger;
    }

    inline int getArrivalTime(const Vertex vertex, const size_t numberOfTrips) const noexcept {
        const StopId target = (vertex == targetVertex) ? (targetStop) : (StopId(vertex));
        size_t round = numberOfTrips * RoundFactor;
        if constexpr (SeparateRouteAndTransferEntries) {
            if ((round + 1 < rounds.size()) && (rounds[round + 1][target].arrivalTime < rounds[round][target].arrivalTime)) round++;
        }
        AssertMsg(rounds[round][target].arrivalTime < never, "No label found for stop " << target << " in round " << round << "!");
        return rounds[round][target].arrivalTime;
    }

    template<bool RESET_CAPACITIES = false>
    inline void clear() noexcept {
        stopsUpdatedByRoute.clear();
        stopsUpdatedByTransfer.clear();
        routesServingUpdatedStops.clear();
        targetStop = StopId(data.numberOfStops());
        sourceDepartureTime = never;
        if constexpr (RESET_CAPACITIES) {
            std::vector<Round>().swap(rounds);
            std::vector<int>(earliestArrival.size(), never).swap(earliestArrival);
        } else {
            rounds.clear();
            Vector::fill(earliestArrival);
        }
    }

    inline void reset() noexcept {
        clear<true>();
    }

private:
    inline void initialize(const Vertex source, const int departureTime, const Vertex target) noexcept {
        sourceVertex = source;
        targetVertex = target;
        if (data.isStop(target)) {
            targetStop = StopId(target);
        }
        sourceDepartureTime = departureTime;
        startNewRound();
        if (data.isStop(source)) {
            debugger.updateStopByRoute(StopId(source), sourceDepartureTime);
            arrivalByRoute(StopId(source), sourceDepartureTime);
            currentRound()[source].parent = source;
            currentRound()[source].parentDepartureTime = sourceDepartureTime;
            currentRound()[source].usesRoute = false;
            if constexpr (!SeparateRouteAndTransferEntries) stopsUpdatedByTransfer.insert(StopId(source));
        }
        if constexpr (SeparateRouteAndTransferEntries) startNewRound();
    }

    inline void collectRoutesServingUpdatedStops() noexcept {
        debugger.startCollectRoutes();
        for (const StopId stop : stopsUpdatedByTransfer) {
            AssertMsg(data.isStop(stop), "Stop " << stop << " is out of range!");
            const int arrivalTime = previousRound()[stop].arrivalTime;
            AssertMsg(arrivalTime < never, "Updated stop has arrival time = never!");
            for (const RouteSegment& route : data.routesContainingStop(stop)) {
                AssertMsg(data.isRoute(route.routeId), "Route " << route.routeId << " is out of range!");
                AssertMsg(data.stopIds[data.firstStopIdOfRoute[route.routeId] + route.stopIndex] == stop, "RAPTOR data contains invalid route segments!");
                if (route.stopIndex + 1 == data.numberOfStopsInRoute(route.routeId)) continue;
                if (data.lastTripOfRoute(route.routeId)[route.stopIndex].departureTime < arrivalTime) continue;
                if (routesServingUpdatedStops.contains(route.routeId)) {
                    routesServingUpdatedStops[route.routeId] = std::min(routesServingUpdatedStops[route.routeId], route.stopIndex);
                } else {
                    routesServingUpdatedStops.insert(route.routeId, route.stopIndex);
                }
            }
        }
        debugger.stopCollectRoutes();
    }

    inline void scanRoutes() noexcept {
        debugger.startScanRoutes();
        stopsUpdatedByRoute.clear();
        for (const RouteId route : routesServingUpdatedStops.getKeys()) {
            debugger.scanRoute(route);
            StopIndex stopIndex = routesServingUpdatedStops[route];
            const size_t tripSize = data.numberOfStopsInRoute(route);
            AssertMsg(stopIndex < tripSize - 1, "Cannot scan a route starting at/after the last stop (Route: " << route << ", StopIndex: " << stopIndex << ", TripSize: " << tripSize << ")!");

            const StopId* stops = data.stopArrayOfRoute(route);
            const StopEvent* trip = data.lastTripOfRoute(route);
            StopId stop = stops[stopIndex];
            AssertMsg(trip[stopIndex].departureTime >= previousRound()[stop].arrivalTime, "Cannot scan a route after the last trip has departed (Route: " << route << ", Stop: " << stop << ", StopIndex: " << stopIndex << ", Time: " << previousRound()[stop].arrivalTime << ", LastDeparture: " << trip[stopIndex].departureTime << ")!");

            StopIndex parentIndex = stopIndex;
            const StopEvent* firstTrip = data.firstTripOfRoute(route);
            while (stopIndex < tripSize - 1) {
                while ((trip > firstTrip) && ((trip - tripSize + stopIndex)->departureTime >= previousRound()[stop].arrivalTime)) {
                    trip -= tripSize;
                    parentIndex = stopIndex;
                }
                stopIndex++;
                stop = stops[stopIndex];
                debugger.scanRouteSegment(data.getRouteSegmentNum(route, stopIndex));
                if (arrivalByRoute(stop, trip[stopIndex].arrivalTime)) {
                    EarliestArrivalLabel& label = currentRound()[stop];
                    label.parent = stops[parentIndex];
                    label.parentDepartureTime = trip[parentIndex].departureTime;
                    label.usesRoute = true;
                    label.routeId = route;
                }
            }
        }
        debugger.stopScanRoutes();
    }

    inline void relaxInitialTransfers(const int sourceDepartureTime) noexcept {
        debugger.startRelaxTransfers();
        initialTransfers.template run<!PreventDirectWalking>(sourceVertex, targetVertex);
        debugger.directWalking(initialTransfers.getDistance());
        for (const Vertex stop : initialTransfers.getForwardPOIs()) {
            if (stop == targetStop) continue;
            AssertMsg(data.isStop(stop), "Reached POI " << stop << " is not a stop!");
            AssertMsg(initialTransfers.getForwardDistance(stop) != INFTY, "Vertex " << stop << " was not reached!");
            const int arrivalTime = sourceDepartureTime + initialTransfers.getForwardDistance(stop);
            if (arrivalByTransfer(StopId(stop), arrivalTime)) {
                debugger.updateStopByTransfer(StopId(stop), arrivalTime);
                EarliestArrivalLabel& label = currentRound()[stop];
                label.parent = sourceVertex;
                label.parentDepartureTime = sourceDepartureTime;
                label.usesRoute = false;
                label.transferId = noEdge;
            }
        }
        if constexpr (!PreventDirectWalking) {
            if (initialTransfers.getDistance() != INFTY) {
                const int arrivalTime = sourceDepartureTime + initialTransfers.getDistance();
                if (arrivalByTransfer(targetStop, arrivalTime)) {
                    debugger.updateStopByTransfer(targetStop, arrivalTime);
                    EarliestArrivalLabel& label = currentRound()[targetStop];
                    label.parent = sourceVertex;
                    label.parentDepartureTime = sourceDepartureTime;
                    label.usesRoute = false;
                    label.transferId = noEdge;
                }
            }
        }
        debugger.stopRelaxTransfers();
    }

    inline void relaxIntermediateTransfers() noexcept {
        debugger.startRelaxTransfers();
        stopsUpdatedByTransfer.clear();
        routesServingUpdatedStops.clear();
        for (const StopId stop : stopsUpdatedByRoute) {
            const int earliestArrivalTime = SeparateRouteAndTransferEntries ? previousRound()[stop].arrivalTime : currentRound()[stop].arrivalTime;
            for (const Edge edge : data.transferGraph.edgesFrom(stop)) {
                const StopId toStop = StopId(data.transferGraph.get(ToVertex, edge));
                if (toStop == targetStop) continue;
                debugger.relaxEdge(edge);
                const int arrivalTime = earliestArrivalTime + data.transferGraph.get(TravelTime, edge);
                AssertMsg(data.isStop(data.transferGraph.get(ToVertex, edge)), "Graph contains edges to non stop vertices!");
                if (arrivalByTransfer(toStop, arrivalTime)) {
                    debugger.updateStopByTransfer(toStop, arrivalTime);
                    EarliestArrivalLabel& label = currentRound()[toStop];
                    label.parent = stop;
                    label.parentDepartureTime = earliestArrivalTime;
                    label.usesRoute = false;
                    label.transferId = edge;
                }
            }
            if (initialTransfers.getBackwardDistance(stop) != INFTY) {
                const int arrivalTime = earliestArrivalTime + initialTransfers.getBackwardDistance(stop);
                if (arrivalByTransfer(targetStop, arrivalTime)) {
                    debugger.updateStopByTransfer(targetStop, arrivalTime);
                    EarliestArrivalLabel& label = currentRound()[targetStop];
                    label.parent = stop;
                    label.parentDepartureTime = earliestArrivalTime;
                    label.usesRoute = false;
                    label.transferId = noEdge;
                }
            }
            if constexpr (SeparateRouteAndTransferEntries) {
                const int arrivalTime = earliestArrivalTime + getMinTransferTime(stop);
                if (arrivalByTransfer(stop, arrivalTime)) {
                    debugger.updateStopByTransfer(stop, arrivalTime);
                    EarliestArrivalLabel& label = currentRound()[stop];
                    label.parent = stop;
                    label.parentDepartureTime = earliestArrivalTime;
                    label.usesRoute = false;
                }
            } else {
                stopsUpdatedByTransfer.insert(stop);
            }
            debugger.settleVertex(stop);
        }
        debugger.stopRelaxTransfers();
    }

    inline int getMinTransferTime(const StopId stop) const noexcept {
        if constexpr (UseMinTransferTimes) {
            return data.stopData[stop].minTransferTime;
        } else {
            suppressUnusedParameterWarning(stop);
            return 0;
        }
    }

    inline Round& currentRound() noexcept {
        AssertMsg(!rounds.empty(), "Cannot return current round, because no round exists!");
        return rounds.back();
    }

    inline Round& previousRound() noexcept {
        AssertMsg(rounds.size() >= 2, "Cannot return previous round, because less than two rounds exist!");
        return rounds[rounds.size() - 2];
    }

    inline void startNewRound() noexcept {
        rounds.emplace_back(data.numberOfStops() + 1);
    }

    inline bool arrivalByRoute(const StopId stop, const int time) noexcept {
        AssertMsg(data.isStop(stop), "Stop " << stop << " is out of range!");
        if constexpr (TargetPruning) if (earliestArrival[targetStop].getArrivalTimeByRoute() <= time) return false;
        if (earliestArrival[stop].getArrivalTimeByRoute() <= time) return false;
        debugger.updateStopByRoute(stop, time);
        currentRound()[stop].arrivalTime = time;
        earliestArrival[stop].setArrivalTimeByRoute(time);
        stopsUpdatedByRoute.insert(stop);
        return true;
    }

    inline bool arrivalByTransfer(const StopId stop, const int time) noexcept {
        AssertMsg(data.isStop(stop) || stop == targetStop, "Stop " << stop << " is out of range!");
        if constexpr (TargetPruning) if (earliestArrival[targetStop].getArrivalTimeByTransfer() <= time) return false;
        if (earliestArrival[stop].getArrivalTimeByTransfer() <= time) return false;
        currentRound()[stop].arrivalTime = time;
        earliestArrival[stop].setArrivalTimeByTransfer(time);
        if (data.isStop(stop)) stopsUpdatedByTransfer.insert(stop);
        return true;
    }

    inline void getArrival(std::vector<ArrivalLabel>& labels, size_t round, const StopId stop) const noexcept {
        if constexpr (SeparateRouteAndTransferEntries) {
            if ((round + 1 < rounds.size()) && (rounds[round + 1][stop].arrivalTime < rounds[round][stop].arrivalTime)) round++;
        }
        if (rounds[round][stop].arrivalTime >= (labels.empty() ? never : labels.back().arrivalTime)) return;
        labels.emplace_back(rounds[round][stop].arrivalTime, round / RoundFactor);
    }

    inline void getArrivalTime(std::vector<int>& labels, size_t round, const StopId stop) const noexcept {
        if constexpr (SeparateRouteAndTransferEntries) {
            if ((round + 1 < rounds.size()) && (rounds[round + 1][stop].arrivalTime < rounds[round][stop].arrivalTime)) round++;
        }
        labels.emplace_back(std::min(rounds[round][stop].arrivalTime, (labels.empty()) ? (never) : (labels.back())));
    }

private:
    const Data& data;

    InitialTransferType initialTransfers;

    std::vector<Round> rounds;

    std::vector<ArrivalTime> earliestArrival;

    IndexedSet<false, StopId> stopsUpdatedByRoute;
    IndexedSet<false, StopId> stopsUpdatedByTransfer;
    IndexedMap<StopIndex, false, RouteId> routesServingUpdatedStops;

    Vertex sourceVertex;
    Vertex targetVertex;
    StopId targetStop;
    int sourceDepartureTime;

    Debugger debugger;

};

}
