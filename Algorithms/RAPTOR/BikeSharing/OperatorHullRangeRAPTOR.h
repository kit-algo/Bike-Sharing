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

#include <vector>
#include <string>
#include <iostream>
#include <iomanip>

#include "../../../Helpers/Types.h"
#include "../../../Helpers/Vector/Vector.h"
#include "../../../Helpers/String/String.h"

#include "../../../DataStructures/Container/ExternalKHeap.h"
#include "../../../DataStructures/Container/Map.h"
#include "../../../DataStructures/Container/Set.h"
#include "../../../DataStructures/RAPTOR/Data.h"

namespace BikeSharing {

template<bool DEBUG>
class OperatorHullRangeRAPTOR {

public:
    inline constexpr static bool Debug = DEBUG;
    using Type = OperatorHullRangeRAPTOR<Debug>;

private:
    struct RoundLabel : public ExternalKHeapElement {
        RoundLabel() : ExternalKHeapElement(), arrivalTime(INFTY), parentIndex(0), routeId(noRouteId), parentVertex(noVertex) {}
        inline bool hasSmallerKey(const RoundLabel* other) const {
            return arrivalTime < other->arrivalTime;
        }
        inline void update(const int at, const u_int32_t pi, const RouteId r, const Vertex pv, ExternalKHeap<2, RoundLabel>& q) noexcept {
            arrivalTime = at;
            parentIndex = pi;
            routeId = r;
            parentVertex = pv;
            q.update(this);
        }
        int arrivalTime;
        u_int32_t parentIndex;
        RouteId routeId;
        Vertex parentVertex;
    };

    using Round = std::vector<RoundLabel>;

    struct DepartureLabel {
        DepartureLabel(const int departureTime, const StopId departureStop) :
            departureTime(departureTime),
            departureStop(departureStop) {
        }
        inline bool operator<(const DepartureLabel& other) const noexcept {
            return departureTime > other.departureTime;
        }
        int departureTime;
        StopId departureStop;
    };

    struct DebugData {
        DebugData() : numberOfDepartures(0), numberOfScannedRoutes(0), numberOfSettledVertices(0), numberOfUsedVertices(0) {}
        inline void operator+=(const DebugData& other) noexcept {
            numberOfDepartures += other.numberOfDepartures;
            numberOfScannedRoutes += other.numberOfScannedRoutes;
            numberOfSettledVertices += other.numberOfSettledVertices;
            numberOfUsedVertices += other.numberOfUsedVertices;
        }
        inline static void PrintHeader() noexcept {
            std::cout << "Round";
            std::cout << std::setw(16) << "Departures";
            std::cout << std::setw(20) << "Scanned Routes";
            std::cout << std::setw(22) << "Settled Vertices";
            std::cout << std::setw(19) << "Used Vertices" << "\n";
        }
        inline void printData(const std::string& round) const noexcept {
            std::cout << std::setw(3) << round << "  ";
            std::cout << std::setw(16) << String::prettyInt(numberOfDepartures);
            std::cout << std::setw(20) << String::prettyInt(numberOfScannedRoutes);
            std::cout << std::setw(22) << String::prettyInt(numberOfSettledVertices);
            std::cout << std::setw(19) << String::prettyInt(numberOfUsedVertices) << "\n";
        }
        size_t numberOfDepartures;
        size_t numberOfScannedRoutes;
        size_t numberOfSettledVertices;
        size_t numberOfUsedVertices;
    };

public:
    OperatorHullRangeRAPTOR(const RAPTOR::Data& cyclingNetwork) :
        cyclingNetwork(cyclingNetwork),
        usedTrips(cyclingNetwork.numberOfRoutes()),
        roundIndex(0),
        currentDepartureTime(never),
        maximumTravelTime(0),
        stopsUpdatedByTransfer(cyclingNetwork.numberOfStops()),
        routesServingUpdatedStops(cyclingNetwork.numberOfRoutes()) {
        for (const RouteId route : cyclingNetwork.routes()) {
            std::vector<bool>(cyclingNetwork.numberOfTripsInRoute(route), false).swap(usedTrips[route]);
        }
    }

public:
    inline void setTargets(const IndexedSet<false, Vertex>& newTargets) noexcept {
        targets = newTargets;
        roundIndex = 0;
        if constexpr (Debug) {
            std::vector<DebugData>(1).swap(debugData);
        }
        std::vector<bool>(cyclingNetwork.transferGraph.numVertices(), false).swap(usedVertices);
        for (std::vector<bool>& flags : usedTrips) {
            std::vector<bool>(flags.size(), false).swap(flags);
        }
        for (const Vertex target : targets) {
            useVertex(target);
        }
    }

    inline const std::vector<bool>& getUsedVertices() const noexcept {
        if constexpr (Debug) {
            std::cout << std::endl << std::endl;
            DebugData sum;
            DebugData::PrintHeader();
            for (size_t i = 0; i < debugData.size(); i++) {
                debugData[i].printData(String::prettyInt(i));
                sum += debugData[i];
            }
            sum.printData("sum");
        }
        return usedVertices;
    }

    inline const std::vector<std::vector<bool>>& getUsedTrips() const noexcept {
        return usedTrips;
    }

    inline void updateUsedVertices(std::vector<bool>& output) const noexcept {
        for (size_t i = 0; i < usedVertices.size(); i++) {
            output[i] = output[i] | usedVertices[i];
        }
    }

    inline void updateUsedVertices(std::vector<bool>& outputVertices, std::vector<std::vector<bool>>& outputTrips) const noexcept {
        for (size_t i = 0; i < usedVertices.size(); i++) {
            outputVertices[i] = outputVertices[i] | usedVertices[i];
        }
        for (size_t i = 0; i < usedTrips.size(); i++) {
            for (size_t j = 0; j < usedTrips[i].size(); j++) {
                outputTrips[i][j] = outputTrips[i][j] | usedTrips[i][j];
            }
        }
    }

    inline void run(const Vertex source) noexcept {
        std::vector<Round>().swap(rounds);
        rounds.emplace_back(cyclingNetwork.transferGraph.numVertices());
        roundIndex = 0;
        initialTransfers(source);
        collectDepartures();
        for (size_t i = 0; i < departures.size(); i++) {
            roundIndex = 0;
            currentDepartureTime = departures[i].departureTime;
            stopsUpdatedByTransfer.clear();
            stopsUpdatedByTransfer.insert(departures[i].departureStop);
            while (i + 1 < departures.size() && departures[i + 1].departureTime == currentDepartureTime) {
                i++;
                stopsUpdatedByTransfer.insert(departures[i].departureStop);
            }
            startNewRound();
            collectRoutesServingUpdatedStops([&](const StopId stop){return rounds[0][stop].arrivalTime + currentDepartureTime;});
            scanRoutes([&](const StopId stop){return rounds[0][stop].arrivalTime + currentDepartureTime;});
            while (roundIndex < 50) {
                if (queue.empty()) break;
                intermediateTransfers();
                if (stopsUpdatedByTransfer.empty()) break;
                startNewRound();
                collectRoutesServingUpdatedStops([&](const StopId stop){return rounds[roundIndex - 1][stop].arrivalTime;});
                scanRoutes([&](const StopId stop){return rounds[roundIndex - 1][stop].arrivalTime;});
            }
        }
    }

private:
    inline void initialTransfers(const Vertex source) noexcept {
        queue.clear();
        rounds[0][source].update(0, noStop, noRouteId, noVertex, queue);
        size_t targetCount = targets.size();
        while(!queue.empty()) {
            RoundLabel* uLabel = queue.extractFront();
            const Vertex u = Vertex(uLabel - &(rounds[0][0]));
            if constexpr (Debug) debugData[0].numberOfSettledVertices++;
            if (targets.contains(u)) {
                usePath(u, 0);
                targetCount--;
                if (targetCount == 0) {
                    maximumTravelTime = uLabel->arrivalTime;
                    break;
                }
            }
            for (const Edge edge : cyclingNetwork.transferGraph.edgesFrom(u)) {
                const Vertex v = cyclingNetwork.transferGraph.get(ToVertex, edge);
                const int arrivalTime = uLabel->arrivalTime + cyclingNetwork.transferGraph.get(TravelTime, edge);
                if (rounds[0][v].arrivalTime > arrivalTime) {
                    rounds[0][v].update(arrivalTime, 0, noRouteId, u, queue);
                }
            }
        }
        queue.clear();
    }

    inline void intermediateTransfers() noexcept {
        while(!queue.empty()) {
            RoundLabel* uLabel = queue.extractFront();
            const Vertex u = Vertex(uLabel - &(rounds[roundIndex][0]));
            if constexpr (Debug) debugData[roundIndex].numberOfSettledVertices++;
            if (targets.contains(u)) {
                usePath(u, roundIndex);
            }
            for (const Edge edge : cyclingNetwork.transferGraph.edgesFrom(u)) {
                const Vertex v = cyclingNetwork.transferGraph.get(ToVertex, edge);
                const int arrivalTime = uLabel->arrivalTime + cyclingNetwork.transferGraph.get(TravelTime, edge);
                if (improveEarliestArrival(v, arrivalTime)) {
                    rounds[roundIndex][v].update(arrivalTime, 0, noRouteId, u, queue);
                    if (cyclingNetwork.isStop(v)) {
                        stopsUpdatedByTransfer.insert(StopId(v));
                    }
                }
            }
        }
        queue.clear();
    }

    inline void collectDepartures() noexcept {
        departures.clear();
        for (const RouteId route : cyclingNetwork.routes()) {
            const size_t numberOfStops = cyclingNetwork.numberOfStopsInRoute(route);
            const StopId* stops = cyclingNetwork.stopArrayOfRoute(route);
            const RAPTOR::StopEvent* stopEvents = cyclingNetwork.firstTripOfRoute(route);
            for (uint32_t stopEventIndex = 0; stopEventIndex < cyclingNetwork.numberOfStopEventsInRoute(route); stopEventIndex++) {
                if ((stopEventIndex + 1) % numberOfStops == 0) continue;
                const StopId stop = stops[stopEventIndex % numberOfStops];
                const int cyclingTime = rounds[0][stop].arrivalTime;
                if (cyclingTime >= INFTY) continue;
                const int departureTime = stopEvents[stopEventIndex].departureTime - cyclingTime;
                departures.emplace_back(departureTime, stop);
            }
        }
        if constexpr (Debug) debugData[roundIndex].numberOfDepartures += departures.size();
        sort(departures);
    }

    inline void startNewRound() noexcept {
        roundIndex++;
        if (roundIndex < rounds.size()) return;
        if (roundIndex == 1) {
            rounds.emplace_back(cyclingNetwork.transferGraph.numVertices());
        } else {
            rounds.emplace_back(rounds.back());
        }
        if constexpr (Debug) {
            if (roundIndex >= debugData.size()) debugData.emplace_back();
        }
    }

    template<typename PREVIOUS_ARRIVAL_TIME>
    inline void collectRoutesServingUpdatedStops(const PREVIOUS_ARRIVAL_TIME& previousArrivalTime) noexcept {
        routesServingUpdatedStops.clear();
        for (const StopId stop : stopsUpdatedByTransfer) {
            for (const RAPTOR::RouteSegment& route : cyclingNetwork.routesContainingStop(stop)) {
                AssertMsg(cyclingNetwork.isRoute(route.routeId), "Route " << route.routeId << " is out of range!");
                AssertMsg(cyclingNetwork.stopIds[cyclingNetwork.firstStopIdOfRoute[route.routeId] + route.stopIndex] == stop, "RAPTOR data contains invalid route segments!");
                if (route.stopIndex + 1 == cyclingNetwork.numberOfStopsInRoute(route.routeId)) continue;
                if (cyclingNetwork.lastTripOfRoute(route.routeId)[route.stopIndex].departureTime < previousArrivalTime(stop)) continue;
                if (routesServingUpdatedStops.contains(route.routeId)) {
                    routesServingUpdatedStops[route.routeId] = std::min(routesServingUpdatedStops[route.routeId], route.stopIndex);
                } else {
                    routesServingUpdatedStops.insert(route.routeId, route.stopIndex);
                }
            }
        }
        stopsUpdatedByTransfer.clear();
    }

    template<typename PREVIOUS_ARRIVAL_TIME>
    inline void scanRoutes(const PREVIOUS_ARRIVAL_TIME& previousArrivalTime) noexcept {
        for (const RouteId route : routesServingUpdatedStops.getKeys()) {
            if constexpr (Debug) debugData[roundIndex].numberOfScannedRoutes++;
            StopIndex stopIndex = routesServingUpdatedStops[route];
            const size_t tripSize = cyclingNetwork.numberOfStopsInRoute(route);
            AssertMsg(stopIndex < tripSize - 1, "Cannot scan a route starting at/after the last stop (Route: " << route << ", StopIndex: " << stopIndex << ", TripSize: " << tripSize << ", RoundIndex: " << roundIndex << ")!");

            const StopId* stops = cyclingNetwork.stopArrayOfRoute(route);
            const RAPTOR::StopEvent* trip = cyclingNetwork.lastTripOfRoute(route);
            StopId stop = stops[stopIndex];
            AssertMsg(trip[stopIndex].departureTime >= previousArrivalTime(stop), "Cannot scan a route after the last trip has departed (Route: " << route << ", Stop: " << stop << ", StopIndex: " << stopIndex << ", Time: " << previousArrivalTime(stop) << ", LastDeparture: " << trip[stopIndex].departureTime << ", RoundIndex: " << roundIndex << ")!");

            StopIndex parentIndex = stopIndex;
            const RAPTOR::StopEvent* firstTrip = cyclingNetwork.firstTripOfRoute(route);
            while (stopIndex < tripSize - 1) {
                while ((trip > firstTrip) && ((trip - tripSize + stopIndex)->departureTime >= previousArrivalTime(stop))) {
                    trip -= tripSize;
                    parentIndex = stopIndex;
                }
                stopIndex++;
                stop = stops[stopIndex];
                if (improveEarliestArrival(stop, trip[stopIndex].arrivalTime)) {
                    rounds[roundIndex][stop].update(trip[stopIndex].arrivalTime, parentIndex, route, noVertex, queue);
                    stopsUpdatedByTransfer.insert(stop);
                }
            }
        }
    }

    inline bool improveEarliestArrival(const Vertex vertex, const int time) noexcept {
        if (currentDepartureTime + rounds[0][vertex].arrivalTime <= time) return false;
        if (currentDepartureTime + maximumTravelTime < time) return false;
        for (size_t i = roundIndex; i > 0; i--) {
            if (rounds[i][vertex].arrivalTime <= time) return false;
        }
        return true;
    }

    inline void usePath(Vertex v, size_t i) noexcept {
        while (true) {
            RoundLabel& label = rounds[i][v];
            if (label.routeId == noRouteId) {
                v = label.parentVertex;
                if (v == noVertex) break;
                useVertex(v);
            } else {
                useTrip(StopId(v), label.routeId, label.arrivalTime);
                for (const StopId* stops = cyclingNetwork.stopArrayOfRoute(label.routeId) + label.parentIndex; *stops != v; stops++) {
                    useVertex(*stops);
                }
                useVertex(v);
                v = cyclingNetwork.stopArrayOfRoute(label.routeId)[label.parentIndex];
                i--;
            }
        }
    }

    inline void useVertex(const Vertex v) noexcept {
        if constexpr (Debug) {
            if (!usedVertices[v]) debugData[roundIndex].numberOfUsedVertices++;
        }
        usedVertices[v] = true;
    }

    inline void useTrip(const StopId stop, const RouteId route, const int arrivalTime) noexcept {
        for (RAPTOR::RouteSegment rs(route, StopIndex(1)); rs.stopIndex < cyclingNetwork.numberOfStopsInRoute(route); rs.stopIndex++) {
            if (cyclingNetwork.stopOfRouteSegment(rs) != stop) continue;
            const size_t tripSize = cyclingNetwork.numberOfStopsInRoute(route);
            const RAPTOR::StopEvent* tripArray = cyclingNetwork.firstTripOfRoute(route) + rs.stopIndex;
            for (size_t tripIndex = 0; tripIndex < cyclingNetwork.numberOfTripsInRoute(route); tripIndex++) {
                if (tripArray[tripIndex * tripSize].arrivalTime != arrivalTime) continue;
                usedTrips[route][tripIndex] = true;
            }
        }
    }

private:
    const RAPTOR::Data& cyclingNetwork;

    IndexedSet<false, Vertex> targets;
    std::vector<bool> usedVertices;
    std::vector<std::vector<bool>> usedTrips;

    std::vector<Round> rounds;
    size_t roundIndex;

    int currentDepartureTime;
    int maximumTravelTime;

    ExternalKHeap<2, RoundLabel> queue;

    std::vector<DepartureLabel> departures;

    IndexedSet<false, StopId> stopsUpdatedByTransfer;
    IndexedMap<StopIndex, false, RouteId> routesServingUpdatedStops;

    std::vector<DebugData> debugData;

};

}

