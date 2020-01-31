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

#include "../../../Helpers/Types.h"
#include "../../../Helpers/Vector/Vector.h"

#include "../../../DataStructures/Container/ExternalKHeap.h"
#include "../../../DataStructures/Container/Map.h"
#include "../../../DataStructures/Container/Set.h"

namespace BikeSharing {

class DistanceDijkstra {

private:
    struct Label : public ExternalKHeapElement {
        Label() : ExternalKHeapElement(), distance(intMax), parent(noVertex), timeStamp(-1) {}
        inline bool hasSmallerKey(const VertexLabel* other) const {
            return distance < other->distance;
        }
        inline void update(const int d, ExternalKHeap<2, Label>& q) noexcept {
            distance = d;
            q.update(this);
        }
        int distance;
        int timeStamp;
    };

public:
    DistanceDijkstra(const TransferGraph& cyclingCoreGraph, const IndexedSet<false, Vertex>& targets) :
        cyclingCoreGraph(cyclingCoreGraph),
        targets(targets),
        label(cyclingCoreGraph.numVertices),
        timeStamp(0) {
    }

    inline void run(const Vertex source) noexcept {
        queue.clear();
        timeStamp++;
        getLabel(source).update(0, noVertex, queue);
        size_t targetCount = targets.size();
        while(!queue.empty()) {
            Label* uLabel = queue.extractFront();
            const Vertex u = Vertex(uLabel - &(label[0]));
            if (targets.contains(u)) {
                targetCount--;
                if (targetCount == 0) break;
            }
            for (const Edge edge : cyclingCoreGraph.edgesFrom(u)) {
                const Vertex v = cyclingCoreGraph.get(ToVertex, edge);
                Label& vLabel = getLabel(v);
                const int distance = uLabel->distance + cyclingCoreGraph.get(TravelTime, edge);
                if (vLabel.distance > distance) {
                    vLabel.update(distance, queue);
                }
            }
        }
    }

    inline int getDistance(const Vertex vertex) noexcept {
        return getLabel(vertex).distance;
    }

private:
    inline Label& getLabel(const Vertex vertex) noexcept {
        Label& result = label[vertex];
        result.distance = (result.timeStamp == timeStamp) ? (result.distance) : (intMax);
        result.timeStamp = timeStamp;
        return result;
    }

private:
    const TransferGraph& cyclingCoreGraph;
    const IndexedSet<false, Vertex>& targets;

    std::vector<Label> label;
    ExternalKHeap<2, Label> queue;
    int timeStamp;

};

class PathDijkstra {

private:
    struct Label : public ExternalKHeapElement {
        Label() : ExternalKHeapElement(), distance(intMax), parent(noVertex), timeStamp(-1) {}
        inline bool hasSmallerKey(const VertexLabel* other) const {
            return distance < other->distance;
        }
        inline void update(const int d, const Vertex p, ExternalKHeap<2, Label>& q) noexcept {
            distance = d;
            parent = p;
            q.update(this);
        }
        int distance;
        Vertex parent;
        int timeStamp;
    };

public:
    PathDijkstra(const TransferGraph& cyclingCoreGraph, std::vector<bool>& usedVertices) :
        cyclingCoreGraph(cyclingCoreGraph),
        usedVertices(usedVertices),
        label(cyclingCoreGraph.numVertices),
        timeStamp(0) {
    }

    inline void run(const Vertex source, const Set<Vertex>& targets) noexcept {
        queue.clear();
        timeStamp++;
        getLabel(source).update(0, noVertex, queue);
        size_t targetCount = targets.size();
        while(!queue.empty()) {
            Label* uLabel = queue.extractFront();
            const Vertex u = Vertex(uLabel - &(label[0]));
            if (targets.contains(u)) {
                targetCount--;
                usedVertices[u] = true;
                while (uLabel.parent != noVertex) {
                    usedVertices[uLabel.parent] = true;
                    uLabel.parent = label[uLabel.parent].parent
                }
                if (targetCount == 0) break;
            }
            for (const Edge edge : cyclingCoreGraph.edgesFrom(u)) {
                const Vertex v = cyclingCoreGraph.get(ToVertex, edge);
                Label& vLabel = getLabel(v);
                const int distance = uLabel->distance + cyclingCoreGraph.get(TravelTime, edge);
                if (vLabel.distance > distance) {
                    vLabel.update(distance, u, queue);
                }
            }
        }
    }

private:
    inline Label& getLabel(const Vertex vertex) noexcept {
        Label& result = label[vertex];
        result.distance = (result.timeStamp == timeStamp) ? (result.distance) : (intMax);
        result.timeStamp = timeStamp;
        return result;
    }

private:
    const TransferGraph& cyclingCoreGraph;
    std::vector<bool>& usedVertices;

    std::vector<Label> label;
    ExternalKHeap<2, Label> queue;
    int timeStamp;

};

class RangeRAPTOR {

private:
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


public:
    RangeRAPTOR(const RAPTOR::Data& cyclingNetwork, const IndexedSet<false, Vertex>& targets, std::vector<bool>& usedVertices, Map<Vertex, Set<Vertex>>& paths) :
        cyclingNetwork(cyclingNetwork),
        targets(targets),
        usedVertices(usedVertices),
        (paths),
        initialTransfers(cyclingNetwork.transferGraph, targets) {
    }

    inline void run(const Vertex source) noexcept {
        initialTransfers.run(source);
        collectDepartures();
        for (size_t i = 0; i < departures.size(); i++) {


            forwardRaptor.template runInitialize<false>(source, departures[i].departureTime);
            forwardRaptor.runAddSource(departures[i].departureStop, departures[i].departureTime + forwardRaptor.getWalkingTravelTime(departures[i].departureStop));
            while (i + 1 < departures.size() && departures[i].departureTime == departures[i + 1].departureTime) {
                i++;
                forwardRaptor.runAddSource(departures[i].departureStop, departures[i].departureTime + forwardRaptor.getWalkingTravelTime(departures[i].departureStop));
            }
            if constexpr (Debug) std::cout << "Departure Time: " << departures[i].departureTime << std::endl;
            forwardRaptor.template runRounds(maxRounds);
            collectArrivals(departures[i].departureTime, targets, forwardRaptor.getReachedVertices());
        }
    }

private:
    inline void collectDepartures() noexcept {
        departures.clear();
        for (const RouteId route : cyclingNetwork.routes()) {
            const size_t numberOfStops = cyclingNetwork.numberOfStopsInRoute(route);
            const StopId* stops = cyclingNetwork.stopArrayOfRoute(route);
            const StopEvent* stopEvents = cyclingNetwork.firstTripOfRoute(route);
            for (uint32_t stopEventIndex = 0; stopEventIndex < cyclingNetwork.numberOfStopEventsInRoute(route); stopEventIndex++) {
                if ((stopEventIndex + 1) % numberOfStops == 0) continue;
                const StopId stop = stops[stopEventIndex % numberOfStops];
                const int cyclingTime = initialTransfers.getDistance(stop);
                if (cyclingTime == intMax) continue;
                const int departureTime = stopEvents[stopEventIndex].departureTime - cyclingTime;
                departures.emplace_back(departureTime, stop);
            }
        }
        sort(departures);
    }

private:
    const RAPTOR::Data& cyclingNetwork;
    const IndexedSet<false, Vertex>& targets;
    std::vector<bool>& usedVertices;
    Map<Vertex, Set<Vertex>>& paths;

    DistanceDijkstra initialTransfers;
    std::vector<DepartureLabel> departures;

}

class Reachability {

public:
    inline std::vector<bool> run(const IndexedSet<false, Vertex>& bikeSharingStations) noexcept {
        std::vector<bool> result(cyclingCoreCH.numVertices(), false);
        Map<Vertex, Set<Vertex>> paths;
        RangeRAPTOR rangeRaptor(cyclingNetwork, bikeSharingStations, result, paths);
        for (const Vertex bikeSharingStation : bikeSharingStations) {
            rangeRaptor.run(bikeSharingStation);
        }
        PathDijkstra dijkstra(cyclingNetwork.transferGraph, result);
        for (const auto& [source, targets] : paths) {
            dijkstra.run(source, targets);
        }
        return result;
    }

private:
    const RAPTOR::Data& cyclingNetwork;

};

}