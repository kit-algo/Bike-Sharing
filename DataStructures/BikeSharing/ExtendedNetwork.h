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

#include "Data.h"
#include "Stations.h"

#include "../RAPTOR/Data.h"
#include "../Intermediate/Data.h"

#include "../../Algorithms/CH/CH.h"
#include "../../Algorithms/CH/Preprocessing/BidirectionalWitnessSearch.h"
#include "../../Algorithms/RAPTOR/TransferShortcuts/Preprocessing/Builder.h"

#include "../../Helpers/String/String.h"
#include "../../Helpers/Console/Progress.h"

namespace BikeSharing {

class ExtendedNetwork {

public:
    ExtendedNetwork(const Data& input, const bool useReachabilityHulls = true, const bool computeUltraShortcuts = true) {
        buildNetwork(input, useReachabilityHulls);
        data.printInfo();
        computeCoreCH();
        if (computeUltraShortcuts) {
            computeShortcuts();
            computeCH();
        }
    }

    ExtendedNetwork(const std::string& fileName) {
        deserialize(fileName);
    }

private:
    inline void buildNetwork(const Data& input, const bool useReachabilityHulls = true) {
        Intermediate::Data inter;
        std::vector<std::vector<Vertex>> oldToNewVertexOfOperator;
        oldToNewVertexOfOperator.emplace_back(input.walkingGraph.numVertices(), noVertex);
        oldToNewVertexOfOperator.resize(input.numberOfOperators(), std::vector<Vertex>(input.cyclingNetwork.transferGraph.numVertices(), noVertex));
        std::vector<bool> isIsolatedCyclingVertex(input.cyclingNetwork.transferGraph.numVertices(), true);
        for (const Vertex vertex : input.cyclingNetwork.transferGraph.vertices()) {
            for (const Edge edge : input.cyclingNetwork.transferGraph.edgesFrom(vertex)) {
                isIsolatedCyclingVertex[vertex] = false;
                isIsolatedCyclingVertex[input.cyclingNetwork.transferGraph.get(ToVertex, edge)] = false;
            }
        }

        for (const StopId stop : input.walkingNetwork.stops()) {
            const Vertex newVertex = inter.transferGraph.addVertex();
            oldToNewVertexOfOperator[0][stop] = newVertex;
            inter.transferGraph.set(Coordinates, newVertex, input.walkingGraph.get(Coordinates, stop));
            inter.stops.emplace_back(input.walkingNetwork.stopData[stop]);
        }

        for (size_t bikeOperator = 1; bikeOperator < input.numberOfOperators(); bikeOperator++) {
            for (const StopId stop : input.cyclingNetwork.stops()) {
                if ((useReachabilityHulls) && (!input.reachableVerticesOfOperator[bikeOperator][stop])) continue;
                const Vertex newVertex = inter.transferGraph.addVertex();
                oldToNewVertexOfOperator[bikeOperator][stop] = newVertex;
                inter.transferGraph.set(Coordinates, newVertex, input.cyclingNetwork.transferGraph.get(Coordinates, stop));
                inter.stops.emplace_back(input.cyclingNetwork.stopData[stop]);
            }
        }

        for (const Vertex vertex : input.walkingGraph.vertices()) {
            if (oldToNewVertexOfOperator[0][vertex] != noVertex) continue;
            const Vertex newVertex = inter.transferGraph.addVertex();
            oldToNewVertexOfOperator[0][vertex] = newVertex;
            inter.transferGraph.set(Coordinates, newVertex, input.walkingGraph.get(Coordinates, vertex));
        }

        for (size_t bikeOperator = 1; bikeOperator < input.numberOfOperators(); bikeOperator++) {
            for (const Vertex vertex : input.cyclingNetwork.transferGraph.vertices()) {
                if (oldToNewVertexOfOperator[bikeOperator][vertex] != noVertex) continue;
                if ((useReachabilityHulls) && (!input.reachableVerticesOfOperator[bikeOperator][vertex])) continue;
                if (isIsolatedCyclingVertex[vertex]) continue;
                const Vertex newVertex = inter.transferGraph.addVertex();
                oldToNewVertexOfOperator[bikeOperator][vertex] = newVertex;
                inter.transferGraph.set(Coordinates, newVertex, input.cyclingNetwork.transferGraph.get(Coordinates, vertex));
            }
        }

        for (const Vertex vertex : input.walkingGraph.vertices()) {
            for (const Edge edge : input.walkingGraph.edgesFrom(vertex)) {
                inter.transferGraph.addEdge(oldToNewVertexOfOperator[0][vertex], oldToNewVertexOfOperator[0][input.walkingGraph.get(ToVertex, edge)]).set(TravelTime, input.walkingGraph.get(TravelTime, edge));
            }
        }

        for (size_t bikeOperator = 1; bikeOperator < input.numberOfOperators(); bikeOperator++) {
            for (const Vertex vertex : input.cyclingNetwork.transferGraph.vertices()) {
                if (oldToNewVertexOfOperator[bikeOperator][vertex] == noVertex) continue;
                for (const Edge edge : input.cyclingNetwork.transferGraph.edgesFrom(vertex)) {
                    const Vertex to = oldToNewVertexOfOperator[bikeOperator][input.cyclingNetwork.transferGraph.get(ToVertex, edge)];
                    if (to == noVertex) continue;
                    inter.transferGraph.addEdge(oldToNewVertexOfOperator[bikeOperator][vertex], to).set(TravelTime, input.cyclingNetwork.transferGraph.get(TravelTime, edge));
                }
            }
        }

        for (size_t bikeOperator = 1; bikeOperator < input.numberOfOperators(); bikeOperator++) {
            for (const Vertex vertex : input.verticesOfOperator[bikeOperator]) {
                AssertMsg(oldToNewVertexOfOperator[0][vertex] != noVertex, "Vertex " << vertex << " is not part of the walking network!");
                AssertMsg(oldToNewVertexOfOperator[bikeOperator][vertex] != noVertex, "Vertex " << vertex << " is not part of the cycling network for operator " << bikeOperator << "!");
                inter.transferGraph.addEdge(oldToNewVertexOfOperator[0][vertex], oldToNewVertexOfOperator[bikeOperator][vertex]).set(TravelTime, input.pickUpTimeOfVertex[vertex]);
                inter.transferGraph.addEdge(oldToNewVertexOfOperator[bikeOperator][vertex], oldToNewVertexOfOperator[0][vertex]).set(TravelTime, input.dropOffTimeOfVertex[vertex]);
            }
        }

        for (const RouteId route : input.walkingNetwork.routes()) {
            const StopId* stops = input.walkingNetwork.stopArrayOfRoute(route);
            const size_t tripSize = input.walkingNetwork.numberOfStopsInRoute(route);
            const RAPTOR::StopEvent* lastTrip = input.walkingNetwork.lastTripOfRoute(route);
            for (const RAPTOR::StopEvent* trip = input.walkingNetwork.firstTripOfRoute(route); trip <= lastTrip; trip += tripSize) {
                Intermediate::Trip newTrip("", input.walkingNetwork.routeData[route].name, input.walkingNetwork.routeData[route].type);
                for (size_t i = 0; i < tripSize; i++) {
                    newTrip.stopEvents.emplace_back(StopId(oldToNewVertexOfOperator[0][stops[i]]), trip[i].arrivalTime, trip[i].departureTime);
                }
                if (newTrip.stopEvents.size() > 1) {
                    inter.trips.emplace_back(std::move(newTrip));
                }
            }
        }

        for (size_t bikeOperator = 1; bikeOperator < input.numberOfOperators(); bikeOperator++) {
            for (const RouteId route : input.cyclingNetwork.routes()) {
                const StopId* stops = input.cyclingNetwork.stopArrayOfRoute(route);
                const size_t tripSize = input.cyclingNetwork.numberOfStopsInRoute(route);
                const RAPTOR::StopEvent* lastTrip = input.cyclingNetwork.lastTripOfRoute(route);
                size_t tripIndex = -1;
                for (const RAPTOR::StopEvent* trip = input.cyclingNetwork.firstTripOfRoute(route); trip <= lastTrip; trip += tripSize) {
                    tripIndex++;
                    if ((useReachabilityHulls) && (!input.reachableTripsOfOperator[bikeOperator][route][tripIndex])) continue;
                    Intermediate::Trip newTrip("", input.cyclingNetwork.routeData[route].name, input.cyclingNetwork.routeData[route].type);
                    for (size_t i = 0; i < tripSize; i++) {
                        if (oldToNewVertexOfOperator[bikeOperator][stops[i]] == noVertex) continue;
                        newTrip.stopEvents.emplace_back(StopId(oldToNewVertexOfOperator[bikeOperator][stops[i]]), trip[i].arrivalTime, trip[i].departureTime);
                    }
                    if (newTrip.stopEvents.size() > 1) {
                        inter.trips.emplace_back(std::move(newTrip));
                    }
                }
            }
        }

        oldToNewVertex.swap(oldToNewVertexOfOperator[0]);

        data = RAPTOR::Data::FromIntermediate(inter, 1);
        data.implicitDepartureBufferTimes = input.walkingNetwork.implicitDepartureBufferTimes;
        data.implicitArrivalBufferTimes = input.walkingNetwork.implicitArrivalBufferTimes;
    }

    inline void computeShortcuts() {
        const int numberOfthreads = numberOfCores();
        RAPTOR::TransferShortcuts::Preprocessing::Builder<false> shortcutGraphBuilder(data);
        std::cout << "Computing Transfer Shortcuts (parallel with " << numberOfthreads << " threads)." << std::endl;
        shortcutGraphBuilder.computeShortcuts(ThreadPinning(numberOfthreads, 1), 20000);
        Graph::move(std::move(shortcutGraphBuilder.getShortcutGraph()), data.transferGraph);
    }

    inline void computeCoreCH() {
        TransferGraph inputGraph = data.transferGraph;
        Intermediate::TransferGraph resultGraph;
        resultGraph.addVertices(data.transferGraph.numVertices());
        resultGraph[Coordinates] = data.transferGraph[Coordinates];
        size_t numberOfCoreVertices = 0;
        std::vector<bool> isNormalVertex(inputGraph.numVertices(), true);
        for (const StopId stop : data.stops()) {
            if (!isNormalVertex[stop]) continue;
            isNormalVertex[stop] = false;
            numberOfCoreVertices++;
        }
        using DEBUGGER = CH::TimeDebugger;
        using WITNESS_SEARCH = CH::BidirectionalWitnessSearch<CHCoreGraph, DEBUGGER, 200>;
        using KEY_FUNCTION = CH::PartialKey<WITNESS_SEARCH, CH::GreedyKey<WITNESS_SEARCH, 1024, 256, 0>>;
        using STOP_CRITERION = CH::CoreCriterion;
        CH::Builder<DEBUGGER, WITNESS_SEARCH, KEY_FUNCTION, STOP_CRITERION, false, false> chBuilder(std::move(inputGraph), inputGraph[TravelTime], KEY_FUNCTION(isNormalVertex, inputGraph.numVertices()), STOP_CRITERION(numberOfCoreVertices, 16));
        chBuilder.run();
        std::cout << "   final core size: " << String::prettyInt(chBuilder.numberOfUncontractedVertices()) << std::endl;
        chBuilder.copyCoreToCH();
        coreCH = std::move(chBuilder);
        for (const Vertex vertex : resultGraph.vertices()) {
            if (coreCH.isCoreVertex(vertex)) {
                for (const Edge edge : coreCH.forward.edgesFrom(vertex)) {
                    resultGraph.addEdge(vertex, coreCH.forward.get(ToVertex, edge)).set(TravelTime, coreCH.forward.get(Weight, edge));
                }
            }
        }
        Graph::move(std::move(data.transferGraph), graph);
        Graph::move(std::move(resultGraph), data.transferGraph);

    }

    inline void computeCH() {
        TransferGraph inputGraph = graph;
        using DEBUGGER = CH::TimeDebugger;
        using WITNESS_SEARCH = CH::BidirectionalWitnessSearch<CHCoreGraph, DEBUGGER, 200>;
        using KEY_FUNCTION = CH::GreedyKey<WITNESS_SEARCH, 1024, 512, 0>;
        using STOP_CRITERION = CH::NoStopCriterion;
        CH::Builder<DEBUGGER, WITNESS_SEARCH, KEY_FUNCTION, STOP_CRITERION, false, false> chBuilder(std::move(inputGraph), inputGraph[TravelTime]);
        chBuilder.run();
        chBuilder.copyCoreToCH();
        ch = std::move(chBuilder);
    }

public:
    inline void useCoreGraph() {
        CHGraph graph = coreCH.forward;
        Graph::move(std::move(graph), data.transferGraph, TravelTime << Weight);
    }

    inline void printInfo() const noexcept {
        data.printInfo();
    }

    inline void serialize(const std::string& fileName) const noexcept {
        IO::serialize(fileName, oldToNewVertex);
        data.serialize(fileName + ".ultra.raptor");
        graph.writeBinary(fileName + ".ultra.graph");
        coreCH.writeBinary(fileName + ".ultra.core.ch");
        ch.writeBinary(fileName + ".ultra.ch");;
    }

    inline void deserialize(const std::string& fileName) noexcept {
        IO::deserialize(fileName, oldToNewVertex);
        data.deserialize(fileName + ".ultra.raptor");
        graph.readBinary(fileName + ".ultra.graph");
        coreCH.readBinary(fileName + ".ultra.core.ch");
        ch.readBinary(fileName + ".ultra.ch");
    }

public:
    RAPTOR::Data data;
    TransferGraph graph;
    CH::CH coreCH;
    CH::CH ch;

    std::vector<Vertex> oldToNewVertex;

};

}
