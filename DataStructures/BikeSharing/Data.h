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

#include <random>

#include "Stations.h"

#include "../RAPTOR/Data.h"
#include "../Intermediate/Data.h"

#include "../../Algorithms/CH/CH.h"
#include "../../Algorithms/CH/Preprocessing/BidirectionalWitnessSearch.h"
#include "../../Algorithms/RAPTOR/TransferShortcuts/Preprocessing/Builder.h"
#include "../../Algorithms/RAPTOR/BikeSharing/OperatorHullRangeRAPTOR.h"

#include "../../Helpers/String/String.h"
#include "../../Helpers/Console/Progress.h"
#include "../../Helpers/MultiThreading.h"

namespace BikeSharing {

class Data {

public:
    Data(Intermediate::Data& data, const Stations& stations, const double seed = 42.0) :
        Data(data, stations, generateBicycleTransportFlags(data, seed)) {
    }

    Data(Intermediate::Data& data, const Stations& stations, const std::vector<bool>& bicycleTransportIsAllowedForTrip) {
        Order newVertexOrder;
        std::vector<Geometry::Point> vertexCoordinates;
        for (const Vertex vertex : data.transferGraph.vertices()) {
            if (data.isStop(vertex)) {
                newVertexOrder.emplace_back(vertex);
            } else {
                vertexCoordinates.emplace_back(data.transferGraph.get(Coordinates, vertex));
            }
        }
        Geometry::Rectangle boundingBox = Geometry::Rectangle::BoundingBox(vertexCoordinates);
        Geometry::GeoMetricAproximation metric = Geometry::GeoMetricAproximation::ComputeCorrection(boundingBox.center());
        CoordinateTree<Geometry::GeoMetricAproximation> ct(metric, vertexCoordinates);
        std::vector<Geometry::Point> stationCoordinates;
        std::vector<Vertex> nearestVertexOfStation;
        std::vector<uint32_t> operatorOfStation;
        std::vector<uint32_t> nearestStationOfVertex(data.transferGraph.numVertices(), -1);
        std::vector<uint32_t> distanceToStationOfVertex(data.transferGraph.numVertices(), intMax);
        uint32_t station = 0;
        operatorNames.emplace_back("None");
        for (auto [key, value] : stations.getOperatorMap()) {
            for (const Geometry::Point& coordinate : value) {
                stationCoordinates.emplace_back(coordinate);
                operatorOfStation.emplace_back(operatorNames.size());
                const Vertex vertex = Vertex(ct.getNearestNeighbor(coordinate) + data.numberOfStops());
                nearestVertexOfStation.emplace_back(vertex);
                const uint32_t distance = geoDistanceInCM(coordinate, data.transferGraph.get(Coordinates, vertex));
                if (distanceToStationOfVertex[vertex] > distance) {
                    distanceToStationOfVertex[vertex] = distance;
                    nearestStationOfVertex[vertex] = station;
                }
                station++;
            }
            operatorNames.emplace_back(key);
        }
        for (size_t station = 0; station < stationCoordinates.size(); station++) {
            if ((nearestStationOfVertex[nearestVertexOfStation[station]] == station) && (distanceToStationOfVertex[nearestVertexOfStation[station]] < 500)) {
                newVertexOrder.emplace_back(nearestVertexOfStation[station]);
            } else {
                const Vertex vertex = Vertex(data.transferGraph.numVertices());
                const double distance = geoDistanceInCM(stationCoordinates[station], data.transferGraph.get(Coordinates, nearestVertexOfStation[station]));
                newVertexOrder.emplace_back(vertex);
                data.transferGraph.addVertex();
                data.transferGraph.set(Coordinates, vertex, stationCoordinates[station]);
                data.transferGraph.addEdge(vertex, nearestVertexOfStation[station]).set(TravelTime, distance * 0.0018);
                data.transferGraph.addEdge(nearestVertexOfStation[station], vertex).set(TravelTime, distance * 0.0018);
                nearestVertexOfStation[station] = vertex;
            }
        }
        for (const Vertex vertex : data.transferGraph.vertices()) {
            if (data.isStop(vertex)) continue;
            if (vertex >= nearestStationOfVertex.size()) continue;
            if ((nearestStationOfVertex[vertex] < nearestVertexOfStation.size()) && (nearestVertexOfStation[nearestStationOfVertex[vertex]] == vertex)) continue;
            newVertexOrder.emplace_back(vertex);
        }
        Permutation vertexPermutation(Construct::Invert, newVertexOrder);
        data.transferGraph.applyVertexPermutation(vertexPermutation);
        operatorOfVertex.resize(data.transferGraph.numVertices(), 0);
        pickUpTimeOfVertex.resize(data.transferGraph.numVertices(), 0);
        dropOffTimeOfVertex.resize(data.transferGraph.numVertices(), 0);
        verticesOfOperator.resize(stations.numberOfOperators() + 1);
        for (size_t station = 0; station < stationCoordinates.size(); station++) {
            const Vertex vertex = Vertex(vertexPermutation[nearestVertexOfStation[station]]);
            pickUpTimeOfVertex[vertex] = 20;
            dropOffTimeOfVertex[vertex] = 10;
            operatorOfVertex[vertex] = operatorOfStation[station];
            verticesOfOperator[operatorOfStation[station]].emplace_back(vertex);
        }
        walkingNetwork = RAPTOR::Data::FromIntermediate(data);
        Graph::computeTravelTimes(walkingNetwork.transferGraph, 4.5, true);
        walkingNetwork.useImplicitDepartureBufferTimes();
        data.removeTripsWithoutBicycleTransport(bicycleTransportIsAllowedForTrip);
        cyclingNetwork = RAPTOR::Data::FromIntermediate(data);
        Graph::computeTravelTimes(cyclingNetwork.transferGraph, 20.0, true);
        cyclingNetwork.useImplicitDepartureBufferTimes();
    }

    Data(const std::string& fileName) {
        deserialize(fileName);
    }

    Data(const Data& other, const std::vector<uint32_t>& operators) :
        walkingNetwork(other.walkingNetwork),
        cyclingNetwork(other.cyclingNetwork),
        walkingGraph(other.walkingGraph),
        walkingCoreCH(other.walkingCoreCH),
        operatorNames(1, other.operatorNames.front()),
        operatorOfVertex(other.operatorOfVertex.size(), 0),
        pickUpTimeOfVertex(other.pickUpTimeOfVertex),
        dropOffTimeOfVertex(other.dropOffTimeOfVertex),
        verticesOfOperator(1, other.verticesOfOperator.front()),
        reachableVerticesOfOperator(1, other.reachableVerticesOfOperator.front()),
        reachableTripsOfOperator(1, other.reachableTripsOfOperator.front()) {
        for (const uint32_t op : operators) {
            for (const Vertex vertex : other.verticesOfOperator[op]) {
                operatorOfVertex[vertex] = operatorNames.size();
            }
            operatorNames.emplace_back(other.operatorNames[op]);
            verticesOfOperator.emplace_back(other.verticesOfOperator[op]);
            reachableVerticesOfOperator.emplace_back(other.reachableVerticesOfOperator[op]);
            reachableTripsOfOperator.emplace_back(other.reachableTripsOfOperator[op]);
        }
    }

    Data(const RAPTOR::Data& walkingNetwork, const TransferGraph& walkingGraph, const CH::CH& walkingCoreCH) :
        walkingNetwork(walkingNetwork),
        walkingGraph(walkingGraph),
        walkingCoreCH(walkingCoreCH),
        operatorNames(1, "None"),
        operatorOfVertex(walkingNetwork.transferGraph.numVertices(), 0),
        pickUpTimeOfVertex(walkingNetwork.transferGraph.numVertices(), 0),
        dropOffTimeOfVertex(walkingNetwork.transferGraph.numVertices(), 0),
        verticesOfOperator(1),
        reachableVerticesOfOperator(1, std::vector<bool>(walkingNetwork.transferGraph.numVertices(), true)),
        reachableTripsOfOperator(1, tripFlags(walkingNetwork, true)) {
    }

public:
    static inline std::vector<bool> generateBicycleTransportFlags(const Intermediate::Data& data, const double seed = 42.0) noexcept {
        std::vector<bool> result(data.numberOfTrips(), false);
        std::mt19937 randomGenerator(seed);
        std::uniform_real_distribution<> uniformDistribution(0.0, 1.0);
        for (size_t i = 0; i < result.size(); i++) {
            switch (data.trips[i].type) {
                case Vehicle::Type::Tram: {
                    result[i] = uniformDistribution(randomGenerator) <= 0.8;
                    break;
                }
                case Vehicle::Type::Subway: {
                    result[i] = true;
                    break;
                }
                case Vehicle::Type::Rail: {
                    const std::string name = data.trips[i].tripName + " " + data.trips[i].routeName;
                    if (String::containsSubString(name, "ICE")) {
                        result[i] = uniformDistribution(randomGenerator) <= 0.1;
                    } else if (String::containsSubString(name, "IC")) {
                        result[i] = uniformDistribution(randomGenerator) <= 0.8;
                    } else if (String::containsSubString(name, "RE")) {
                        result[i] = true;
                    } else {
                        result[i] = true;
                    }
                    break;
                }
                case Vehicle::Type::Bus: {
                    result[i] = uniformDistribution(randomGenerator) <= 0.5;
                    break;
                }
                case Vehicle::Type::Ferry: {
                    result[i] = true;
                    break;
                }
                case Vehicle::Type::CableCar: {
                    result[i] = false;
                    break;
                }
                case Vehicle::Type::Gondola: {
                    result[i] = false;
                    break;
                }
                case Vehicle::Type::Funicular: {
                    result[i] = false;
                    break;
                }
                default: {
                    result[i] = uniformDistribution(randomGenerator) <= 0.5;
                    break;
                }
            }
        }
        return result;
    }

    inline void computeCoreCHs() noexcept {
        CH::CH cyclingCoreCH;
        walkingGraph = walkingNetwork.transferGraph;
        computeCoreCH(walkingNetwork, walkingCoreCH);
        computeCoreCH(cyclingNetwork, cyclingCoreCH);
    }

    inline void computeCoreCH(RAPTOR::Data& data, CH::CH& coreCH) noexcept {
        Intermediate::TransferGraph resultGraph;
        resultGraph.addVertices(data.transferGraph.numVertices());
        resultGraph[Coordinates] = data.transferGraph[Coordinates];
        CHCoreGraph graph;
        Graph::move(std::move(data.transferGraph), graph, Weight << TravelTime);
        size_t numberOfCoreVertices = 0;
        std::vector<bool> isNormalVertex(graph.numVertices(), true);
        for (const StopId stop : walkingNetwork.stops()) {
            if (!isNormalVertex[stop]) continue;
            isNormalVertex[stop] = false;
            numberOfCoreVertices++;
        }
        for (const std::vector<Vertex>& stations : verticesOfOperator) {
            for (const Vertex station : stations) {
                if (!isNormalVertex[station]) continue;
                isNormalVertex[station] = false;
                numberOfCoreVertices++;
            }
        }
        using DEBUGGER = CH::TimeDebugger;
        using WITNESS_SEARCH = CH::BidirectionalWitnessSearch<CHCoreGraph, DEBUGGER, 200>;
        using KEY_FUNCTION = CH::PartialKey<WITNESS_SEARCH, CH::GreedyKey<WITNESS_SEARCH, 1024, 256, 0>>;
        using STOP_CRITERION = CH::CoreCriterion;
        CH::Builder<DEBUGGER, WITNESS_SEARCH, KEY_FUNCTION, STOP_CRITERION, false, false> chBuilder(std::move(graph), KEY_FUNCTION(isNormalVertex, graph.numVertices()), STOP_CRITERION(numberOfCoreVertices, 14));
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
        Graph::move(std::move(resultGraph), data.transferGraph);
    }

    inline void computeOperatorHulls() noexcept {
        std::cout << "Computing reachable subgraphs...\n";
        reachableVerticesOfOperator.clear();
        reachableVerticesOfOperator.emplace_back(std::vector<bool>(walkingNetwork.transferGraph.numVertices(), true));
        reachableTripsOfOperator.clear();
        reachableTripsOfOperator.emplace_back(tripFlags(walkingNetwork, true));

        Progress progress(numberOfBikeSharingStations());
        OperatorHullRangeRAPTOR<true> ohrr(cyclingNetwork);
        for (size_t bikeOperator = 1; bikeOperator < numberOfOperators(); bikeOperator++) {
            IndexedSet<false, Vertex> stations(cyclingNetwork.transferGraph.numVertices(), verticesOfOperator[bikeOperator]);
            ohrr.setTargets(stations);
            for (const Vertex station : stations) {
                ohrr.run(station);
                progress++;
            }
            reachableVerticesOfOperator.emplace_back(ohrr.getUsedVertices());
            reachableTripsOfOperator.emplace_back(ohrr.getUsedTrips());
        }
        progress.finished();
    }

    inline void computeOperatorHulls(const int numberOfThreads, const int pinMultiplier = 1) noexcept {
        std::cout << "Computing reachable subgraphs...\n";
        reachableVerticesOfOperator.clear();
        reachableVerticesOfOperator.emplace_back(std::vector<bool>(walkingNetwork.transferGraph.numVertices(), true));
        reachableVerticesOfOperator.resize(numberOfOperators(), std::vector<bool>(walkingNetwork.transferGraph.numVertices(), false));
        reachableTripsOfOperator.clear();
        reachableTripsOfOperator.emplace_back(tripFlags(walkingNetwork, true));
        reachableTripsOfOperator.resize(numberOfOperators(), tripFlags(cyclingNetwork, false));

        Progress progress(numberOfBikeSharingStations());

        std::vector<IndexedSet<false, Vertex>> stationsOfOperator(1, IndexedSet<false, Vertex>());
        std::vector<std::pair<Vertex, size_t>> queries;
        for (size_t bikeOperator = 1; bikeOperator < numberOfOperators(); bikeOperator++) {
            stationsOfOperator.emplace_back(cyclingNetwork.transferGraph.numVertices(), verticesOfOperator[bikeOperator]);
            for (const Vertex station : stationsOfOperator.back()) {
                queries.emplace_back(std::make_pair(station, bikeOperator));
            }
        }
        const size_t numberOfQueries = queries.size();
        const int numCores = numberOfCores();

        omp_set_num_threads(numberOfThreads);
        #pragma omp parallel
        {
            int threadId = omp_get_thread_num();
            pinThreadToCoreId((threadId * pinMultiplier) % numCores);
            AssertMsg(omp_get_num_threads() == numberOfThreads, "Number of threads is " << omp_get_num_threads() << ", but should be " << numberOfThreads << "!");

            OperatorHullRangeRAPTOR<false> ohrr(cyclingNetwork);
            size_t currentOperator = 0;

            #pragma omp for schedule(dynamic,1)
            for (size_t i = 0; i < numberOfQueries; i++) {
                const Vertex station = queries[i].first;
                const size_t nextOperator = queries[i].second;
                if (currentOperator != nextOperator) {
                    if (currentOperator != 0) {
                        #pragma omp critical
                        {
                            ohrr.updateUsedVertices(reachableVerticesOfOperator[currentOperator], reachableTripsOfOperator[currentOperator]);
                        }
                    }
                    currentOperator = nextOperator;
                    ohrr.setTargets(stationsOfOperator[currentOperator]);
                }
                ohrr.run(station);
                progress++;
            }

            #pragma omp critical
            {
                ohrr.updateUsedVertices(reachableVerticesOfOperator[currentOperator], reachableTripsOfOperator[currentOperator]);
            }
        }

        progress.finished();

        std::cout << "Operator      Used Vertices      Used Trips\n";
        size_t vertexCountSum = 0;
        size_t tripCountSum = 0;
        for (size_t bikeOperator = 1; bikeOperator < numberOfOperators(); bikeOperator++) {
            size_t vertexCount = 0;
            for (const Vertex vertex : walkingNetwork.transferGraph.vertices()) {
                if (reachableVerticesOfOperator[bikeOperator][vertex]) vertexCount++;
            }
            vertexCountSum += vertexCount;
            size_t tripCount = 0;
            for (size_t i = 0; i < reachableTripsOfOperator[bikeOperator].size(); i++) {
                for (size_t j = 0; j < reachableTripsOfOperator[bikeOperator][i].size(); j++) {
                    if (reachableTripsOfOperator[bikeOperator][i][j]) tripCount++;
                }
            }
            tripCountSum += tripCount;
            std::cout << std::setw(3) << bikeOperator << "     " << std::setw(19) << String::prettyInt(vertexCount) << std::setw(16) << String::prettyInt(tripCount) << "\n";
        }
        std::cout << std::setw(3) << "sum" << "     " << std::setw(19) << String::prettyInt(vertexCountSum) << std::setw(16) << String::prettyInt(tripCountSum) << "\n";
        std::cout << std::endl;
    }

private:
    inline std::vector<std::vector<bool>> tripFlags(const RAPTOR::Data& network, const bool value) const noexcept {
        std::vector<std::vector<bool>> result(network.numberOfRoutes());
        for (const RouteId route : network.routes()) {
            std::vector<bool>(network.numberOfTripsInRoute(route), value).swap(result[route]);
        }
        return result;
    }

public:
    inline bool isStop(const Vertex vertex) const noexcept {
        return walkingNetwork.isStop(vertex);
    }

    inline size_t numberOfStops() const noexcept {
        return walkingNetwork.numberOfStops();
    }

    inline size_t numberOfOperators() const noexcept {
        return operatorNames.size();
    }

    inline size_t numberOfBikeSharingStations() const noexcept {
        size_t result = 0;
        for (const std::vector<Vertex>& vertices : verticesOfOperator) {
            result += vertices.size();
        }
        return result;
    }

public:
    inline void printInfo() const noexcept {
        size_t numberOfStations = 0;
        size_t lastStationId = 0;
        for (const std::vector<Vertex>& stations : verticesOfOperator) {
            for (const Vertex station : stations) {
                numberOfStations++;
                lastStationId = std::max<size_t>(lastStationId, station);
            }
        }
        std::cout << "Bike sharing network:" << std::endl;
        std::cout << "   Number of Operators:      " << std::setw(12) << String::prettyInt(operatorNames.size()) << std::endl;
        std::cout << "   Number of Stations:       " << std::setw(12) << String::prettyInt(numberOfStations) << std::endl;
        std::cout << "   Last Stations ID:         " << std::setw(12) << String::prettyInt(lastStationId) << std::endl;
        std::cout << "Walking network:" << std::endl;
        walkingNetwork.printInfo();
        std::cout << "Cycling network:" << std::endl;
        cyclingNetwork.printInfo();
    }

    inline void serialize(const std::string& fileName) const noexcept {
        IO::serialize(fileName, operatorNames, operatorOfVertex, pickUpTimeOfVertex, dropOffTimeOfVertex, verticesOfOperator, reachableVerticesOfOperator, reachableTripsOfOperator);
        walkingNetwork.serialize(fileName + ".walking.raptor");
        cyclingNetwork.serialize(fileName + ".cycling.raptor");
        walkingGraph.writeBinary(fileName + ".walking.graph");;
        walkingCoreCH.writeBinary(fileName + ".walking.core");;
    }

    inline void deserialize(const std::string& fileName) noexcept {
        IO::deserialize(fileName, operatorNames, operatorOfVertex, pickUpTimeOfVertex, dropOffTimeOfVertex, verticesOfOperator, reachableVerticesOfOperator, reachableTripsOfOperator);
        walkingNetwork.deserialize(fileName + ".walking.raptor");
        cyclingNetwork.deserialize(fileName + ".cycling.raptor");
        walkingGraph.readBinary(fileName + ".walking.graph");
        walkingCoreCH.readBinary(fileName + ".walking.core");
    }

public:
    RAPTOR::Data walkingNetwork;
    RAPTOR::Data cyclingNetwork;

    TransferGraph walkingGraph;
    CH::CH walkingCoreCH;

    std::vector<std::string> operatorNames;
    std::vector<uint32_t> operatorOfVertex;
    std::vector<int32_t> pickUpTimeOfVertex;
    std::vector<int32_t> dropOffTimeOfVertex;
    std::vector<std::vector<Vertex>> verticesOfOperator;

    std::vector<std::vector<bool>> reachableVerticesOfOperator;

    std::vector<std::vector<std::vector<bool>>> reachableTripsOfOperator;

};

}
