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
#include <algorithm>
#include <random>
#include <vector>
#include <string>
#include <optional>
#include <cmath>

#include "../../Shell/Shell.h"

#include "../../DataStructures/Intermediate/Data.h"
#include "../../DataStructures/RAPTOR/Data.h"
#include "../../DataStructures/BikeSharing/Data.h"
#include "../../DataStructures/BikeSharing/Stations.h"
#include "../../DataStructures/BikeSharing/ExtendedNetwork.h"

#include "../../Algorithms/StronglyConnectedComponents.h"
#include "../../Algorithms/CH/Preprocessing/CHBuilder.h"
#include "../../Algorithms/CH/Preprocessing/Debugger.h"
#include "../../Algorithms/CH/Preprocessing/WitnessSearch.h"
#include "../../Algorithms/CH/Preprocessing/BidirectionalWitnessSearch.h"
#include "../../Algorithms/CH/Preprocessing/StopCriterion.h"
#include "../../Algorithms/CH/Query/CHQuery.h"
#include "../../Algorithms/CH/CH.h"
#include "../../Algorithms/RAPTOR/BikeSharing/OperatorExpandedQueries.h"
#include "../../Algorithms/RAPTOR/BikeSharing/OperatorDependentQueries.h"
#include "../../Algorithms/Dijkstra/Dijkstra.h"

using namespace Shell;

namespace BikeSharing {

struct Query {
    Query(const int s = -1, const int t = -1, const int dt = never) : source(s), target(t), departureTime(dt), earliestArrivalTime(never), numberOfTrips(-1), queryTime(0) {}
    friend std::ostream& operator<<(std::ostream& out, const Query& q) {
        return out << q.source.value() << "," << q.target.value() << "," << q.departureTime << "," << q.earliestArrivalTime << "," << q.numberOfTrips << "," << q.queryTime;
    }
    Vertex source;
    Vertex target;
    int departureTime;
    int earliestArrivalTime;
    int numberOfTrips;
    double queryTime;
};

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// SanitizeBikeSharingStations //////////////////////////////////////////////////////////////////////
class SanitizeBikeSharingStations : public ParameterizedCommand {

public:
    SanitizeBikeSharingStations(BasicShell& shell) :
        ParameterizedCommand(shell, "sanitizeBikeSharingStations", "Maps bike station names to a small set of operators.") {
        addParameter("Raw bss file");
        addParameter("Map file");
        addParameter("Output file");
    }

    virtual void execute() noexcept {
        const std::string bssFileName = getParameter("Raw bss file");
        const std::string mapFileName = getParameter("Map file");
        const std::string outputFileName = getParameter("Output file");

        std::string inputLineA = "";
        std::string inputLineB = "";

        Map<std::string, std::string> map;
        std::vector<std::string> values;

        std::ifstream mapFile(mapFileName);
        AssertMsg(mapFile.is_open(), "Could not open file " << mapFileName << "!");
        while (!mapFile.eof()) {
            getline(mapFile, inputLineA);
            if (mapFile.eof()) break;
            getline(mapFile, inputLineB);
            map.insert(inputLineA, inputLineB);
            values.emplace_back(inputLineB);
        }
        mapFile.close();

        std::ifstream bssFile(bssFileName);
        AssertMsg(bssFile.is_open(), "Could not open file " << bssFileName << "!");
        std::ofstream outputFile(outputFileName);
        AssertMsg(outputFile.is_open(), "Could not open file " << outputFileName << "!");
        while (!bssFile.eof()) {
            getline(bssFile, inputLineA);
            std::vector<std::string> tokens = String::split(inputLineA, ' ');
            if (tokens.size() < 4) continue;
            inputLineB = (tokens.size() > 4) ? (tokens[4]) : ("");
            for (size_t i = 5; i < tokens.size(); i++) {
                inputLineB = inputLineB + " " + tokens[i];
            }
            if (!map.contains(inputLineB)) {
                inputLineA = shell.ask(inputLineB + "> ", values);
                map.insert(inputLineB, inputLineA);
                if (inputLineA == "") continue;
                if (!Vector::contains(values, inputLineA)) values.emplace_back(inputLineA);
            }
            if (map[inputLineB] == "") continue;
            outputFile << tokens[2] << " " << tokens[3] << " " << map[inputLineB] << "\n";
        }
        bssFile.close();
        outputFile.close();

        std::ofstream newMapFile(mapFileName);
        AssertMsg(newMapFile.is_open(), "Could not open file " << mapFileName << "!");
        for (auto [key, value] : map) {
            newMapFile << key << "\n" << value << "\n";
        }
        newMapFile.close();

    }

};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// BuildBikeSharingData //////////////////////////////////////////////////////////////////////
class BuildBikeSharingData : public ParameterizedCommand {

public:
    BuildBikeSharingData(BasicShell& shell) :
        ParameterizedCommand(shell, "buildBikeSharingData", "Creates bike sharing data.") {
        addParameter("Intermediate file");
        addParameter("Station file");
        addParameter("Output file");
        addParameter("Num threads", "0");
        addParameter("Thread offset", "1");
        addParameter("Country", "switzerland", {"switzerland", "germany", "london"});
    }

    virtual void execute() noexcept {
        const std::string intermediateFile = getParameter("Intermediate file");
        const std::string stationFile = getParameter("Station file");
        const std::string outputFile = getParameter("Output file");
        const int numThreads = getParameter<int>("Num threads");
        const int pinMultiplier = getParameter<int>("Thread offset");
        const std::string country = getParameter("Country");

        Intermediate::Data inter = Intermediate::Data::FromBinary(intermediateFile);
        inter.printInfo();
        BikeSharing::Stations stations = BikeSharing::Stations::FromIntermediateFormat(stationFile);
        stations.printInfo();
        stations.deleteSmallOperators(1, {"Unknown"});
        stations.reassignStationsOfOperator("Unknown");
        stations.reassignOutlierStations(2000000);
        stations.printInfo();
        BikeSharing::Data data(inter, stations);
        data.computeCoreCHs();
        if (numThreads == 0) {
            data.computeOperatorHulls();
        } else {
            data.computeOperatorHulls(numThreads, pinMultiplier);
        }
        data.printInfo();
        data.serialize(outputFile);
    }

};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// BuildExtendedBikeSharingData //////////////////////////////////////////////////////////////////////
class BuildExtendedBikeSharingData : public ParameterizedCommand {

public:
    BuildExtendedBikeSharingData(BasicShell& shell) :
        ParameterizedCommand(shell, "buildExtendedBikeSharingData", "Creates extended bike sharing data.") {
        addParameter("BikeSharing file");
        addParameter("Result file");
        addParameter("Use reachability flags", {"true", "false"});
        addParameter("Compute ULTRA shortcuts", "true", {"true", "false"});
    }

    virtual void execute() noexcept {
        const std::string bikeSharingFile = getParameter("BikeSharing file");
        const std::string resultFile = getParameter("Result file");
        const bool useReachabilityFlags = getParameter<bool>("Use reachability flags");
        const bool computeUltraShortcuts = getParameter<bool>("Compute ULTRA shortcuts");

        BikeSharing::Data data(bikeSharingFile);
        data.printInfo();

        BikeSharing::ExtendedNetwork extendedNetwork(data, useReachabilityFlags, computeUltraShortcuts);
        extendedNetwork.printInfo();
        extendedNetwork.serialize(resultFile);
    }

};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// BuildPartialBikeSharingData //////////////////////////////////////////////////////////////////////
class BuildPartialBikeSharingData : public ParameterizedCommand {

public:
    BuildPartialBikeSharingData(BasicShell& shell) :
        ParameterizedCommand(shell, "buildPartialBikeSharingData", "Creates partial bike sharing data.") {
        addParameter("BikeSharing file");
        addParameter("Result file");
        addParameter("Operators");
    }

    virtual void execute() noexcept {
        const std::string bikeSharingFile = getParameter("BikeSharing file");
        const std::string resultFile = getParameter("Result file");
        std::vector<uint32_t> operators = getParameters<uint32_t>("Operators");

        BikeSharing::Data data(bikeSharingFile);
        data.printInfo();

        while (!operators.empty()) {
            BikeSharing::Data partialData(data, operators);
            partialData.serialize(resultFile + std::to_string(operators.size()));
            operators.pop_back();
        }
        BikeSharing::Data partialData(data, operators);
        partialData.serialize(resultFile + std::to_string(operators.size()));
    }

};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// GenerateBikeSharingQueries //////////////////////////////////////////////////////////////////////
class GenerateBikeSharingQueries : public ParameterizedCommand {

public:
    GenerateBikeSharingQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "generateBikeSharingQueries", "Generates random bike sharing queries.") {
        addParameter("BikeSharing file");
        addParameter("Query file");
        addParameter("Number of queries");
        addParameter("Seed", "42");
        addParameter("Minimum departure time", "00:00:00");
        addParameter("Maximum departure time", "24:00:00");
    }

    virtual void execute() noexcept {
        const std::string bikeSharingFile = getParameter("BikeSharing file");
        const std::string queryFile = getParameter("Query file");
        const size_t numberOfQueries = getParameter<int>("Number of queries");
        const int seed = getParameter<int>("Seed");
        const int minimumTime = String::parseSeconds(getParameter("Minimum departure time"));
        const int maximumTime = String::parseSeconds(getParameter("Maximum departure time"));

        BikeSharing::Data data(bikeSharingFile);

        std::mt19937 randomGenerator(seed);
        std::uniform_int_distribution<> vertexDistribution(0, data.walkingGraph.numVertices() - 1);
        std::uniform_int_distribution<> timeDistribution(minimumTime, maximumTime);
        std::vector<BikeSharing::Query> queries;
        while (queries.size() < numberOfQueries) {
            queries.emplace_back(vertexDistribution(randomGenerator), vertexDistribution(randomGenerator), timeDistribution(randomGenerator));
        }

        IO::serialize(queryFile, queries);
    }

};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// GenerateBikeSharingRankQueries //////////////////////////////////////////////////////////////////////
class GenerateBikeSharingRankQueries : public ParameterizedCommand {

public:
    GenerateBikeSharingRankQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "generateBikeSharingRankQueries", "Generates random bike sharing queries.") {
        addParameter("BikeSharing file");
        addParameter("Query file");
        addParameter("Number of queries");
        addParameter("Seed", "42");
        addParameter("Number of queries per source", "10");
        addParameter("Minimum distance", "100");
        addParameter("Maximum distance", "720000");
        addParameter("Minimum departure time", "00:00:00");
        addParameter("Maximum departure time", "24:00:00");
    }

    virtual void execute() noexcept {
        const std::string bikeSharingFile = getParameter("BikeSharing file");
        const std::string queryFile = getParameter("Query file");
        const size_t numberOfQueries = getParameter<int>("Number of queries");
        const int seed = getParameter<int>("Seed");
        const size_t numberOfQueriesPerSource = getParameter<int>("Number of queries per source");
        const double minimumDist = getParameter<double>("Minimum distance");
        const double maximumDist = getParameter<double>("Maximum distance");
        const int minimumTime = String::parseSeconds(getParameter("Minimum departure time"));
        const int maximumTime = String::parseSeconds(getParameter("Maximum departure time"));

        BikeSharing::Data data(bikeSharingFile);
        Dijkstra dijkstra(data.walkingGraph);

        std::mt19937 randomGenerator(seed);
        std::uniform_int_distribution<> vertexDistribution(0, data.walkingGraph.numVertices() - 1);
        std::uniform_int_distribution<> timeDistribution(minimumTime, maximumTime);
        std::uniform_real_distribution<> distanceDistribution(std::log2(minimumDist), std::log2(maximumDist));
        std::vector<BikeSharing::Query> queries;
        while (queries.size() < numberOfQueries) {
            std::vector<double> distances;
            while (distances.size() < numberOfQueriesPerSource) {
                distances.emplace_back(std::exp2(distanceDistribution(randomGenerator)));
            }
            std::sort(distances.begin(), distances.end());
            size_t i = 0;
            const Vertex source = Vertex(vertexDistribution(randomGenerator));
            dijkstra.run(source, noVertex, [&](const Vertex u){
                if (dijkstra.getDistance(u) > distances[i]) {
                    i++;
                    queries.emplace_back(source, u, timeDistribution(randomGenerator));
                }
            }, [&](){
                return (i >= distances.size()) || (queries.size() >= numberOfQueries);
            });
        }

        Permutation perm(Construct::Random, queries.size());
        perm.permutate(queries);

        IO::serialize(queryFile, queries);
    }

};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// RunBikeSharingQueries //////////////////////////////////////////////////////////////////////
class RunBikeSharingQueries : public ParameterizedCommand {

public:
    RunBikeSharingQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "runBikeSharingQueries", "Evaluates random bike sharing queries.") {
        addParameter("BikeSharing file");
        addParameter("Extended BikeSharing file");
        addParameter("Query file");
        addParameter("Result file");
        addParameter("Query type", {"MR-OD", "MR-OD-OP", "MR-OE", "ULTRA-OE"});
    }

    virtual void execute() noexcept {
        const std::string bikeSharingFile = getParameter("BikeSharing file");
        const std::string extendedBikeSharingFile = getParameter("Extended BikeSharing file");
        const std::string queryFile = getParameter("Query file");
        const std::string resultFileName = getParameter("Result file");
        const std::string queryType = getParameter("Query type");

        BikeSharing::Data data(bikeSharingFile);
        std::vector<BikeSharing::Query> queries;
        IO::deserialize(queryFile, queries);

        if (queryType == "MR-OD") {
            RAPTOR::OperatorDependentRAPTOR<false, true, false> bsr(data);
            runQueries(bsr, queries);
        } else if (queryType == "MR-OD-OP") {
            RAPTOR::OperatorDependentRAPTOR<true, true, true> bsr(data);
            runQueries(bsr, queries);
        } else if (queryType == "MR-OE") {
            BikeSharing::ExtendedNetwork extendedNetwork(extendedBikeSharingFile);
            extendedNetwork.useCoreGraph();
            RAPTOR::OperatorExpandedOperatorDependentRAPTOR<true> bsr(extendedNetwork);
            runQueries(bsr, queries);
        } else if (queryType == "ULTRA-OE") {
            BikeSharing::ExtendedNetwork extendedNetwork(extendedBikeSharingFile);
            RAPTOR::OperatorExpandedULTRA<true> ultra(extendedNetwork);
            runQueries(ultra, queries);
        }

        std::ofstream resultFile(resultFileName);
        AssertMsg(resultFile.is_open(), "Could not open file " << resultFileName << "!");
        resultFile << "Source,Target,DepTime,ArrTime,Trips,QueryTime\n";
        for (BikeSharing::Query& query : queries) {
            resultFile << query << "\n";
        }
    }

private:
    template<typename ALGORITHM>
    inline void runQueries(ALGORITHM& algorithm, std::vector<BikeSharing::Query>& queries) {
        std::cout << "Evaluating " << String::prettyInt(queries.size()) << " queries..." << std::endl;
        Timer timer;
        Timer queryTimer;
        for (BikeSharing::Query& query : queries) {
            queryTimer.restart();
            algorithm.run(query.source, query.departureTime, query.target);
            query.queryTime = queryTimer.elapsedMilliseconds();
            query.earliestArrivalTime = algorithm.getEarliestArrivalTime();
            query.numberOfTrips = algorithm.getEarliestArrivalNumerOfTrips();
        }
        const double time = timer.elapsedMilliseconds();
        std::cout << "Done in " << String::msToString(time) << " (" << String::prettyDouble(time / queries.size(), 1) << "ms per query)" << std::endl;
        algorithm.debug(queries.size());
    }

};
