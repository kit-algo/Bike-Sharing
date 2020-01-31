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
#include <iomanip>
#include <vector>
#include <string>

#include "../../DataStructures/RAPTOR/Data.h"

#include "../../Helpers/Timer.h"

namespace RAPTOR {

class NoDebugger {

public:
    inline void initialize(const Data&, const TransferGraph* = NULL) const noexcept {}

    inline void start() const noexcept {}
    inline void done() const noexcept {}

    inline void startInitialization() const noexcept {}
    inline void doneInitialization() const noexcept {}
    inline void directWalking(const int) const noexcept {}

    inline void newRound() const noexcept {}

    inline void startCollectRoutes() const noexcept {}
    inline void stopCollectRoutes() const noexcept {}
    inline void startScanRoutes() const noexcept {}
    inline void stopScanRoutes() const noexcept {}
    inline void startRelaxTransfers() const noexcept {}
    inline void stopRelaxTransfers() const noexcept {}

    inline void scanRoute(const RouteId) const noexcept {}
    inline void scanRouteSegment(const size_t) const noexcept {}

    inline void settleVertex(const Vertex) const noexcept {}
    inline void relaxEdge(const Edge) const noexcept {}
    inline void relaxShortcut(const Vertex, const Vertex) const noexcept {}

    inline void updateStop(const StopId, const int) const noexcept {}
    inline void updateStopByRoute(const StopId, const int) const noexcept {}
    inline void updateStopByTransfer(const StopId, const int) const noexcept {}
};

class TimeDebugger : public NoDebugger {

private:
    struct RoundData {
        inline RoundData& operator+=(const RoundData& other) noexcept {
            numberOfScannedRoutes += other.numberOfScannedRoutes;
            numberOfSettledVertices += other.numberOfSettledVertices;
            numberOfRelaxedEdges += other.numberOfRelaxedEdges;
            numberOfUpdatedStops += other.numberOfUpdatedStops;
            time += other.time;
            return *this;
        }
        int numberOfScannedRoutes{0};
        int numberOfSettledVertices{0};
        int numberOfRelaxedEdges{0};
        int numberOfUpdatedStops{0};
        double time{0.0};
    };

public:
    inline void start() noexcept {
        timer.restart();
        time = 0.0;
        statistics.clear();
        statistics.emplace_back();
    }
    inline void done() noexcept {
        stopTime();
        printHeader();
        printStatistics();
    }

    inline void doneInitialization() noexcept {
        stopTime();
    }

    inline void directWalking(const int time) const noexcept {
        std::cout << "Time required for direct walking: " << String::secToTime(time) << std::endl;
    }

    inline void newRound() noexcept {
        stopTime();
        statistics.emplace_back();
    }

    inline void scanRoute(const RouteId) noexcept {
        statistics.back().numberOfScannedRoutes++;
    }

    inline void settleVertex(const Vertex) noexcept {
        statistics.back().numberOfSettledVertices++;
    }
    inline void relaxEdge(const Edge) noexcept {
        statistics.back().numberOfRelaxedEdges++;
    }

    inline void updateStop(const StopId, const int) noexcept {
        statistics.back().numberOfUpdatedStops++;
    }

private:
    inline void stopTime() noexcept {
        double elapsedMilliseconds = timer.elapsedMilliseconds();
        statistics.back().time = elapsedMilliseconds  - time;
        time = elapsedMilliseconds;
    }

    inline void printHeader() const noexcept {
        std::cout << "\nStatistics:\n"
                  << std::setw(8) << "Round"
                  << std::setw(18) << "Scanned Routes"
                  << std::setw(18) << "Settled Vertices"
                  << std::setw(18) << "Relaxed Edges"
                  << std::setw(18) << "Updated Stops"
                  << std::setw(14) << "Time" << std::endl;
    }

    inline void printRow(const std::string& name, const RoundData& roundData) const noexcept {
        std::cout << std::setw(8) << name
                  << std::setw(18) << String::prettyInt(roundData.numberOfScannedRoutes)
                  << std::setw(18) << String::prettyInt(roundData.numberOfSettledVertices)
                  << std::setw(18) << String::prettyInt(roundData.numberOfRelaxedEdges)
                  << std::setw(18) << String::prettyInt(roundData.numberOfUpdatedStops)
                  << std::setw(14) << String::msToString(roundData.time) << std::endl;
    }

    inline void printStatistics() const noexcept {
        RoundData total = statistics.front();
        printRow("init", total);
        for (size_t i = 1; i < statistics.size(); i++) {
            printRow(String::prettyInt(i - 1), statistics[i]);
            total += statistics[i];
        }
        printRow("total", total);
    }

private:
    Timer timer;
    double time;

    std::vector<RoundData> statistics;

};

}
