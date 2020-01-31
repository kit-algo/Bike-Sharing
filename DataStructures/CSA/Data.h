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
#include <vector>
#include <string>
#include <set>

#include "Entities/Connection.h"
#include "Entities/Stop.h"
#include "Entities/Trip.h"
#include "Entities/Journey.h"

#include "../Intermediate/Data.h"
#include "../Graph/Graph.h"
#include "../Geometry/Rectangle.h"
#include "../GTFS/Entities/Vehicle.h"

#include "../../Helpers/IO/Serialization.h"
#include "../../Helpers/IO/CSVData.h"
#include "../../Helpers/IO/ParserCSV.h"
#include "../../Helpers/String/String.h"
#include "../../Helpers/String/TextFileUtils.h"
#include "../../Helpers/Vector/Vector.h"
#include "../../Helpers/Vector/Permutation.h"
#include "../../Helpers/Console/Progress.h"
#include "../../Helpers/Console/ProgressBar.h"
#include "../../Helpers/FileSystem/FileSystem.h"
#include "../../Helpers/Ranges/Range.h"
#include "../../DataStructures/Container/Map.h"
#include "../../DataStructures/Container/Set.h"
#include "../../Algorithms/Dijkstra/Dijkstra.h"

namespace CSA {

using TransferGraph = ::TransferGraph;

class Data {

public:
    static const std::vector<std::string> stopFileNameAliases;
    static const std::vector<std::string> connectionFileNameAliases;
    static const std::vector<std::string> tripFileNameAliases;
    static const std::vector<std::string> transferFileNameAliases;
    static const std::vector<std::string> zoneFileNameAliases;
    static const std::vector<std::string> zoneTransferFileNameAliases;
    static const std::vector<std::string> demandFileNameAliases;

private:
    Data() :
        implicitDepartureBufferTimes(false),
        implicitArrivalBufferTimes(false) {
    }

public:
    inline static Data FromBinary(const std::string& fileName) noexcept {
        Data data;
        data.deserialize(fileName);
        return data;
    }

    inline static Data FromIntermediate(const Intermediate::Data& inter) noexcept {
        Data data;
        for (const Intermediate::Stop& stop : inter.stops) {
            data.stopData.emplace_back(stop);
        }
        for (const Intermediate::Trip& trip : inter.trips) {
            AssertMsg(!trip.stopEvents.empty(), "Intermediate data contains trip without any stop event!");
            for (size_t i = 1; i < trip.stopEvents.size(); i++) {
                const Intermediate::StopEvent& from = trip.stopEvents[i - 1];
                const Intermediate::StopEvent& to = trip.stopEvents[i];
                data.connections.emplace_back(from.stopId, to.stopId, from.departureTime, to.arrivalTime, TripId(data.tripData.size()));
            }
            data.tripData.emplace_back(trip);
        }
        std::sort(data.connections.begin(), data.connections.end());
        Intermediate::TransferGraph transferGraph = inter.transferGraph;
        Graph::printInfo(transferGraph);
        transferGraph.printAnalysis();
        Graph::move(std::move(transferGraph), data.transferGraph);
        return data;
    }

    template<bool MAKE_BIDIRECTIONAL = true>
    inline static Data FromCSV(const std::string& fileNameBase) noexcept {
        Data data;
        data.readStops(fileNameBase);
        data.readConnections(fileNameBase);
        data.readTrips(fileNameBase);
        data.readTransfers<MAKE_BIDIRECTIONAL>(fileNameBase);
        return data;
    }

    template<bool MAKE_BIDIRECTIONAL = true>
    inline static Data FromCSVwithZones(const std::string& fileNameBase) noexcept {
        Data data;
        data.readStops(fileNameBase);
        data.readConnections(fileNameBase);
        data.readTrips(fileNameBase);
        data.readTransfers<MAKE_BIDIRECTIONAL>(fileNameBase);
        data.readZones(fileNameBase);
        data.readZoneTransfers(fileNameBase);
        return data;
    }

    inline static Data FromPajorCSV(const std::string& fileNameBase) noexcept {
        Data data;
        data.readStops(fileNameBase);
        data.readPajorConnections(fileNameBase);
        data.readTransfers<false>(fileNameBase);
        std::vector<Set<Vertex>> neighbors(data.stopData.size());
        for (const Vertex vertex : data.transferGraph.vertices()) {
            for (const Edge edge : data.transferGraph.edgesFrom(vertex)) {
                const Vertex other = data.transferGraph.get(ToVertex, edge);
                if (data.stopData[vertex].coordinates.x == 0 && data.stopData[vertex].coordinates.y == 0 && data.stopData[other].coordinates.x == 0 && data.stopData[other].coordinates.y == 0) continue;
                neighbors[vertex].insert(other);
                neighbors[other].insert(vertex);
            }
        }
        for (const Connection c : data.connections) {
            if (c.travelTime() > 120) continue;
            if (data.stopData[c.departureStopId].coordinates.x == 0 && data.stopData[c.departureStopId].coordinates.y == 0 && data.stopData[c.arrivalStopId].coordinates.x == 0 && data.stopData[c.arrivalStopId].coordinates.y == 0) continue;
            neighbors[c.departureStopId].insert(c.arrivalStopId);
            neighbors[c.arrivalStopId].insert(c.departureStopId);
        }
        for (const Vertex vertex : data.transferGraph.vertices()) {
            if (data.stopData[vertex].coordinates.x != 0) continue;
            if (data.stopData[vertex].coordinates.y != 0) continue;
            if (neighbors[vertex].empty()) {
                warning("Stop ", vertex ," coordinates cannot be fixed");
            } else {
                for (const Vertex other : neighbors[vertex]) {
                    data.stopData[vertex].coordinates += data.stopData[other].coordinates;
                }
                data.stopData[vertex].coordinates /= neighbors[vertex].size();
                data.transferGraph.set(Coordinates, vertex, data.stopData[vertex].coordinates);
            }
        }
        return data;
    }

    inline static void RepairFiles(const std::string& fileNameBase) noexcept {
        RepairFileNames(fileNameBase);
        RepairHeaders(fileNameBase);
        RepairIDs(fileNameBase);
    }

    inline static void RepairFileNames(const std::string& fileNameBase) noexcept {
        for (size_t i = 1; i < stopFileNameAliases.size(); i++) FileSystem::renameFile(fileNameBase + stopFileNameAliases[i], fileNameBase + stopFileNameAliases.front());
        for (size_t i = 1; i < connectionFileNameAliases.size(); i++) FileSystem::renameFile(fileNameBase + connectionFileNameAliases[i], fileNameBase + connectionFileNameAliases.front());
        for (size_t i = 1; i < tripFileNameAliases.size(); i++) FileSystem::renameFile(fileNameBase + tripFileNameAliases[i], fileNameBase + tripFileNameAliases.front());
        for (size_t i = 1; i < transferFileNameAliases.size(); i++) FileSystem::renameFile(fileNameBase + transferFileNameAliases[i], fileNameBase + transferFileNameAliases.front());
        for (size_t i = 1; i < zoneFileNameAliases.size(); i++) FileSystem::renameFile(fileNameBase + zoneFileNameAliases[i], fileNameBase + zoneFileNameAliases.front());
        for (size_t i = 1; i < zoneTransferFileNameAliases.size(); i++) FileSystem::renameFile(fileNameBase + zoneTransferFileNameAliases[i], fileNameBase + zoneTransferFileNameAliases.front());
        for (size_t i = 1; i < demandFileNameAliases.size(); i++) FileSystem::renameFile(fileNameBase + demandFileNameAliases[i], fileNameBase + demandFileNameAliases.front());
    }

    inline static void RepairHeaders(const std::string& fileNameBase) noexcept {
        std::string stops = TextFile::read(fileNameBase + stopFileNameAliases.front());
        stops = String::replaceAll(stops, "STOPNO[-]", "stop_id");
        stops = String::replaceAll(stops, "TRANSFERTIME", "change_time");
        stops = String::replaceAll(stops, "TRANSFERTIME[SEC]", "change_time");
        stops = String::replaceAll(stops, "NAME[-]", "name");
        stops = String::replaceAll(stops, "XCOORD[-]", "lon");
        stops = String::replaceAll(stops, "YCOORD[-]", "lat");
        TextFile::write(fileNameBase + stopFileNameAliases.front(), stops);
        std::string connections = TextFile::read(fileNameBase + connectionFileNameAliases.front());
        connections = String::replaceAll(connections, "FROMSTOPNO[-]", "dep_stop");
        connections = String::replaceAll(connections, "TOSTOPNO[-]", "arr_stop");
        connections = String::replaceAll(connections, "DEPARTURE[SEC]", "dep_time");
        connections = String::replaceAll(connections, "ARRIVAL[SEC]", "arr_time");
        connections = String::replaceAll(connections, "TRIP_ID[-]", "trip_id");
        TextFile::write(fileNameBase + connectionFileNameAliases.front(), connections);
        std::string trips = TextFile::read(fileNameBase + tripFileNameAliases.front());
        trips = String::replaceAll(trips, "TRIP_NO[-]", "trip_id");
        trips = String::replaceAll(trips, "LINE_NAME[-]", "name");
        trips = String::replaceAll(trips, "T_SYS_CODE[-]", "vehicle");
        trips = String::replaceAll(trips, "LINE_ROUTE_NAME[-]", "line_id");
        TextFile::write(fileNameBase + tripFileNameAliases.front(), trips);
        std::string transfers = TextFile::read(fileNameBase + transferFileNameAliases.front());
        transfers = String::replaceAll(transfers, "FROMSTOPNO[-]", "dep_stop");
        transfers = String::replaceAll(transfers, "TOSTOPNO[-]", "arr_stop");
        transfers = String::replaceAll(transfers, "DURATION[SEC]", "duration");
        TextFile::write(fileNameBase + transferFileNameAliases.front(), transfers);
        std::string zones = TextFile::read(fileNameBase + zoneFileNameAliases.front());
        zones = String::replaceAll(zones, "NO[-]", "zone_id");
        zones = String::replaceAll(zones, "XCOORD[-]", "lon");
        zones = String::replaceAll(zones, "YCOORD[-]", "lat");
        TextFile::write(fileNameBase + zoneFileNameAliases.front(), zones);
        std::string zoneTransfers = TextFile::read(fileNameBase + zoneTransferFileNameAliases.front());
        zoneTransfers = String::replaceAll(zoneTransfers, "FROMZONENO[-]", "zone_id");
        zoneTransfers = String::replaceAll(zoneTransfers, "TOSTOPNO[-]", "stop_id");
        zoneTransfers = String::replaceAll(zoneTransfers, "DURATION[SEC]", "duration");
        TextFile::write(fileNameBase + zoneTransferFileNameAliases.front(), zoneTransfers);
        std::string demand = TextFile::read(fileNameBase + demandFileNameAliases.front());
        demand = String::replaceAll(demand, "FROMZONENO[-]", "dep_zone");
        demand = String::replaceAll(demand, "TOZONENO[-]", "arr_zone");
        demand = String::replaceAll(demand, "MINDEPARTURE[SEC]", "min_dep_time");
        demand = String::replaceAll(demand, "MAXDEPARTURE[SEC]", "max_dep_time");
        demand = String::replaceAll(demand, "DEMAND[-]", "passenger_count");
        TextFile::write(fileNameBase + demandFileNameAliases.front(), demand);
    }

    inline static void RepairIDs(const std::string& fileNameBase) noexcept {
        Map<std::string, std::string> stopIDs;
        Map<std::string, std::string> tripIDs;
        Map<std::string, std::string> zoneIDs;
        IO::CSVData<std::string, IO::TrimChars<>, IO::DoubleQuoteEscape<',','"'>> stops(fileNameBase + stopFileNameAliases.front());
        IO::CSVData<std::string, IO::TrimChars<>, IO::DoubleQuoteEscape<',','"'>> connections(fileNameBase + connectionFileNameAliases.front());
        IO::CSVData<std::string, IO::TrimChars<>, IO::DoubleQuoteEscape<',','"'>> trips(fileNameBase + tripFileNameAliases.front());
        IO::CSVData<std::string, IO::TrimChars<>, IO::DoubleQuoteEscape<',','"'>> transfers(fileNameBase + transferFileNameAliases.front());
        IO::CSVData<std::string, IO::TrimChars<>, IO::DoubleQuoteEscape<',','"'>> zones(fileNameBase + zoneFileNameAliases.front());
        IO::CSVData<std::string, IO::TrimChars<>, IO::DoubleQuoteEscape<',','"'>> zoneTransfers(fileNameBase + zoneTransferFileNameAliases.front());
        IO::CSVData<std::string, IO::TrimChars<>, IO::DoubleQuoteEscape<',','"'>> demand(fileNameBase + demandFileNameAliases.front());
        for (std::string& stopId : stops.getColumn("stop_id")) {
            if (stopIDs.contains(stopId)) {
                error("Repeated stop id: " + stopId + " = " + stopIDs[stopId] + "!");
            } else {
                stopIDs.insert(stopId, std::to_string(stopIDs.size()));
            }
            stopId = stopIDs[stopId];
        }
        for (std::string& tripId : trips.getColumn("trip_id")) {
            if (tripIDs.contains(tripId)) {
                error("Repeated trip id: " + tripId + " = " + tripIDs[tripId] + "!");
            } else {
                tripIDs.insert(tripId, std::to_string(tripIDs.size()));
            }
            tripId = tripIDs[tripId];
        }
        for (std::string& zoneId : zones.getColumn("zone_id")) {
            if (zoneIDs.contains(zoneId)) {
                error("Repeated zone id: " + zoneId + " = " + zoneIDs[zoneId] + "!");
            } else {
                zoneIDs.insert(zoneId, std::to_string(zoneIDs.size()));
            }
            zoneId = zoneIDs[zoneId];
        }
        for (std::string& stopId : connections.getColumn("dep_stop")) {
            if (stopIDs.contains(stopId)) {
                stopId = stopIDs[stopId];
            } else {
                error("Unknown stop id: " + stopId + "!");
            }
        }
        for (std::string& stopId : connections.getColumn("arr_stop")) {
            if (stopIDs.contains(stopId)) {
                stopId = stopIDs[stopId];
            } else {
                error("Unknown stop id: " + stopId + "!");
            }
        }
        for (std::string& stopId : transfers.getColumn("dep_stop")) {
            if (stopIDs.contains(stopId)) {
                stopId = stopIDs[stopId];
            } else {
                error("Unknown stop id: " + stopId + "!");
            }
        }
        for (std::string& stopId : transfers.getColumn("arr_stop")) {
            if (stopIDs.contains(stopId)) {
                stopId = stopIDs[stopId];
            } else {
                error("Unknown stop id: " + stopId + "!");
            }
        }
        for (std::string& stopId : zoneTransfers.getColumn("stop_id")) {
            if (stopIDs.contains(stopId)) {
                stopId = stopIDs[stopId];
            } else {
                error("Unknown stop id: " + stopId + "!");
            }
        }
        for (std::string& tripId : connections.getColumn("trip_id")) {
            if (tripIDs.contains(tripId)) {
                tripId = tripIDs[tripId];
            } else {
                error("Unknown trip id: " + tripId + "!");
            }
        }
        for (std::string& zoneId : zoneTransfers.getColumn("zone_id")) {
            if (zoneIDs.contains(zoneId)) {
                zoneId = zoneIDs[zoneId];
            } else {
                error("Unknown zone id: " + zoneId + "!");
            }
        }
        for (std::string& zoneId : demand.getColumn("dep_zone")) {
            if (zoneIDs.contains(zoneId)) {
                zoneId = zoneIDs[zoneId];
            } else {
                error("Unknown zone id: " + zoneId + "!");
            }
        }
        for (std::string& zoneId : demand.getColumn("arr_zone")) {
            if (zoneIDs.contains(zoneId)) {
                zoneId = zoneIDs[zoneId];
            } else {
                error("Unknown zone id: " + zoneId + "!");
            }
        }
        stops.write(fileNameBase + stopFileNameAliases.front());
        connections.write(fileNameBase + connectionFileNameAliases.front());
        trips.write(fileNameBase + tripFileNameAliases.front());
        transfers.write(fileNameBase + transferFileNameAliases.front());
        zones.write(fileNameBase + zoneFileNameAliases.front());
        zoneTransfers.write(fileNameBase + zoneTransferFileNameAliases.front());
        demand.write(fileNameBase + demandFileNameAliases.front());
    }

    template<bool MAKE_BIDIRECTIONAL = true, typename GRAPH_TYPE>
    inline static Data FromInput(const std::vector<Stop>& stops, const std::vector<Connection>& connections, const std::vector<Trip>& trips, GRAPH_TYPE transferGraph) noexcept {
        AssertMsg(transferGraph.numVertices() >= stops.size(), "Network containes " << stops.size() << " stops, but transfer graph has only " << transferGraph.numVertices() << " vertices!");
        Data data;
        data.stopData = stops;
        data.tripData = trips;
        for (const Connection con : connections) {
            if (con.departureStopId >= data.stopData.size() || data.stopData[con.departureStopId].minTransferTime < 0) continue;
            if (con.arrivalStopId >= data.stopData.size() || data.stopData[con.arrivalStopId].minTransferTime < 0) continue;
            if (con.departureTime > con.arrivalTime) continue;
            if (con.tripId >= data.tripData.size()) continue;
            data.connections.emplace_back(con);
        }
        Intermediate::TransferGraph graph;
        Graph::move(std::move(transferGraph), graph);
        if constexpr (MAKE_BIDIRECTIONAL) graph.makeBidirectional();
        graph.reduceMultiEdgesBy(TravelTime);
        graph.packEdges();
        Graph::move(std::move(graph), data.transferGraph);
        return data;
    }

protected:
    inline void readStops(const std::string& fileNameBase, const bool verbose = true) {
        IO::readFile(stopFileNameAliases, "Stops", [&](){
            size_t count = 0;
            IO::CSVReader<5, IO::TrimChars<>, IO::DoubleQuoteEscape<',','"'>> in(fileNameBase + stopFileNameAliases);
            in.readHeader(alias{"stop_id", "StopId"}, alias{"lon", "Longitude"}, alias{"lat", "Latitude"}, alias{"name", "CommonName"}, alias{"change_time", "min_change_time", "TransferDuration"});
            StopId stopID;
            Stop stop;
            while (in.readRow(stopID, stop.coordinates.longitude, stop.coordinates.latitude, stop.name, stop.minTransferTime)) {
                if (stopID >= stopData.size()) stopData.resize(stopID + 1, Stop("NOT_NAMED", Geometry::Point(), -1));
                stopData[stopID] = stop;
                count++;
            }
            return count;
        }, verbose);
    }

    inline void readConnections(const std::string& fileNameBase, const bool verbose = true) {
        IO::readFile(connectionFileNameAliases, "Connections", [&](){
            size_t count = 0;
            IO::CSVReader<5, IO::TrimChars<>, IO::DoubleQuoteEscape<',','"'>> in(fileNameBase + connectionFileNameAliases);
            in.readHeader("dep_stop", "arr_stop", "dep_time", "arr_time", "trip_id");
            Connection con;
            while (in.readRow(con.departureStopId, con.arrivalStopId, con.departureTime, con.arrivalTime, con.tripId)) {
                if (con.departureStopId >= stopData.size() || stopData[con.departureStopId].minTransferTime < 0) continue;
                if (con.arrivalStopId >= stopData.size() || stopData[con.arrivalStopId].minTransferTime < 0) continue;
                if (con.tripId >= tripData.size()) tripData.resize(con.tripId + 1, Trip("NOT_NAMED", "NOT_NAMED", -2));
                connections.emplace_back(con);
                count++;
            }
            return count;
        }, verbose);
    }

    inline void readPajorConnections(const std::string& fileNameBase, const bool verbose = true) {
        IO::readFile(connectionFileNameAliases, "Connections", [&](){
            std::vector<int> types = {
                GTFS::Type::Subway,
                GTFS::Type::Gondola,
                GTFS::Type::Gondola,
                GTFS::Type::Bus,
                GTFS::Type::Funicular,
                GTFS::Type::Funicular,
                GTFS::Type::CableCar,
                GTFS::Type::CableCar,
                GTFS::Type::Funicular,
                GTFS::Type::Funicular,
                GTFS::Type::Bus,
                GTFS::Type::Rail,
                GTFS::Type::Ferry,
                GTFS::Type::Ferry,
                GTFS::Type::Tram
            };
            size_t count = 0;
            IO::CSVReader<8, IO::TrimChars<>, IO::DoubleQuoteEscape<',','"'>> in(fileNameBase + connectionFileNameAliases);
            in.readHeader("TripId", "FromStopId", "ToStopId", "DepartureTime", "Duration", "LineId", "SuperLineName", "VehicleTypeId");
            std::string tripName;
            std::string routeName;
            int pajorType;
            Connection con;
            while (in.readRow(con.tripId, con.departureStopId, con.arrivalStopId, con.departureTime, con.arrivalTime, tripName, routeName, pajorType)) {
                if (con.departureStopId >= stopData.size() || stopData[con.departureStopId].minTransferTime < 0) continue;
                if (con.arrivalStopId >= stopData.size() || stopData[con.arrivalStopId].minTransferTime < 0) continue;
                if (pajorType < 0 || pajorType >= static_cast<int>(types.size())) pajorType = 11;
                if (con.tripId >= tripData.size()) tripData.resize(con.tripId + 1, Trip("NOT_NAMED", "NOT_NAMED", -2));
                if (tripData[con.tripId].type == -2) tripData[con.tripId] = Trip(tripName, routeName, types[pajorType]);
                con.arrivalTime = con.arrivalTime + con.departureTime;
                connections.emplace_back(con);
                count++;
            }
            return count;
        }, verbose);
        sanitizeTrips();
    }

    inline void readTrips(const std::string& fileNameBase, const bool verbose = true) {
        IO::readFile(tripFileNameAliases, "Trips", [&](){
            size_t count = 0;
            IO::CSVReader<4, IO::TrimChars<>, IO::DoubleQuoteEscape<',','"'>> in(fileNameBase + tripFileNameAliases);
            in.readHeader(IO::IGNORE_EXTRA_COLUMN | IO::IGNORE_MISSING_COLUMN, "trip_id", "name", "vehicle", "line_id");
            TripId tripID;
            std::string tripName = "NOT_NAMED";
            std::string type = "train";
            std::string route = "NOT_NAMED";
            while (in.readRow(tripID, tripName, type, route)) {
                if (tripID >= tripData.size()) continue;
                tripData[tripID] = Trip(tripName, route, GTFS::Type::Rail);
                type = String::toLower(type);
                if (type == "b") tripData[tripID].type = GTFS::Type::Bus;
                if (type == "bus") tripData[tripID].type = GTFS::Type::Bus;
                if (type == "s") tripData[tripID].type = GTFS::Type::Tram;
                if (type == "str") tripData[tripID].type = GTFS::Type::Tram;
                if (type == "stb") tripData[tripID].type = GTFS::Type::Tram;
                if (type == "air") tripData[tripID].type = GTFS::Type::Tram;
                if (type == "u") tripData[tripID].type = GTFS::Type::Subway;
                if (type == "underground") tripData[tripID].type = GTFS::Type::Subway;
                if (type == "subway") tripData[tripID].type = GTFS::Type::Subway;
                //if (type == "U70") tripData[tripID].type = GTFS::Type::Subway;
                //if (type == "UBU") tripData[tripID].type = GTFS::Type::Subway;
                count++;
            }
            return count;
        }, verbose);
        sanitizeTrips();
    }

    inline void sanitizeTrips() {
        Permutation permutation(tripData.size());
        size_t tripCount = 0;
        for (size_t i = 0; i < tripData.size(); i++) {
            if (tripData[i].type != -2) {
                permutation[i] = tripCount;
                tripCount++;
            } else {
                permutation[i] = tripData.size() - i + tripCount - 1;
            }
        }
        if (tripCount < tripData.size()) {
            permutation.permutate(tripData);
            tripData.resize(tripCount);
            for (Connection& con : connections) {
                con.tripId = permutation.permutate(con.tripId);
                AssertMsg(con.tripId < tripCount, "Connection belongs to trip without trip data! (" << con << ", number of trips: " << tripCount << ")");
            }
        }
    }

    template<bool MAKE_BIDIRECTIONAL = true>
    inline void readTransfers(const std::string& fileNameBase, const bool verbose = true) {
        Intermediate::TransferGraph graph;
        graph.addVertices(stopData.size());
        for (const Vertex vertex : graph.vertices()) {
            graph.set(Coordinates, vertex, stopData[vertex].coordinates);
        }
        IO::readFile(transferFileNameAliases, "Transfers", [&](){
            size_t count = 0;
            IO::CSVReader<3, IO::TrimChars<>, IO::DoubleQuoteEscape<',','"'>> in(fileNameBase + transferFileNameAliases);
            in.readHeader(alias{"dep_stop", "DepartureStopId"}, alias{"arr_stop", "ArrivalStopId"}, alias{"duration", "Duration"});
            Vertex from;
            Vertex to;
            int travelTime;
            while (in.readRow(from, to, travelTime)) {
                if (!graph.isVertex(from) || stopData[from].minTransferTime == -1) continue;
                if (!graph.isVertex(to) || stopData[to].minTransferTime == -1) continue;
                if (from == to && isStop(from)) {
                    stopData[from].minTransferTime = std::max(stopData[from].minTransferTime, travelTime);
                } else {
                    graph.addEdge(from, to).set(TravelTime, travelTime);
                    if constexpr (MAKE_BIDIRECTIONAL) graph.addEdge(to, from).set(TravelTime, travelTime);
                }
                count++;
            }
            return count;
        }, verbose);
        graph.reduceMultiEdgesBy(TravelTime);
        graph.packEdges();
        Graph::move(std::move(graph), transferGraph);
    }

    inline void readZones(const std::string& fileNameBase, const bool verbose = true) {
        IO::readFile(zoneFileNameAliases, "Zones", [&](){
            size_t count = 0;
            IO::CSVReader<3, IO::TrimChars<>, IO::DoubleQuoteEscape<',','"'>> in(fileNameBase + zoneFileNameAliases);
            in.readHeader("zone_id", "lon", "lat");
            Vertex zoneID;
            Geometry::Point coordinates;
            while (in.readRow(zoneID, coordinates.longitude, coordinates.latitude)) {
                zoneID += stopData.size();
                transferGraph.addVertices(zoneID - transferGraph.numVertices() + 1);
                transferGraph.set(Coordinates, zoneID, coordinates);
                count++;
            }
            return count;
        }, verbose);
    }

    inline void readZoneTransfers(const std::string& fileNameBase, const bool verbose = true) {
        Intermediate::TransferGraph graph;
        Graph::move(std::move(transferGraph), graph);
        IO::readFile(zoneTransferFileNameAliases, "ZoneTransfers", [&](){
            size_t count = 0;
            IO::CSVReader<3, IO::TrimChars<>, IO::DoubleQuoteEscape<',','"'>> in(fileNameBase + zoneTransferFileNameAliases);
            in.readHeader("zone_id", "stop_id", "duration");
            Vertex zoneID;
            StopId stopId;
            int travelTime;
            while (in.readRow(zoneID, stopId, travelTime)) {
                zoneID += stopData.size();
                if (!graph.isVertex(zoneID)) continue;
                if (!graph.isVertex(stopId) || stopData[stopId].minTransferTime == -1) continue;
                graph.addEdge(zoneID, stopId).set(TravelTime, travelTime);
                graph.addEdge(stopId, zoneID).set(TravelTime, travelTime);
                count++;
            }
            return count;
        }, verbose);
        graph.reduceMultiEdgesBy(TravelTime);
        graph.packEdges();
        Graph::move(std::move(graph), transferGraph);
    }

public:
    inline void useImplicitDepartureBufferTimes() noexcept {
        warning("Implicit buffer times are deprecated!");
        if (implicitDepartureBufferTimes | implicitArrivalBufferTimes) return;
        for (Connection& connection : connections) {
            connection.departureTime -= minTransferTime(connection.departureStopId);
        }
        implicitDepartureBufferTimes = true;
    }

    inline void dontUseImplicitDepartureBufferTimes() noexcept {
        if (!implicitDepartureBufferTimes) return;
        for (Connection& connection : connections) {
            connection.departureTime += minTransferTime(connection.departureStopId);
        }
        implicitDepartureBufferTimes = false;
    }

    inline void useImplicitArrivalBufferTimes() noexcept {
        warning("Implicit buffer times are deprecated!");
        if (implicitDepartureBufferTimes | implicitArrivalBufferTimes) return;
        for (Connection& connection : connections) {
            connection.arrivalTime += minTransferTime(connection.arrivalStopId);
        }
        implicitArrivalBufferTimes = true;
    }

    inline void dontUseImplicitArrivalBufferTimes() noexcept {
        if (!implicitArrivalBufferTimes) return;
        for (Connection& connection : connections) {
            connection.arrivalTime -= minTransferTime(connection.arrivalStopId);
        }
        implicitArrivalBufferTimes = false;
    }

    inline bool hasImplicitBufferTimes() const noexcept {return implicitDepartureBufferTimes | implicitArrivalBufferTimes;}

    inline size_t numberOfStops() const noexcept {return stopData.size();}
    inline bool isStop(const Vertex stop) const noexcept {return stop < numberOfStops();}
    inline Range<StopId> stops() const noexcept {return Range<StopId>(StopId(0), StopId(numberOfStops()));}

    inline size_t numberOfTrips() const noexcept {return tripData.size();}
    inline bool isTrip(const TripId tripId) const noexcept {return tripId < numberOfTrips();}
    inline Range<TripId> tripIds() const noexcept {return Range<TripId>(TripId(0), TripId(numberOfTrips()));}

    inline size_t numberOfConnections() const noexcept {return connections.size();}
    inline bool isConnection(const ConnectionId connectionId) const noexcept {return connectionId < numberOfConnections();}
    inline Range<ConnectionId> connectionIds() const noexcept {return Range<ConnectionId>(ConnectionId(0), ConnectionId(numberOfConnections()));}

    inline int minTransferTime(const StopId stop) const noexcept {return stopData[stop].minTransferTime;}

    inline Geometry::Rectangle boundingBox() const noexcept {
        Geometry::Rectangle result = Geometry::Rectangle::Empty();
        for (const Stop& stop : stopData) {
            result.extend(stop.coordinates);
        }
        return result;
    }

    inline void sortConnectionsAscendingByDepartureTime() noexcept {
        std::stable_sort(connections.begin(), connections.end(), [](const Connection& a, const Connection& b){
            return a.departureTime < b.departureTime;
        });
    }

    inline void sortConnectionsAscendingByArrivalTime() noexcept {
        std::stable_sort(connections.begin(), connections.end(), [](const Connection& a, const Connection& b){
            return a.arrivalTime < b.arrivalTime;
        });
    }

    inline void sortConnectionsDescendingByDepartureTime() noexcept {
        std::stable_sort(connections.begin(), connections.end(), [](const Connection& a, const Connection& b){
            return a.departureTime > b.departureTime;
        });
    }

    inline void sortConnectionsDescendingByArrivalTime() noexcept {
        std::stable_sort(connections.begin(), connections.end(), [](const Connection& a, const Connection& b){
            return a.arrivalTime > b.arrivalTime;
        });
    }

    inline void sortUnique() noexcept {
        std::sort(connections.begin(), connections.end(), [](const Connection& a, const Connection& b){
            return std::make_tuple(a.departureTime, a.arrivalTime, a.departureStopId, a.arrivalStopId, a.tripId) < std::make_tuple(b.departureTime, b.arrivalTime, b.departureStopId, b.arrivalStopId, b.tripId);
        });
    }

    inline TransferGraph minTravelTimeGraph() const noexcept {
        DynamicTransferGraph graph;
        graph.addVertices(transferGraph.numVertices());
        for (Vertex vertex : transferGraph.vertices()) {
            graph.set(Coordinates, vertex, transferGraph.get(Coordinates, vertex));
            for (Edge edge : transferGraph.edgesFrom(vertex)) {
                if (vertex == transferGraph.get(ToVertex, edge)) continue;
                const Edge newEdge = graph.findEdge(vertex, transferGraph.get(ToVertex, edge));
                if (graph.isEdge(newEdge)) {
                    graph.set(TravelTime, newEdge, std::min(graph.get(TravelTime, newEdge), transferGraph.get(TravelTime, edge)));
                } else {
                    graph.addEdge(vertex, transferGraph.get(ToVertex, edge)).set(TravelTime, transferGraph.get(TravelTime, edge));
                }
                const Edge newEdgeB = graph.findEdge(transferGraph.get(ToVertex, edge), vertex);
                if (graph.isEdge(newEdgeB)) {
                    graph.set(TravelTime, newEdgeB, std::min(graph.get(TravelTime, newEdgeB), transferGraph.get(TravelTime, edge)));
                } else {
                    graph.addEdge(transferGraph.get(ToVertex, edge), vertex).set(TravelTime, transferGraph.get(TravelTime, edge));
                }
            }
        }
        for (const Connection connection : connections) {
            const Edge newEdge = graph.findEdge(connection.departureStopId, connection.arrivalStopId);
            if (graph.isEdge(newEdge)) {
                graph.set(TravelTime, newEdge, std::min(graph.get(TravelTime, newEdge), connection.travelTime()));
            } else {
                graph.addEdge(connection.departureStopId, connection.arrivalStopId).set(TravelTime,  connection.travelTime());
            }
            const Edge newEdgeB = graph.findEdge(connection.arrivalStopId, connection.departureStopId);
            if (graph.isEdge(newEdgeB)) {
                graph.set(TravelTime, newEdgeB, std::min(graph.get(TravelTime, newEdgeB), connection.travelTime()));
            } else {
                graph.addEdge(connection.arrivalStopId, connection.departureStopId).set(TravelTime,  connection.travelTime());
            }
        }
        graph.deleteEdges([&](const Edge edge){return graph.get(TravelTime, edge) > 1200;});
        TransferGraph result;
        Graph::move(std::move(graph), result);
        return result;
    }

    inline std::vector<uint32_t> numberOfNeighborsOfStop() const noexcept {
        std::vector<Set<StopId>> sourcesOfStop(numberOfStops());
        std::vector<Set<StopId>> targetsOfStop(numberOfStops());
        for (const StopId stop : stops()) {
            sourcesOfStop[stop].insert(stop);
            for (const Edge edge : transferGraph.edgesFrom(stop)) {
                sourcesOfStop[stop].insert(StopId(transferGraph.get(ToVertex, edge)));
                targetsOfStop[stop].insert(StopId(transferGraph.get(ToVertex, edge)));
            }
        }
        for (const Connection& connection : connections) {
            for (const StopId source : sourcesOfStop[connection.departureStopId]) {
                targetsOfStop[source].insert(connection.arrivalStopId);
            }
        }
        std::vector<uint32_t> result;
        for (const StopId stop : stops()) {
            result.emplace_back(targetsOfStop[stop].size());
        }
        return result;
    }

    inline std::vector<uint32_t> numberOfConnectionsOfStop() const noexcept {
        std::vector<Set<StopId>> sourcesOfStop(numberOfStops());
        for (const StopId stop : stops()) {
            sourcesOfStop[stop].insert(stop);
            for (const Edge edge : transferGraph.edgesFrom(stop)) {
                sourcesOfStop[stop].insert(StopId(transferGraph.get(ToVertex, edge)));
            }
        }
        std::vector<uint32_t> result(numberOfStops(), 0);
        for (const Connection& connection : connections) {
            for (const StopId source : sourcesOfStop[connection.departureStopId]) {
                result[source] += 1;
            }
        }
        return result;
    }

    inline std::vector<double> importanceOfStop() const noexcept {
        std::vector<Set<StopId>> targetsOfStop(numberOfStops());
        for (const StopId stop : stops()) {
            for (const Edge edge : transferGraph.edgesFrom(stop)) {
                if (transferGraph.get(TravelTime, edge) > 600) continue;
                targetsOfStop[stop].insert(StopId(transferGraph.get(ToVertex, edge)));
            }
        }
        std::vector<double> connectionsOfStop(numberOfStops(), 0);
        for (const Connection& connection : connections) {
            targetsOfStop[connection.departureStopId].insert(connection.arrivalStopId);
            connectionsOfStop[connection.departureStopId] += 1;
        }
        for (const StopId stop : stops()) {
            connectionsOfStop[stop] *= targetsOfStop[stop].size();
        }
        std::vector<double> result = connectionsOfStop;
        for (const StopId stop : stops()) {
            for (const Edge edge : transferGraph.edgesFrom(stop)) {
                if (transferGraph.get(TravelTime, edge) > 600) continue;
                result[stop] += connectionsOfStop[transferGraph.get(ToVertex, edge)] / (1.0 + (transferGraph.get(TravelTime, edge) / 60.0));
            }
        }
        for (const StopId stop : stops()) {
            for (const Edge edge : transferGraph.edgesFrom(stop)) {
                if (transferGraph.get(TravelTime, edge) > 600) continue;
                if (result[stop] > result[transferGraph.get(ToVertex, edge)]) {
                    result[transferGraph.get(ToVertex, edge)] = 0;
                } else {
                    result[stop] = 0;
                }
            }
        }
        return result;
    }

    inline bool isCombinable(const Vertex source, const int departureTime, const Vertex target, const int arrivalTime = intMax) const noexcept {
        AssertMsg(transferGraph.isVertex(source), "Source vertex id " << source << " does not represent a vertex!");
        AssertMsg(transferGraph.isVertex(target), "Target vertex id " << target << " does not represent a vertex!");
        if (source == target) {
            return departureTime <= arrivalTime;
        } else {
            Edge transferEdge = transferGraph.findEdge(source, target);
            if (!transferGraph.isEdge(transferEdge)) return false;
            return departureTime + transferGraph.get(TravelTime, transferEdge) <= arrivalTime;
        }
    }

    template<bool APPLY_MIN_TRANSFER_TIME>
    inline bool isCombinable(const StopId source, const int departureTime, const StopId target, const int arrivalTime = intMax) const noexcept {
        AssertMsg(isStop(source), "Source vertex id " << source << " does not represent a stop!");
        AssertMsg(isStop(target), "Target vertex id " << target << " does not represent a stop!");
        if constexpr (APPLY_MIN_TRANSFER_TIME) {
            if (source == target) {
                return departureTime + minTransferTime(source) <= arrivalTime;
            }
        }
        return isCombinable(source, departureTime, target, arrivalTime);
    }

    inline bool isCombinable(const Connection& first, const Connection& second) const noexcept {
        if (first.arrivalTime > second.departureTime) return false;
        if (first.tripId == second.tripId) return true;
        return isCombinable<true>(first.arrivalStopId, first.arrivalTime, second.departureStopId, second.departureTime);
    }

    inline bool isCombinable(const Vertex source, const int departureTime, const Connection& second) const noexcept {
        return isCombinable(source, departureTime, second.departureStopId, second.departureTime);
    }

    template<bool APPLY_MIN_TRANSFER_TIME>
    inline bool isCombinable(const StopId source, const int departureTime, const Connection& second) const noexcept {
        return isCombinable<APPLY_MIN_TRANSFER_TIME>(source, departureTime, second.departureStopId, second.departureTime);
    }

    inline bool isCombinable(const Connection& first, const StopId target, const int arrivalTime = intMax) const noexcept {
        return isCombinable<true>(first.arrivalStopId, first.arrivalTime, target, arrivalTime);
    }

    inline bool isCombinable(const Connection& first, const Vertex target, const int arrivalTime = intMax) const noexcept {
        return isCombinable(first.arrivalStopId, first.arrivalTime, target, arrivalTime);
    }


    inline void makeUndirectedTransitiveStopGraph(const bool verbose = false) noexcept {
        Intermediate::TransferGraph graph;
        graph.addVertices(transferGraph.numVertices());
        for (const Vertex from : transferGraph.vertices()) {
            graph.set(Coordinates, from, transferGraph.get(Coordinates, from));
            for (const Edge edge : transferGraph.edgesFrom(from)) {
                const Vertex to = transferGraph.get(ToVertex, edge);
                if (to >= stopData.size()) continue;
                graph.addEdge(from, to).set(TravelTime, transferGraph.get(TravelTime, edge));
            }
        }
        graph.packEdges();
        Graph::move(std::move(graph), transferGraph);
        graph.clear();
        graph.addVertices(transferGraph.numVertices());
        Dijkstra<TransferGraph, false> dijkstra(transferGraph, transferGraph[TravelTime]);
        Progress progress(transferGraph.numVertices(), verbose);
        for (const Vertex v : transferGraph.vertices()) {
            graph.set(Coordinates, v, transferGraph.get(Coordinates, v));
            dijkstra.run(v, noVertex, [&](const Vertex u) {
                if (u >= v) return;
                const int travelTime = dijkstra.getDistance(u);
                graph.addEdge(v, u).set(TravelTime, travelTime);
                graph.addEdge(u, v).set(TravelTime, travelTime);
            });
            progress++;
        }
        graph.packEdges();
        Graph::move(std::move(graph), transferGraph);
    }

    inline void makeDirectedTransitiveStopGraph(const bool verbose = false) noexcept {
        Intermediate::TransferGraph toZones;
        Intermediate::TransferGraph fromZones;
        Intermediate::TransferGraph newTransferGraph;
        toZones.addVertices(transferGraph.numVertices());
        fromZones.addVertices(transferGraph.numVertices());
        newTransferGraph.addVertices(transferGraph.numVertices());
        for (const Vertex from : transferGraph.vertices()) {
            newTransferGraph.set(Coordinates, from, transferGraph.get(Coordinates, from));
            for (const Edge edge : transferGraph.edgesFrom(from)) {
                const Vertex to = transferGraph.get(ToVertex, edge);
                const int travelTime = transferGraph.get(TravelTime, edge);
                if (from < stopData.size()) toZones.addEdge(from, to).set(TravelTime, travelTime);
                if (to < stopData.size()) fromZones.addEdge(from, to).set(TravelTime, travelTime);
            }
        }
        if (stopData.size() > 0) {
            toZones.packEdges();
            Dijkstra<Intermediate::TransferGraph, false> dijkstra(toZones, toZones[TravelTime]);
            Progress progress(stopData.size(), verbose);
            for (const StopId stop : stops()) {
                dijkstra.run(stop, noVertex, [&](const Vertex u) {
                    if (u != stop) newTransferGraph.addEdge(stop, u).set(TravelTime, dijkstra.getDistance(u));
                });
                progress++;
            }
        }
        if (transferGraph.numVertices() > stopData.size()) {
            fromZones.packEdges();
            Dijkstra<Intermediate::TransferGraph, false> dijkstra(fromZones, fromZones[TravelTime]);
            Progress progress(transferGraph.numVertices() - stopData.size(), verbose);
            for (Vertex v = Vertex(stopData.size()); v < transferGraph.numVertices(); v++) {
                dijkstra.run(v, noVertex, [&](const Vertex u) {
                    if (u != v) newTransferGraph.addEdge(v, u).set(TravelTime, dijkstra.getDistance(u));
                });
                progress++;
            }
        }
        Dijkstra<TransferGraph, false> dijkstraToZones(transferGraph, transferGraph[TravelTime]);
        newTransferGraph.packEdges();
        Graph::move(std::move(newTransferGraph), transferGraph);
    }

    inline void duplicateConnections(const int timeOffset = 24 * 60 * 60) noexcept {
        const size_t oldConnectionCount = connections.size();
        const size_t oldTripCount = tripData.size();
        for (size_t i = 0; i < oldConnectionCount; i++) {
            connections.emplace_back(connections[i], timeOffset, oldTripCount);
        }
        for (size_t i = 0; i < oldTripCount; i++) {
            tripData.emplace_back(tripData[i]);
        }
    }

    inline std::vector<int> numberOfNeighborStopsByStop() const noexcept {
        std::vector<int> result(numberOfStops(), 0);
        for (const StopId stop : stops()) {
            for (const Edge edge : transferGraph.edgesFrom(stop)) {
                if (isStop(transferGraph.get(ToVertex, edge))) {
                    result[stop]++;
                }
            }
        }
        return result;
    }

    inline void applyMinTravelTime(const double minTravelTime) noexcept {
        for (const Vertex from : transferGraph.vertices()) {
            for (const Edge edge : transferGraph.edgesFrom(from)) {
                if (transferGraph.get(TravelTime, edge) < minTravelTime) {
                    transferGraph.set(TravelTime, edge, minTravelTime);
                }
            }
        }
    }

public:
    inline const std::vector<Geometry::Point>& getCoordinates() const noexcept {
        return transferGraph[Coordinates];
    }

    inline std::string journeyToShortText(const std::vector<int>& connectionList) const noexcept {
        if (connectionList.empty()) return "";
        std::stringstream text;
        TripId trip = connections[connectionList.front()].tripId;
        text << stopData[connections[connectionList.front()].departureStopId].name << "[" << connections[connectionList.front()].departureStopId << "] -> ";
        text << tripData[trip].tripName << "[" << trip << "] -> ";
        for (size_t i = 1; i < connectionList.size(); i++) {
            if (connections[connectionList[i]].tripId == trip) continue;
            trip = connections[connectionList[i]].tripId;
            text << stopData[connections[connectionList[i - 1]].arrivalStopId].name << "[" << connections[connectionList[i - 1]].arrivalStopId << "] -> ";
            if (connections[connectionList[i]].departureStopId != connections[connectionList[i - 1]].arrivalStopId) {
                text << stopData[connections[connectionList[i]].departureStopId].name << "[" << connections[connectionList[i]].departureStopId << "] -> ";
            }
            text << tripData[trip].tripName << "[" << trip << "] -> ";
        }
        text << stopData[connections[connectionList.back()].arrivalStopId].name << "[" << connections[connectionList.front()].arrivalStopId << "]";
        return text.str();
    }

    inline std::vector<std::string> journeyToText(const Journey& journey) const noexcept {
        std::vector<std::string> text;
        for (const JourneyLeg& leg : journey) {
            std::stringstream line;
            if (leg.usesTrip) {
                line << "Take " << GTFS::TypeNames[tripData[leg.tripId].type];
                line << ": " << tripData[leg.tripId].routeName << "(" << tripData[leg.tripId].tripName << ")" << "[" << leg.tripId << "] ";
                line << "from " << stopData[leg.from].name << "[" << leg.from << "] ";
                line << "departing at " << String::secToTime(leg.departureTime) << "[" << leg.departureTime << "] ";
                line << "to " << stopData[leg.to].name << "[" << leg.to << "] ";
                line << "arrive at " << String::secToTime(leg.arrivalTime) << "[" << leg.arrivalTime << "];";
            } else if (leg.from == leg.to) {
                line << "Wait at " << stopData[leg.from].name << " [" << leg.from << "], ";
                line << "minimal waiting time: " << String::secToString(leg.arrivalTime - leg.departureTime) << ".";
            } else {
                line << "Walk from " << (isStop(leg.from) ? stopData[leg.from].name : "Vertex") << " [" << leg.from << "] ";
                line << "to " << (isStop(leg.to) ? stopData[leg.to].name : "Vertex") << " [" << leg.to << "], ";
                line << "start at " << String::secToTime(leg.departureTime) << " [" << leg.departureTime << "] ";
                line << "and arrive at " << String::secToTime(leg.arrivalTime) << " [" << leg.arrivalTime << "] ";
                line << "(" << String::secToString(leg.arrivalTime - leg.departureTime) << ").";
            }
            text.emplace_back(line.str());
        }
        return text;
    }

    inline std::string journeyToText(const std::vector<int>& connectionList) const noexcept {
        std::stringstream text;
        TripId currentTripIndex = TripId(0);
        TripId nextTripIndex = TripId(0);
        while (currentTripIndex < connectionList.size()) {
            while (nextTripIndex < connectionList.size() && connections[connectionList[currentTripIndex]].tripId == connections[connectionList[nextTripIndex]].tripId) {
                nextTripIndex++;
            }
            Connection c1 = connections[connectionList[currentTripIndex]];
            Connection c2 = connections[connectionList[nextTripIndex - 1]];
            text << "Take " << GTFS::TypeNames[tripData[c1.tripId].type];
            text << ": " << tripData[c1.tripId].routeName << "(" << tripData[c1.tripId].tripName << ")" << "[" << c1.tripId << "] ";
            text << "from " << stopData[c1.departureStopId].name << "[" << c1.departureStopId << "] ";
            text << "departing at " << String::secToTime(c1.departureTime) << "[" << c1.departureTime << "] ";
            text << "to " << stopData[c2.arrivalStopId].name << "[" << c2.arrivalStopId << "] ";
            text << "arrive at " << String::secToTime(c2.arrivalTime) << "[" << c2.arrivalTime << "];";
            if (nextTripIndex < connectionList.size()) {
                text << " ";
                Connection c3 = connections[connectionList[nextTripIndex]];
                if (c2.arrivalStopId == c3.departureStopId) {
                    text << "Wait at " << stopData[c2.arrivalStopId].name << " [" << c2.arrivalStopId << "], for " << String::secToString(c3.departureTime - c2.arrivalTime) << "; ";
                } else {
                    text << "Walk from " << stopData[c2.arrivalStopId].name << "[" << c2.arrivalStopId << "] ";
                    text << "to " << stopData[c3.departureStopId].name << "[" << c3.departureStopId << "] ";
                    text << "in " << String::secToString(c3.departureTime - c2.arrivalTime) << "; ";
                }
            }
            currentTripIndex = nextTripIndex;
        }
        return text.str();
    }

    inline int numberOfTransfers(const std::vector<int>& connectionList) const noexcept {
        if (connectionList.empty()) return 0;
        int result = 0;
        for (size_t i = 0; i < connectionList.size() - 1; i++) {
            if (connections[connectionList[i]].tripId != connections[connectionList[i + 1]].tripId) {
                result++;
            }
        }
        return result;
    }

    inline int waitingTime(const std::vector<int>& connectionList) const noexcept {
        if (connectionList.empty()) return 0;
        int result = 0;
        for (size_t i = 0; i < connectionList.size() - 1; i++) {
            if (connections[connectionList[i]].tripId != connections[connectionList[i + 1]].tripId) {
                result += connections[connectionList[i + 1]].departureTime - connections[connectionList[i]].arrivalTime;
            }
        }
        return result;
    }

    inline int travelTime(const std::vector<int>& connectionList) const noexcept {
        if (connectionList.empty()) return 0;
        return connections[connectionList.back()].arrivalTime - connections[connectionList.front()].departureTime;
    }

    inline void printInfo(const bool characterizeGraph = false) const noexcept {
        int firstDay = std::numeric_limits<int>::max();
        int lastDay = std::numeric_limits<int>::min();
        std::vector<int> departuresByStop(numberOfStops(), 0);
        std::vector<int> arrivalsByStop(numberOfStops(), 0);
        std::vector<int> connectionsByStop(numberOfStops(), 0);
        for (const Connection& connection : connections) {
            if (firstDay > connection.departureTime) firstDay = connection.departureTime;
            if (lastDay < connection.arrivalTime) lastDay = connection.arrivalTime;
            departuresByStop[connection.departureStopId]++;
            arrivalsByStop[connection.arrivalStopId]++;
            connectionsByStop[connection.departureStopId]++;
            connectionsByStop[connection.arrivalStopId]++;
        }
        size_t numberOfIsolatedStops = 0;
        for (const StopId stop : stops()) {
            if (transferGraph.outDegree(stop) == 0) numberOfIsolatedStops++;
        }
        std::cout << "CSA public transit data:" << std::endl;
        std::cout << "   Number of Stops:           " << std::setw(12) << String::prettyInt(numberOfStops()) << std::endl;
        std::cout << "   Number of Isolated Stops:  " << std::setw(12) << String::prettyInt(numberOfIsolatedStops) << std::endl;
        std::cout << "   Number of Trips:           " << std::setw(12) << String::prettyInt(numberOfTrips()) << std::endl;
        std::cout << "   Number of Stop Events:     " << std::setw(12) << String::prettyInt(numberOfConnections() + numberOfTrips()) << std::endl;
        std::cout << "   Number of Connections:     " << std::setw(12) << String::prettyInt(numberOfConnections()) << std::endl;
        std::cout << "   Number of Vertices:        " << std::setw(12) << String::prettyInt(transferGraph.numVertices()) << std::endl;
        std::cout << "   Number of Edges:           " << std::setw(12) << String::prettyInt(transferGraph.numEdges()) << std::endl;
        std::cout << "   Stops without departures:  " << std::setw(12) << String::prettyInt(Vector::count(departuresByStop, 0)) << std::endl;
        std::cout << "   Stops without arrivals:    " << std::setw(12) << String::prettyInt(Vector::count(arrivalsByStop, 0)) << std::endl;
        std::cout << "   Stops without connections: " << std::setw(12) << String::prettyInt(Vector::count(connectionsByStop, 0)) << std::endl;
        std::cout << "   First Day:                 " << std::setw(12) << String::prettyInt(firstDay / (60 * 60 * 24)) << std::endl;
        std::cout << "   Last Day:                  " << std::setw(12) << String::prettyInt(lastDay / (60 * 60 * 24)) << std::endl;
        std::cout << "   First Departure:           " << std::setw(12) << String::secToTime(firstDay) << std::endl;
        std::cout << "   Last Arrival:              " << std::setw(12) << String::secToTime(lastDay) << std::endl;
        std::cout << "   Bounding Box:              " << std::setw(12) << boundingBox() << std::endl;
        if (!characterizeGraph) return;
        std::cout << "   Transfer graph:            " << std::setw(12) << Graph::characterize(transferGraph) << std::endl;
        if (transferGraph.numVertices() > numberOfStops()) {
            DynamicTransferGraph stopGraph;
            Graph::copy(transferGraph, stopGraph);
            stopGraph.deleteVertices([&](const Vertex v){return v >= numberOfStops();});
            std::cout << "   Stop graph:                " << std::setw(12) << Graph::characterize(stopGraph) << std::endl;
        }
    }

    inline void serialize(const std::string& fileName) const noexcept {
        IO::serialize(fileName, connections, stopData, tripData);
        transferGraph.writeBinary(fileName + ".graph");
    }

    inline void deserialize(const std::string& fileName) noexcept {
        IO::deserialize(fileName, connections, stopData, tripData);
        transferGraph.readBinary(fileName + ".graph");
    }

    inline void writeCSV(const std::string& fileNameBase) const noexcept {
        writeVectorCSV(fileNameBase + stopFileNameAliases.front(), stopData, "stop_id");
        writeVectorCSV(fileNameBase + connectionFileNameAliases.front(), connections, "connection_id");
        writeVectorCSV(fileNameBase + tripFileNameAliases.front(), tripData, "trip_id");
        writeTransferCSV(fileNameBase + transferFileNameAliases.front());
    }

    inline void writeExchangeFormat(const std::string& fileNameBase) const noexcept {
        writeExchangeFormatConnections(fileNameBase + "connections.csv");
        writeExchangeFormatTrips(fileNameBase + "trips.csv");
        writeExchangeFormatStops(fileNameBase + "stops.csv");
        Graph::toDimacs(fileNameBase + "footpaths", transferGraph, transferGraph[TravelTime]);
    }

protected:
    template<typename TYPE>
    inline void writeVectorCSV(const std::string& fileName, const std::vector<TYPE>& data, const std::string& idHeader = "id") const noexcept {
        std::ofstream file(fileName);
        Assert(file);
        Assert(file.is_open());
        file << idHeader << "," << TYPE::CSV_HEADER << "\n";
        for (size_t i = 0; i < data.size(); i++) {
            file << i << "," << data[i].toCSV() << "\n";
        }
        file.close();
    }

    inline void writeTransferCSV(const std::string& fileName) const noexcept {
        std::ofstream file(fileName);
        Assert(file);
        Assert(file.is_open());
        file << "transfer_id,dep_stop,arr_stop,duration\n";
        for (const Vertex vertex : transferGraph.vertices()) {
            if (!isStop(vertex)) continue;
            for (const Edge edge : transferGraph.edgesFrom(vertex)) {
                if (!isStop(transferGraph.get(ToVertex, edge))) continue;
                file << edge << "," << vertex << "," << transferGraph.get(ToVertex, edge) << "," << transferGraph.get(TravelTime, edge) << "\n";
            }
        }
        file.close();
    }

    inline void writeExchangeFormatConnections(const std::string& fileName) const noexcept {
        std::ofstream file(fileName);
        Assert(file);
        Assert(file.is_open());
        file << "connection_id,departure_vertex,arrival_vertex,departure_time,arrival_time,trip_id";
        for (size_t i = 0; i < connections.size(); i++) {
            const Connection& connection = connections[i];
            file << "\n" << (i + 1) << "," << (connection.departureStopId + 1) << "," << (connection.arrivalStopId + 1) << "," << connection.departureTime << "," << connection.arrivalTime << "," << (connection.tripId + 1);
        }
        file.close();
    }

    inline void writeExchangeFormatTrips(const std::string& fileName) const noexcept {
        std::ofstream file(fileName);
        Assert(file);
        Assert(file.is_open());
        file << "trip_id,gtfs_trip_short_name,gtfs_route_type,gtfs_route_short_name,gtfs_route_long_name";
        for (size_t i = 0; i < tripData.size(); i++) {
            const Trip& trip = tripData[i];
            const std::vector<std::string> routeName = String::split(trip.routeName, ']');
            AssertMsg(routeName.size() == 1 || routeName.size() == 2, "Route name consists of " << routeName.size() << " parts!");
            if (routeName.size() == 1) {
                file << "\n" << (i + 1) << "," << String::replaceAll(trip.tripName, ',', "") << "," << trip.type << "," << String::replaceAll(trip.routeName, ',', "") << "," << String::replaceAll(trip.routeName, ',', "");
            } else {
                file << "\n" << (i + 1) << "," << String::replaceAll(trip.tripName, ',', "") << "," << trip.type << "," << String::replaceAll(routeName[0].substr(1), ',', "") << "," << String::replaceAll(routeName[1].substr(1), ',', "");
            }
        }
        file.close();
    }

    inline void writeExchangeFormatStops(const std::string& fileName) const noexcept {
        std::ofstream file(fileName);
        Assert(file);
        Assert(file.is_open());
        file << "vertex_id,gtfs_stop_name,minimum_change_time";
        for (size_t i = 0; i < stopData.size(); i++) {
            const Stop& stop = stopData[i];
            file << "\n" << (i + 1) << "," << String::replaceAll(stop.name, ',', "") << "," << stop.minTransferTime;
        }
        file.close();
    }

public:
    std::vector<Connection> connections;
    std::vector<Stop> stopData;
    std::vector<Trip> tripData;

    TransferGraph transferGraph;

    bool implicitDepartureBufferTimes;
    bool implicitArrivalBufferTimes;

};

const std::vector<std::string> Data::stopFileNameAliases{"stops.csv", "stop.csv"};
const std::vector<std::string> Data::connectionFileNameAliases{"connections.csv", "connection.csv", "con.csv"};
const std::vector<std::string> Data::tripFileNameAliases{"trips.csv", "trip.csv"};
const std::vector<std::string> Data::transferFileNameAliases{"transfers.csv", "transfer.csv", "footpaths.csv", "footpath.csv"};
const std::vector<std::string> Data::zoneFileNameAliases{"zones.csv", "zone.csv"};
const std::vector<std::string> Data::zoneTransferFileNameAliases{"zone_transfers.csv", "zone_transfer.csv"};
const std::vector<std::string> Data::demandFileNameAliases{"demand.csv"};

}
