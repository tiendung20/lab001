//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//

#ifndef VEINS_INET_NISHIDADSTARLITE_H_
#define VEINS_INET_NISHIDADSTARLITE_H_

#include "ArrivalDijkstra.h"

//typedef std::pair<int, int> Node;
typedef std::pair<double, double> key;

typedef std::tuple<std::string, std::string, std::string> Itinerary;
typedef std::map<std::string, Itinerary*> Itineraries;

typedef std::map<std::string, std::vector<std::string>> Paths;

class DStarLite {

public:
    DStarLite(std::vector<std::vector<TimeSpace>> *graph, double start,
            Objective *target, double dt, Node s_start, Node s_goal);
    std::vector<std::vector<TimeSpace>> *graph;
    double start;
    Objective *target;
    double dt;
    Node s_start, s_goal;
    void main();
    void reMain();
    void update(Node u);
    std::vector<std::string> convert();
    std::vector<std::pair<int, int>> path;
    std::map<std::string, double> edgeStartTime;
    void computeShortestPath();

private:
    std::map<Node, key> U;
    double k_m;
    Node s_last;
    std::map<Node, double> g, rhs;
    double getG(Node u);
    double getRHS(Node u);
    double weightEdge(Node a, Node b);
    double getH(Node a, Node b);
    void setG(Node u, double v);
    void setRHS(Node u, double v);

    key calculateKey(Node u);
    void initialize();
    std::vector<Node> getSucc(Node u);
    std::vector<Node> getPred(Node u);
    Node top();
    void updateVertex(Node u);
    Node searchGoalNode();
};

typedef std::map<std::string, DStarLite*> DStarLiteMap;
typedef std::pair<std::string, DStarLite*> DStarLitePair;

class NishidaDStarLite: public ArrivalDijkstra {
public:
    NishidaDStarLite();
    void getItineraries(std::string itineraryFile) override;
    std::vector<std::string> findPath(TimeSpaceGraph *graph, std::string name,
            double start, Objective *target, double dt, AGV *cur,
            std::string identifier, std::string new_identifier);
    std::vector<std::string> reFindPath(TimeSpaceGraph *graph, std::string name,
            double start, Objective *target, double dt, AGV *cur,
            std::string identifier, std::string new_identifier);
    std::string getRoute(std::string trace, std::string currentLane,
            int currentVertex, int nextVertex, int exitVertex) override;
    void planOut(int source, int target, std::string currLane, AGV *cur)
            override;
    Itineraries allItinerary;
    Paths paths;
    DStarLiteMap allDStarLite;
    std::string updateEdge = "";
    std::map<std::string, std::string> listUpdateAGV;

private:
    TimeSpaceGraph *graph;
    std::map<std::string, std::map<std::string, double>> allEdgeStartTime;
};

#endif /* VEINS_INET_NISHIDADSTARLITE_H_ */
