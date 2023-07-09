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

#ifndef VEINS_INET_HOSPITALCONTROLAPP_H_
#define VEINS_INET_HOSPITALCONTROLAPP_H_

#include "RSUControlApp.h"
#pragma once

#include "veins/veins.h"
#include "Zone.h"
//#include "Parser.cc"
#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"
#include "veins/modules/application/traci/TraCIDemoRSU11p.h"
#include "veins/modules/application/traci/TraCIDemo11pMessage_m.h"
//#include "Graph.h"
#include <vector>
//#include "Djisktra.h"
//#include "ArrivalDijkstra.h"
//#include "HarmfulnessDijkstra.h"
#include "DecisionDijkstra.h"
//#include "LatencyEmergencyTime.h"
// #include "AntShortestPathSystem.h"
//#include "NishidaDStarLite.h"
#include "Utility.h"
#include <string>

#include <lib/nlohmann/json.hpp>
using json = nlohmann::json;

using namespace omnetpp;
using namespace Utility;

namespace veins {

class HospitalControlApp: public TraCIDemoRSU11p {
public:
    void initialize(int stage) override;
    void finish() override;
    cMessage *sendBeacon;
    Parser *graphGenerator;

protected:
    void onBSM(DemoSafetyMessage *bsm) override;
    void onWSM(BaseFrame1609_4 *wsm) override;
    void onWSA(DemoServiceAdvertisment *wsa) override;

    void handleSelfMsg(cMessage *msg) override;
    void handlePositionUpdate(cObject *obj) override;

private:
    std::vector<Crossing> crossings;
    void readCrossing();
    //virtual std::string getIdOfGLane(std::string str);
    //virtual int getX2(int x);
    std::vector<AGV*> vhs;
    Graph *graph;
    bool hasStopped = false;
    int subscribedServiceId = 0;
    std::vector<std::string> message;
    std::string readMessage(TraCIDemo11pMessage *bc);
    void exponentialSmoothing(NodeVertex *nv, double stopTime);
    double getAverageVelocityByDensity(double density);
    void readLane(AGV *cur, std::string str);
    double lastUpdate = -2.0;
    long count = 0;
    //double getAvailablePerdestrian(std::string crossId, double _time);
    //double getVeloOfPerdestrian(std::string crossId, double _time);
    void predictDispearTime();
    Djisktra *djisktra;
    //HarmfulnessDijkstra* djisktra;
    std::string reRoute(AGV *cur, std::string routeId/*, double t*/);
    void sendToAGV(std::string content);
    std::string removeAntidromic(std::string input);
    std::vector<std::vector<int>> aroundIntersections;
    bool checkCycle(std::string route);
    double *areas;
    std::string excludeDuplication(std::string futureLane,
            std::string lastRoute);

    // For SFM
    void updateAGVSFMInfo(string agvId, string laneId);
    json agvInfo;

    std::vector<std::vector<std::string>> divideByAnyPrefix (const std::vector<std::string>& v);

};
}
#endif /* VEINS_INET_HOSPITALCONTROLAPP_H_ */
