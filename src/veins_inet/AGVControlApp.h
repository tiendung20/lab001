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

#ifndef VEINS_INET_AGVCONTROLAPP_H_
#define VEINS_INET_AGVCONTROLAPP_H_

#include "VehicleControlApp.h"
#pragma once

#include "veins/veins.h"

#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"
#include "veins/modules/application/traci/TraCIDemo11p.h"
#include <float.h>
#include <map>
#include <string>
#include "Graph.h"
#include "jute.h"
#include "StateOfAGV.h"
#include <lib/nlohmann/json.hpp>
#include <regex>
#include <random>
#include <chrono>
#include "SocialForce.h"
#include "Constant.h"
#include "Utility.h"

using namespace omnetpp;
using namespace Utility;
using json = nlohmann::json;

using namespace std;

namespace veins {

typedef std::map<std::string, double> Dictionary;
//typedef std::map<std::string, std::string> RouteDictionary;

//extern std::map<std::string, std::string> routeDict;

class AGVControlApp: public veins::TraCIDemo11p {
public:
    void initialize(int stage) override;
    void finish() override;
    cMessage* sendBeacon;
    //static std::map<std::string, std::string> routeDict;
protected:
    void onBSM(DemoSafetyMessage* bsm) override;
    void onWSM(BaseFrame1609_4* wsm) override;
    void onWSA(DemoServiceAdvertisment* wsa) override;

    void handleSelfMsg(cMessage* msg) override;
    void handlePositionUpdate(cObject* obj) override;
    void handleLowerMsg(cMessage* msg) override;
private:
    std::string checkForPausing();
    void addExpectedTime(std::string str);
    void exponentialSmooth(std::string key, double realTime);
    bool sentFirstMessage = false;
    bool receivedReRouting = false;
    double travellingTime = 0;
    long waitingIntervals = 0;
    //attributes for adaptive responsive rate
    double lambda = 0;
    int k = 0;
    double Qt = 0;
    double Dt = 0;
    double predictRatio = 1.1;

    std::string originalRoute = "";
    std::string prevRoute = "";
    std::string expectedRoute = "";
    std::string content = "";
    double velocityBeforeHalt = -1;
    double pausingTime = DBL_MAX;

    double waitingTime = DBL_MAX;
    bool requested = false;
    bool deletedOldRes = false;

    Dictionary dict;
    double APE = 0;
    int T = 0;
    double harmfulness = 0;
    Station* station;
    int indexInRoute = -1;
    int idOfMessage = 0;
    int idDebug;
    void runAfterStuck();
    void saveBeginningOfStuck(std::string junc);
    Dictionary stuckAtJunc;
    //static RouteDictionary routeDict;
    //void getIndexInFlow(std::string idOfAGV, std::string routeId);
    std::vector<std::string> passedEdges;
    std::vector<std::string> emergencyLanes;
    void parseLanes(int start, std::vector<std::string> v);
    void readSingleMessage(jute::jValue v);
    std::list<std::string> l;
    bool checkEmergencySituation();
    double sooner;
    double later;
    StateOfAGV* state;
    double wc1 = 0, wc2 = 0, cpuT = 0;

    std::vector<json> mapData;
    std::vector<std::string> cyclicalData;
    std::vector<std::string> vertices;
    void getListVertices(std::string fileName);
    std::string srcJuncSFM = "#";
    std::string destJuncSFM = "#";
    float hallwayLength = -1;
    std::string curLaneIds = "#";
    std::string prevStartingEdge = "#";
    std::string savedEdge = "#";
    bool isGettingEdge = false;
    bool isSimulating = false;
    bool isHalting = false;
    double timeEnterHallway = 0;
    double timeLastSimulation = 0;
    double timeSpent = 0;

    json getHallwayCharc(std::string lanedId);
    std::vector<json> getOtherAGVInfo();
    double runSimulation(std::vector<json> agvInfo, int totalAgents);

    // For SFM
    SocialForce *socialForce;
    int currTime = 0;
    int startTime = 0;
    float speedConsiderAsStop = 0.2;

    json agvInfo;

    json inputData;
    json timeFrameData;
    std::vector<float> juncDataGraphMode;
    float walkwayWidth;

    std::chrono::time_point<std::chrono::high_resolution_clock> startTiming;

    std::vector<int> numOfPeople;
    int totalAgents = 0;
    double timeRequired = 0;

    void createWalls(SocialForce *socialForce, std::vector<float> juncData);

    void setAgentsFlow(SocialForce *socialForce, Agent *agent, float desiredSpeed, int caseJump);
    void createAgents(SocialForce *socialForce, int noAgents);

    void createAGVs(std::vector<json> agvSrcDestCodes);

    bool updateNoGraphics();
};
}
#endif /* VEINS_INET_AGVCONTROLAPP_H_ */
