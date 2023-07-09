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

#include "AGVControlApp.h"
// #include "HospitalControlApp.h"
#include "Constant.h"
#include "veins/modules/application/traci/TraCIDemo11pMessage_m.h"
#include <cstdlib>
#include <iostream>
#include <fstream>

#include "boost/lexical_cast.hpp"
using boost::lexical_cast;

using namespace veins;

Register_Class(AGVControlApp);

void AGVControlApp::initialize(int stage)
{
    TraCIDemo11p::initialize(stage);
    this->station = new Station();
    sentFirstMessage = false;
    if (stage == 0)
    {
        idDebug = getId();
        sendBeacon = new cMessage("send Beacon");
        {
            mobility = TraCIMobilityAccess().get(getParentModule());
            traciVehicle = mobility->getVehicleCommandInterface();
            originalRoute = traciVehicle->getRouteId();
        }
    }
    else if (stage == 1)
    {

        if (sendBeacon->isScheduled())
        {
            cancelEvent(sendBeacon);
        }

        scheduleAt(simTime() + 0.1, sendBeacon);
        this->travellingTime = simTime().dbl();
        curPosition = mobility->getPositionAt(simTime());

        if (Constant::activation == NULL)
        {
            Constant::activation = mobility;
        }
        this->indexInRoute = getIndexInFlow(std::to_string(myId),
                                            originalRoute);
        this->station->getStation(originalRoute);
        traciVehicle->setSpeedMode(0x1f);
        this->state = new StateOfAGV();

        timing(&wc1, &cpuT);
    }
    this->sooner = 0;
    this->later = 0;

    // Read map data
    std::ifstream jsonFile("mapData.json");
    std::string fileContent((std::istreambuf_iterator<char>(jsonFile)),
                            (std::istreambuf_iterator<char>()));

    // Parse the string into a json object
    json data = json::parse(fileContent);

    // Check if the object is an array
    if (data.is_array())
    {
        // Get the number of elements in the array
        //        std::cout << "The array has " << data.size() << " elements.\n";

        // Loop through the elements of the array
        for (size_t i = 0; i < data.size(); i++)
        {
            // Access the element by index
            json element = data.at(i);
            mapData.push_back(element);
            // Do something with the element
            //        std::cout << "Element " << i << ": " << element.dump() << "\n";
        }
    }
    else
    {
        // The object is not an array
        std::cout << "The object is not an array.\n";
    }

    // Read cyclicalData file
    ifstream file("cyclicalData.txt");
    // Check if the file is open
    if (file.is_open())
    {
        // Define a string variable to store each line
        std::string line;

        // Loop through each line of the file
        while (getline(file, line))
        {
            // Add the line to the vector
            cyclicalData.push_back(trim(line));
        }

        // Close the file
        file.close();
    }
    else
    {
        // Handle the error
        cout << "Unable to open file\n";
    }

    // Read input data
    inputData = Utility::readInputData("input.json");
    timeFrameData = Utility::readInputData("timeframe.json");
    getListVertices("i-vertex.txt");
}

void AGVControlApp::getListVertices(std::string fileName)
{
    if (vertices.size() > 0)
    {
        return;
    }
    std::ifstream file1(fileName);
    std::string line;
    while (getline(file1, line))
    {
        vertices.push_back(line);
    }
    file1.close();
}

void AGVControlApp::finish()
{
    EV << myId << endl;
    this->travellingTime = // traciVehicle->getWaitingTime();
                           //                     traciVehicle->getAccumulatedWaitingTime();
        simTime().dbl() - this->travellingTime;
    // EV<<"This AGV spends "<<this->travellingTime<<" for travelling"<<endl;
    Constant::TOTAL_TRAVELLING_TIME += this->travellingTime;
    Constant::TOTAL_WAITING_TIME += this->waitingIntervals;
    Constant::TOTAL_AGV++;
    Constant::GLOBAL_SONNER += this->sooner;
    Constant::GLOBAL_LATER += this->later;
    Constant::GLOBAL_HARMFULNESS += this->harmfulness;
    Constant::EMERGENCY_TIME += this->state->totalEmergencyTime();
    if (T != 0)
        Constant::TOTAL_APE += (APE / T);
    TraCIDemo11p::finish();
    if (Constant::activation == NULL)
    {
        EV << "Constant is helpless eventually" << endl;
    }
    timing(&wc2, &cpuT);
    wc2 -= wc1;
    EV << "\n Time: " << wc2 / 60 << "(ms)\n";
    // An example of executing other C program from Omnet++ source code
    // std::system("./a.o > outCar.txt");
    //  statistics recording goes here
}

void AGVControlApp::onBSM(DemoSafetyMessage *bsm)
{
    // for my own simulation circle
}

void AGVControlApp::onWSM(BaseFrame1609_4 *wsm)
{
    // Your application has received a data message from another car or RSU
    // code for handling the message goes here, see TraciDemo11p.cc for examples
    // }
}

void AGVControlApp::onWSA(DemoServiceAdvertisment *wsa)
{
    // Your application has received a service advertisement from another car or RSU
    // code for handling the message goes here, see TraciDemo11p.cc for examples
}

void AGVControlApp::handleSelfMsg(cMessage *msg)
{
    TraCIDemo11p::handleSelfMsg(msg);
    // this method is for self messages (mostly timers)
    // it is important to call the DemoBaseApplLayer function for BSM and WSM transmission
    {
        TraCIDemo11pMessage *carBeacon = new TraCIDemo11pMessage("test", 0);
        {
            //            std::string inputPath = "find input/ -name " + std::to_string(myId) + "_* -exec basename {} ';' -quit";
            //            std::string fileName = this->exec(inputPath.c_str());

            std::string str = traciVehicle->getLaneId();
            str.erase(str.find("_"));

            double temp = simTime().dbl();

            if (temp > 142 && temp < 144)
            {
                EV << "hello" << std::endl;
            }
            std::string roadId = traciVehicle->getRoadId();
            bool isJunction = false;
            if (roadId.find(":J") == 0)
            {
                roadId.erase(roadId.find("_"));
                isJunction = true;
            }
            else if (isSimulating)
            {
                // Check if agv have reached the new edge
                if (savedEdge.compare(str) != 0)
                {
                    savedEdge = str;
                    std::string fileName = Utility::cmdFindFile("agv_info",
                                                                std::to_string(myId));
                    if (fileName.length() == 0)
                    { // write agv's data if not already exist
                        std::ofstream(
                            std::string("agv_info/") + srcJuncSFM + "_" + destJuncSFM + "_" + str + "_" + std::to_string(myId));
                    }
                    else
                    { // update agv's data
                        std::string cmdRenameFile = "mv agv_info/" + fileName + " agv_info/" + srcJuncSFM + "_" + destJuncSFM + "_" + str + "_" + std::to_string(myId);
                        std::system(cmdRenameFile.c_str());
                    }

                    // std::cout << std::to_string(myId) << " - src: "
                    //           << srcJuncSFM << " curLane: " << savedEdge
                    //           << " dest: " << destJuncSFM << std::endl;
                    // Get all AGV information in current hallway
                    if (srcJuncSFM.compare("#") != 0 && destJuncSFM.compare("#") != 0)
                    {
                        std::vector<std::string> laneIds = split(curLaneIds,
                                                                 " ");
                        float segLength = hallwayLength / laneIds.size();

                        bool needToRunSimulation = false;
                        std::vector<json> otherAGVs = getOtherAGVInfo();
                        for (size_t i = 0; i < otherAGVs.size(); i++)
                        {
                            // if (temp > 7.5) {
                            //     std::cout << std::to_string(myId) << otherAGVs[i] << std::endl;
                            // }
                            if (otherAGVs[i]["position"] < segLength + 1)
                            {
                                needToRunSimulation = true;
                                break;
                            }
                        }
//                        std::cout << "Need to run simulation: "
//                                  << needToRunSimulation << std::endl;
                        if (needToRunSimulation)
                        {
                            timeLastSimulation = temp;
                            std::vector<json> agvSrcDestCodes;
                            for (size_t j = 0; j < laneIds.size(); j++)
                            {
                                if (laneIds[j].compare(str) == 0)
                                {
                                    float pos = j * segLength;
                                    agvSrcDestCodes.push_back({{"src", 0}, {"position", pos}, {"main", 1}});
                                    break;
                                }
                            }
                            if (otherAGVs.size() > 0)
                            {
                                agvSrcDestCodes.insert(agvSrcDestCodes.end(),
                                                       otherAGVs.begin(), otherAGVs.end());
                            }

                            // Run simulation
                            timeRequired = runSimulation(agvSrcDestCodes,
                                                         totalAgents);
                            // timeRequired = 1;
                            std::cout << "++--++ Last Required Time: "
                                      << timeRequired << " - Last simulation: "
                                      << timeLastSimulation << endl;

                            delete socialForce;
                            socialForce = 0;
                        }
                    }
                }
            }

            if (isJunction && std::find(vertices.begin(), vertices.end(), roadId) != vertices.end())
            {
                //                roadId.erase(roadId.find("_"));
                if (srcJuncSFM.compare(roadId) != 0)
                {
                    if (isSimulating && !isGettingEdge && destJuncSFM.compare(roadId) == 0)
                    {
                        if (fabs(timeLastSimulation - 0) > 0.01)
                        {
                            timeSpent = temp - timeLastSimulation;
                        }
                        else
                        {
                            timeSpent = temp - timeEnterHallway;
                        }
                        std::cout << std::to_string(myId) << " - Spent time: "
                                  << timeSpent << std::endl;
                        if (timeSpent < timeRequired)
                        {
                            pausingTime = temp;
                            waitingTime = timeRequired - timeSpent;
                            std::cout << std::to_string(myId)
                                      << " - Waiting time: " << waitingTime
                                      << std::endl;
                            velocityBeforeHalt = traciVehicle->getSpeed();
                            //                            traciVehicle->setSpeed(0);
                            isHalting = true;
                            std::cout << std::to_string(myId)
                                      << " - Stop at time: " << temp
                                      << " - Location: " << roadId << std::endl;
                        }
                        isSimulating = false;

                        // delete old data when the simulation is finished
                        std::string fileName = Utility::cmdFindFile("agv_info",
                                                                    std::to_string(myId));
                        if (fileName.length() != 0)
                        {
                            std::string cmdRemoveFile = "rm agv_info/" + fileName;
                            std::system(cmdRemoveFile.c_str());
                        }
                    }
                    for (size_t i = 0; i < cyclicalData.size(); i++)
                    {
                        if (cyclicalData[i].compare(roadId) == 0)
                        {
                            std::cout << std::to_string(myId)
                                      << " ===>Starting junction: " << roadId
                                      << std::endl;
                            srcJuncSFM = roadId;
                            timeEnterHallway = temp;
                            //                            std::cout << "Time enter: " << timeEnterHallway
                            //                                    << std::endl;
                            isGettingEdge = true;
                            isSimulating = true;
                            timeLastSimulation = 0;
                            break;
                        }
                    }
                }
            }
            else if (isGettingEdge)
            {
                json hallwayCharc = getHallwayCharc(roadId);
                isGettingEdge = false;
                std::cout << std::to_string(myId) << " - Starting edge: "
                          << roadId << std::endl;

                if (hallwayCharc != nullptr)
                {
                    // Get dest
                    destJuncSFM = hallwayCharc["dest"];
                    Utility::removeChar(destJuncSFM, '"');

                    walkwayWidth = (float)inputData["hallwayWidth"]["value"];

                    // Get hallway length
                    std::string lengthTemp = hallwayCharc["length"];
                    Utility::removeChar(lengthTemp, '"');

                    hallwayLength = stof(lengthTemp);

                    // Get number of total agents
                    int useOldNoAgents =
                        (int)inputData["useOldNoAgents"]["value"];
                    std::string fileName = Utility::cmdFindFile("agent_data",
                                                                srcJuncSFM + "_" + destJuncSFM);
                    if (useOldNoAgents == 0)
                    {
                        vector<int> minMaxAgents = Utility::getMinMaxAgents(
                            timeFrameData, 8, srcJuncSFM);
                        totalAgents = Utility::getNumTotalAgents(
                            minMaxAgents[0], minMaxAgents[1]);
                        if (fileName.length() == 0)
                        {
                            std::ofstream(
                                std::string("agent_data/") + std::to_string(totalAgents) + "_" + srcJuncSFM + "_" + destJuncSFM);
                        }
                        else
                        {
                            std::vector<std::string> noAgentsTokens = split(
                                fileName, "_");
                            if (totalAgents != stoi(noAgentsTokens[0]))
                            {
                                std::string cmdRenameFile = "mv agent_data/" + fileName + " agent_data/" + std::to_string(totalAgents) + "_" + srcJuncSFM + "_" + destJuncSFM;
                                std::system(cmdRenameFile.c_str());
                            }
                        }
                    }
                    else
                    {
                        if (fileName.length() == 0)
                        { // write number of agent data if not already exist
                            vector<int> minMaxAgents = Utility::getMinMaxAgents(
                                timeFrameData, 8, srcJuncSFM);
                            totalAgents = Utility::getNumTotalAgents(
                                minMaxAgents[0], minMaxAgents[1]);
                            std::ofstream(
                                std::string("agent_data/") + std::to_string(totalAgents) + "_" + srcJuncSFM + "_" + destJuncSFM);
                        }
                        else
                        { // use old data
                            std::vector<std::string> noAgentsTokens = split(
                                fileName, "_");
                            totalAgents = stoi(noAgentsTokens[0]);
                        }
                    }

                    // Get all laneId in current hallway
                    std::string laneIdsStr = hallwayCharc["laneIds"];
                    Utility::removeChar(laneIdsStr, '"');
                    curLaneIds = laneIdsStr;

                    // Get all AGV information in current hallway
                    std::vector<json> otherAGVs = getOtherAGVInfo();

                    std::vector<json> agvSrcDestCodes;
                    agvSrcDestCodes.push_back({{"src", 0},
                                               {"position", 0},
                                               {"main", 1}});
                    if (otherAGVs.size() > 0)
                    {
                        agvSrcDestCodes.insert(agvSrcDestCodes.end(),
                                               otherAGVs.begin(), otherAGVs.end());
                    }

                    // Run simulation
                    timeRequired = runSimulation(agvSrcDestCodes, totalAgents);
                    std::ofstream(
                                std::string("traveling_time/") + srcJuncSFM + "_" + destJuncSFM + "_" + std::to_string(timeRequired));
                    // timeRequired = 1;
                    std::cout << "Required Time: " << timeRequired << endl;

                    delete socialForce;
                    socialForce = 0;
                }
            }
            //            }

            if (isHalting)
            {
                if (velocityBeforeHalt == -1)
                {
                    velocityBeforeHalt = 4;
                }
                traciVehicle->setSpeed(0);
            }

            //            if (str.compare("-E121") == 0) {
            //                pausingTime = simTime().dbl();
            //                traciVehicle->setSpeed(0);
            //                if (fileName.length() == 0 && requested == false) {
            //                    waitingTime = DBL_MAX;
            //                    std::ofstream(std::string("input/") + std::to_string(myId)
            //                                            + "_" + str + "_" + lexical_cast<std::string>(pausingTime));
            //                    requested = true;
            //                }
            //            }

            if (str[0] != ':')
            {
                bool add = true;
                if (passedEdges.size() > 0)
                {
                    if (passedEdges[passedEdges.size() - 1].compare(str) == 0)
                    {
                        add = false;
                    }
                }
                if (add)
                {
                    passedEdges.push_back(str);
                }
            }
            if (goAround(passedEdges))
            {
                EV << "Go around" << myId << " " << originalRoute << endl;
            }
            content = "{\"laneId\" : \"" + traciVehicle->getLaneId() + "\", ";

            content = content + " \"idMess\" : \"" + std::to_string(idOfMessage) + "\", ";
            idOfMessage++;

            content = content + checkForPausing();

            double speed = traciVehicle->getSpeed();
            if (speed == 0.0)
            {
                //               double temp = simTime().dbl();
                //               if (temp > 56 ){
                //                   EV<<"Stoppppp"<<endl;
                //               }
                //               if (waitingTime == DBL_MAX) {
                //                  std::string outputPath = "find output/ -name " + lexical_cast<std::string>(16)
                //                          + "_* -exec basename {} ';' -quit";
                //                  std::string originalFileName = this->exec(outputPath.c_str());
                //                  originalFileName = originalFileName.substr(0, originalFileName.size() - 1); // remove newline character at the end of line
                //                  std::string tempFile = "";
                //                  tempFile.assign(originalFileName);
                //                  if (tempFile.length() > 0) {
                //                      tempFile.erase(std::remove(tempFile.begin(), tempFile.end(), '\n'), tempFile.cend());
                //                      tempFile.erase(0, tempFile.find("_") + std::string("_").length());
                //                      waitingTime = std::stod(tempFile);
                //                      std::string tempCmd = "rm output/" + originalFileName;
                //                      std::system(tempCmd.c_str());
                //                      deletedOldRes = true;
                //                  }
                //               }
                //               if (deletedOldRes && str.compare("-E121") != 0) {
                //                   requested = false;
                //                   deletedOldRes = false;
                //               }

                this->waitingIntervals++;
                std::string laneID = traciVehicle->getLaneId();
                this->saveBeginningOfStuck(laneID);
                if (this->stuckAtJunc[laneID] + waitingTime < simTime().dbl())
                {
                    this->runAfterStuck();
                    if (isHalting)
                    {
                        isHalting = false;
                        std::cout << std::to_string(myId) << " - Re-run at: "
                                  << temp << std::endl;
                    }
                }
                else
                {
                    if (this->checkEmergencySituation())
                    {
                        this->state->startEmergencyMode();
                        traciVehicle->setSpeedMode(0);
                        traciVehicle->setSpeed(Constant::MAX_SPEED + 2);
                        std::cout << "Emergency case" << std::endl;
                    }
                    else
                    {
                        this->state->stopEmergencyMode();
                        traciVehicle->setSpeedMode(0x1f);
                    }
                }
            }

            content = content + "\"lanePos\" : " + /*"L.P"*/ "\"" + std::to_string(traciVehicle->getLanePosition()) + "\", ";
            this->exponentialSmooth(traciVehicle->getLaneId(), simTime().dbl());
            content = content + "\"speed\" : " + "\""
                      /*"velo:"*/
                      + std::to_string(speed) + "\", ";
            content = content + "\"ratio\" : \"" + std::to_string(predictRatio) + "\", ";

            content = content + "\"now\" : \"" + std::to_string(simTime().dbl()) + "\", ";

            content = content + "\"indexInRoute\" : \"" + std::to_string(this->indexInRoute) + "\", ";

            content = content + "\"originalRouteId\" : " + "\"" + originalRoute + "\"}";
            carBeacon->setDemoData(content.c_str());
            carBeacon->setSenderAddress(myId);
            BaseFrame1609_4 *WSM = new BaseFrame1609_4();
            WSM->encapsulate(carBeacon);
            populateWSM(WSM);
            // std::cout << "==========> Ready to send ++++++++++ " << myId << " " << temp << std::endl;
            send(WSM, lowerLayerOut);

            if (expectedRoute.length() > 0 && !Constant::SHORTEST_PATH)
            {
                double t = simTime().dbl();

                std::string current = traciVehicle->getLaneId();
                int x = current.find("_");
                if (x > 0)
                    current = current.substr(0, x);
                if (expectedRoute.find(current) != std::string::npos)
                {
                    try
                    {
                        std::vector<std::string> v = split(expectedRoute, " ");

                        int i = 0;
                        bool found = false;
                        if (goAround(v))
                        {
                            EV << "Go around" << endl;
                        }
                        for (i = 0; i < v.size(); i++)
                        {
                            if (v[i].compare(current) == 0)
                            {
                                found = true;
                                break;
                            }
                            /*if(v[i][0] != '^'){

                             }*/
                            else
                            {
                                if (v[i].compare("^" + current) == 0)
                                {
                                    found = true;
                                    break;
                                }
                            }
                        }
                        if (!found)
                            i = 0;
                        // std::list<std::string> l(v.begin() + i, v.end());
                        parseLanes(i, v);
                        bool change = traciVehicle->changeVehicleRoute(l);
                        if (change)
                        {
                            expectedRoute = "";
                            v.clear();
                            // emergencyLanes.clear();
                        }
                    }
                    catch (std::exception &e)
                    {
                        // const char* x= e.what();
                        // EV<<x<<endl;
                    }
                }
            }
            return;
        }
    }
}

std::vector<json> AGVControlApp::getOtherAGVInfo()
{
    // std::cout << srcJuncSFM << " " << destJuncSFM << " " << curLaneIds << " " << hallwayLength << std::endl;
    std::vector<json> result;

    std::vector<std::string> laneIds = split(curLaneIds, " ");
    float segLength = hallwayLength / laneIds.size();
    std::string endWithMyId = "_" + std::to_string(myId);

    // find agvs moving in the same direction
    std::string cmdFindAGVsSameDirection = "find agv_info/ -name " + srcJuncSFM + "_" + destJuncSFM + "_* -exec basename {} ';'";
    std::string sameDirection = Utility::exec(cmdFindAGVsSameDirection.c_str());
    sameDirection = Utility::trim(sameDirection);
    std::vector<std::string> sameDList = split(sameDirection, "\n");
    for (size_t i = 0; i < sameDList.size(); i++)
    {
        if (sameDList[i].rfind(endWithMyId) != sameDList[i].length() - endWithMyId.length())
        {
            std::vector<std::string> tokenList = split(sameDList[i], "_");
            std::string inEdge = tokenList[2];

            for (size_t j = 0; j < laneIds.size(); j++)
            {
                if (laneIds[j].compare(inEdge) == 0)
                {
                    float pos = j * segLength;
                    result.push_back({{"src", 0}, {"position", pos}, {"main", 0}});
                    break;
                }
            }
        }
    }

    // find the agvs moving in the opposite direction
    std::string cmdFindAGVsOpDirection = "find agv_info/ -name " + destJuncSFM + "_" + srcJuncSFM + "_* -exec basename {} ';'";
    std::string opDirection = Utility::exec(cmdFindAGVsOpDirection.c_str());
    opDirection = Utility::trim(opDirection);
    std::vector<std::string> opList = split(opDirection, "\n");
    for (size_t i = 0; i < opList.size(); i++)
    {
        if (opList[i].rfind(endWithMyId) != opList[i].length() - endWithMyId.length())
        {
            std::vector<std::string> tokenList = split(opList[i], "_");
            std::string inEdge = tokenList[2];

            for (size_t j = 0; j < laneIds.size(); j++)
            {
                if (laneIds[j].compare(inEdge) == 0)
                {
                    result.push_back(
                        {{"src", 2}, {"position", (laneIds.size() - 1 - j) * segLength}, {"main", 0}});
                    break;
                }
            }
        }
    }

    return result;
}

double AGVControlApp::runSimulation(std::vector<json> agvInfo,
                                    int totalAgents)
{
    std::cout << "Start simulation" << std::endl;

    double result = -1;

    socialForce = new SocialForce;

    if (hallwayLength < 0)
    {
        return -1;
    }

    float length1Side = hallwayLength / 2;
    juncDataGraphMode = {length1Side, length1Side};

    createWalls(socialForce, juncDataGraphMode);
    createAgents(socialForce, totalAgents);
    createAGVs(agvInfo);

    startTiming = chrono::high_resolution_clock::now();
    std::vector<SFMAGV *> agvs = socialForce->getAGVs();
    std::cout << std::to_string(myId) << " - Total number of AGVs: "
              << agvs.size() << std::endl;
    string message = "AGVs are running on hallway with length " + std::to_string(juncDataGraphMode[0] * 2) + " with " + std::to_string(totalAgents) + " agents";
    cout << message << endl;

    for (SFMAGV *agv : agvs)
    {
        agv->setIsMoving(true);
        auto elapsedTime = chrono::duration_cast<chrono::milliseconds>(
            chrono::high_resolution_clock::now() - startTiming);
        int agvStartTime = static_cast<int>(elapsedTime.count());
        agv->setTravelingTime(agvStartTime);
        agv->setPrevTime(agvStartTime);
    }

    while (updateNoGraphics())
    {
    };

    for (SFMAGV *agv : agvs)
    {
        if (agv->getMainAgv() == 1)
        {
            result = agv->getTravelingTime() / 1000;
            break;
        }
    }

    return result;
}

json AGVControlApp::getHallwayCharc(std::string laneId)
{
    std::vector<std::string> result;
    json objectData = nullptr;
    for (size_t i = 0; i < mapData.size(); i++)
    {
        std::string startingEdge = to_string(mapData[i]["startingEdge"]);
        Utility::removeChar(startingEdge, '"');
        if (startingEdge.compare(laneId) == 0)
        {
            objectData = mapData[i];
            break;
        }
    }
    if (!objectData.empty())
    {
        std::string src = objectData["src"];
        Utility::removeChar(src, '"');
        std::string dest = objectData["dest"];
        Utility::removeChar(dest, '"');
        for (size_t i = 0; i < cyclicalData.size(); i++)
        {
            if (cyclicalData[i].compare(src) == 0 || cyclicalData[i].compare(dest) == 0)
            {
                // std::string length = objectData["length"];
                // Utility::removeChar(length, '"');
                // result.push_back(dest);
                // result.push_back(length);
                // return result;
                return objectData;
            }
        }
    }
    return nullptr;
}

void AGVControlApp::createWalls(SocialForce *socialForce,
                                std::vector<float> juncData)
{
    Wall *wall;

    vector<float> coors = Utility::getWallCoordinates(walkwayWidth, juncData);

    // Upper Wall
    wall = new Wall(coors[0], coors[1], coors[2], coors[3]);
    socialForce->addWall(wall);
    // Lower Wall
    wall = new Wall(coors[4], coors[5], coors[6], coors[7]);
    socialForce->addWall(wall);
}

void AGVControlApp::setAgentsFlow(SocialForce *socialForce, Agent *agent,
                                  float desiredSpeed, int caseJump)
{
    // if (socialForce->getCrowdSize() < threshold)
    // {
    //   agent->setStopAtCorridor(true);
    // }

    int codeSrc = 0;
    // int codeDes = 0;

    if (caseJump < 3)
    {
        codeSrc = 0;
    }
    else
    {
        codeSrc = 1;
    }

    vector<float> position = Utility::getAgentSrc(codeSrc,
                                                  (float)inputData["totalCrowdLength"]["value"],
                                                  (float)inputData["headCrowdLength"]["value"],
                                                  (float)inputData["crowdWidth"]["value"], walkwayWidth,
                                                  juncDataGraphMode);

    vector<float> desList = Utility::getAgentDest(codeSrc, caseJump % 3,
                                                  walkwayWidth, juncDataGraphMode, agent->getStopAtCorridor());

    agent->setPosition(position[0], position[1]);
    agent->setPath(desList[0], desList[1], desList[2]);
    agent->setDestination(desList[0], desList[1]);
    agent->setDesiredSpeed(desiredSpeed);
    std::vector<float> color = getAgentColor(agent->getDesiredSpeed());
    agent->setColor(color[0], color[1], color[2]);
    socialForce->addAgent(agent);
}

void AGVControlApp::createAgents(SocialForce *socialForce, int noAgents)
{
    Agent *agent;

    float deviationParam = randomFloat(
        1 - (float)inputData["experimentalDeviation"]["value"] / 100,
        1 + (float)inputData["experimentalDeviation"]["value"] / 100);
    // cout << "Deviation: "<< deviationParam <<" - Num agents: "<< int(int(inputData["numOfAgents"]["value"]) * deviationParam) << endl;
    numOfPeople = Utility::getNumAgentsFlow(noAgents);
    vector<double> velocityList = Utility::getAgentsSpeed(inputData, noAgents,
                                                          deviationParam);

    auto rng = std::default_random_engine{};
    std::shuffle(velocityList.begin(), velocityList.end(), rng);

    int pedesCount = 0;
    // A starting direction will have 3 arrivals on the opposite side
    for (int idx = 0; idx < 6; idx++)
    {
        for (int temp = 0; temp < numOfPeople[idx]; temp++)
        {
            agent = new Agent;
            setAgentsFlow(socialForce, agent, velocityList[pedesCount], idx);
            pedesCount = pedesCount + 1;
        }
    }
}

void AGVControlApp::createAGVs(vector<json> agvSrcDestCodes)
{
    SFMAGV *agv = NULL;
    vector<int> array;

    // std::cout << "AGV Src Dest Codes: " << agvSrcDestCodes << std::endl;
    for (int i = 0; i < agvSrcDestCodes.size(); i++)
    {
        agv = new SFMAGV();
        agv->setMainAgv(agvSrcDestCodes[i]["main"]);
        vector<Point3f> route = Utility::getRouteAGV(agvSrcDestCodes[i]["src"],
                                                     agvSrcDestCodes[i]["position"], walkwayWidth,
                                                     juncDataGraphMode);
        agv->setDirection(agvSrcDestCodes[i]["src"], 1);
        agv->setPosition(route[0].x, route[0].y);
        agv->setDestination(route[route.size() - 1].x,
                            route[route.size() - 1].y);
        agv->setDesiredSpeed((float)inputData["agvDesiredSpeed"]["value"]);
        agv->setAcceleration(inputData["acceleration"]["value"]);
        agv->setThresholdDisToPedes(
            (float)inputData["thresDistance"]["value"]);
        for (int i = 1; i < route.size(); i++)
        {
            agv->setPath(route[i].x, route[i].y, 1.0);
        }
        socialForce->addAGV(agv);
    }
}

// return true if the agv is still moving
bool AGVControlApp::updateNoGraphics()
{

    int frameTime; // Store time in milliseconds
    auto elapsedTime = chrono::duration_cast<chrono::milliseconds>(
        chrono::high_resolution_clock::now() - startTiming);
    currTime = static_cast<int>(elapsedTime.count());

    static int prevTime;

    frameTime = currTime - prevTime;
    prevTime = currTime;

    int count_agvs = 0;

    std::vector<Agent *> agents = socialForce->getCrowd();
    for (Agent *agent : agents)
    {
        Point3f src = agent->getPosition();
        Point3f des = agent->getDestination();

        if (Utility::isPositionErr(src, walkwayWidth, juncDataGraphMode.size(),
                                   socialForce->getAGVs()))
        {
            socialForce->removeAgent(agent->getId());
            continue;
        }

        if (agent->getVelocity().length() < Constant::LOWER_SPEED_LIMIT + 0.2 && agent->getMinDistanceToWalls(socialForce->getWalls(), src, agent->getRadius()) < 0.2 && (agent->interDes).size() == 0)
        {
            Point3f intermediateDes = Utility::getIntermediateDes(src,
                                                                  walkwayWidth, walkwayWidth);

            (agent->interDes).push_back(intermediateDes);
            agent->setPath(intermediateDes.x, intermediateDes.y, 1.0);
            agent->setPath(des.x, des.y, 1.0);
        }

        if ((agent->interDes).size() > 0)
        {
            float distanceToInterDes = src.distance((agent->interDes).front());
            if (distanceToInterDes <= 1)
            {
                (agent->interDes).clear();
            }
        }

        float distanceToTarget = src.distance(des);
        if (distanceToTarget <= 1 || isnan(distanceToTarget))
        {
            agent->setIsMoving(false);
            if (!agent->getStopAtCorridor())
            {
                socialForce->removeAgent(agent->getId());
            }
        }
    }

    std::vector<SFMAGV *> agvs = socialForce->getAGVs();

    for (SFMAGV *agv : agvs)
    {
        // AGV processing
        if (agv->getCollisionStartTime() == 0 && agv->getVelocity().length() < speedConsiderAsStop && agv->getIsMoving())
        {
            agv->setCollisionStartTime(currTime);
            // cout << "AGV ID: " << agv->getId() << " - Start collision: " << convertTime(agv->getCollisionStartTime()) << endl;
        }

        if (agv->getCollisionStartTime() != 0 && agv->getVelocity().length() > speedConsiderAsStop && agv->getIsMoving())
        {
            agv->setTotalStopTime(
                agv->getTotalStopTime() + currTime - agv->getCollisionStartTime());
            // cout << "AGV ID: " << agv->getId() << " - Stop collision: " << convertTime(currTime) << endl;
            // cout << "AGV ID: " << agv->getId() << "=> Total collision: " << convertTime(agv->getTotalStopTime()) << endl;
            agv->setCollisionStartTime(0);
        }

        Point3f src = agv->getPosition();
        Point3f des = agv->getDestination();

        float distance = src.distance(des);
        if (distance <= 1 || isnan(distance))
        {
            if (agv->getIsMoving())
            {
                agv->setTravelingTime(currTime - agv->getTravelingTime());
                agv->setIsMoving(false);
            }
            // count_agvs = count_agvs + 1;
            if (agv->getMainAgv() == 1)
            {
                // int totalRunningTime = currTime - startTime;
                //                Utility::writeResult("end.txt", agvs, totalAgents,
                //                        totalRunningTime);

                // std::cout << "Maximum speed: " << maxSpeed << " - Minimum speed: " << minSpeed << endl;
                //                std::cout << "AGV running time: "
                //                        << convertTime(agv->getTravelingTime()) << endl;
                // std::cout << "Finish in: " << Utility::convertTime(totalRunningTime) << endl;
                delete socialForce;
                socialForce = 0;

                return false;
            }
        }
    }

    socialForce->moveCrowd(static_cast<float>(frameTime) / 1000); // Perform calculations and move agents
    socialForce->moveAGVs(static_cast<float>(frameTime) / 1000);
    return true;
}

std::string AGVControlApp::checkForPausing()
{
    std::string content = traciVehicle->getLaneId();

    if (content.find(this->station->getName() + "_") != std::string::npos && this->station->getName().length() > 0 && // pausingTime == DBL_MAX
        pausingTime + Constant::PAUSING_TIME > simTime().dbl())
    {
        if (velocityBeforeHalt == -1 && pausingTime == DBL_MAX)
        {
            velocityBeforeHalt = traciVehicle->getSpeed();
            if (velocityBeforeHalt == 0)
                velocityBeforeHalt = 2;
            pausingTime = simTime().dbl();
            traciVehicle->setSpeed(0);
        }
        if (pausingTime != DBL_MAX)
        {
            return " \"atStation\" : \"" + std::to_string(pausingTime) + "\", ";
        }
    }
    return "";
}

void AGVControlApp::handlePositionUpdate(cObject *obj)
{
    TraCIDemo11p::handlePositionUpdate(obj);
    // the vehicle has moved. Code that reacts to new positions goes here.
    // member variables such as currentPosition and currentSpeed are updated in the parent class
}

void AGVControlApp::addExpectedTime(std::string str)
{
    std::vector<std::string> list = split(str, "_");
    if (list[1].compare("0") != 0 && list[0].length() > 0)
    {
        dict[list[0]] = std::stod(list[1]);
    }
}

void AGVControlApp::exponentialSmooth(std::string key, double realTime)
{

    key.erase(key.find("_"));
    std::map<std::string, double>::iterator it;
    it = dict.find(key);
    if (it == dict.end())
        return;
    double weight = dict[key];
    if (weight == 0)
        return;

    dict.erase(key); // no longer check the key

    double realRatio = realTime / weight;

    double error = realRatio - predictRatio;
    if (key.compare(this->station->getName()) == 0)
    {
        APE += abs(realTime - weight) / realTime;
        T++;
        this->harmfulness = this->station->getHarmfulness(realTime,
                                                          this->indexInRoute, &(this->sooner), &(this->later));
    }
    Qt = Constant::GAMMA * error - (1 - Constant::GAMMA) * Qt;
    Dt = Constant::GAMMA * abs(error) - (1 - Constant::GAMMA) * Dt;
    Dt = (Dt == 0) ? 1 : Dt;
    double lambda = abs(Qt / Dt);
    predictRatio = lambda * realRatio + (1 - lambda) * predictRatio;

    if (predictRatio <= 0)
        predictRatio = 1;
}

void AGVControlApp::handleLowerMsg(cMessage *msg)
{
    BaseFrame1609_4 *WSM = check_and_cast<BaseFrame1609_4 *>(msg);
    cPacket *enc = WSM->getEncapsulatedPacket();
    if (TraCIDemo11pMessage *bc = dynamic_cast<TraCIDemo11pMessage *>(enc))
    {
        std::stringstream streamData(bc->getDemoData());
        std::string tmp;
        std::string str = "";
        while (getline(streamData, tmp))
            str += tmp;
        jute::jValue v = jute::parser::parse(str);
        std::string id = v["id"].as_string();
        if (std::to_string(myId).compare(id) == 0)
        {
            double t = simTime().dbl();
            this->readSingleMessage(v);
        }
        else
        {
            // std::string temp = v["agvInfo"].as_string();
            // if (temp.length() > 0) {
            //     string agvInfoString = v["agvInfo"].as_string();
            //     std::replace(agvInfoString.begin(), agvInfoString.end(), '@', '"');
            //     agvInfo = json::parse(agvInfoString);
            //     std::cout << "AGV Info: " << agvInfo << std::endl;
            // }
        }
    }
    else
    {
    }
}

void AGVControlApp::readSingleMessage(jute::jValue v)
{
    std::string newRoute = v["newRoute"].as_string();
    // std::string temp = v["agvInfo"].as_string();
    // if (temp.length() > 0) {
    //     string agvInfoString = v["agvInfo"].as_string();
    //     std::replace(agvInfoString.begin(), agvInfoString.end(), '@', '"');
    //     agvInfo = json::parse(agvInfoString);
    //     std::cout << "AGV Info: " << agvInfo << std::endl;
    // }

    int size = v["weights"].size();

    for (int i = 0; i < size; i++)
    {
        addExpectedTime(v["weights"][i].as_string());
    }
    if (prevRoute.compare(newRoute) != 0)
    {
        if (newRoute.compare("0") == 0)
        {
            // force AGV to stop
            if (velocityBeforeHalt == -1)
            {
                velocityBeforeHalt = traciVehicle->getSpeed();
                if (velocityBeforeHalt == 0)
                    velocityBeforeHalt = 2;
            }
            pausingTime = simTime().dbl();
            traciVehicle->setSpeed(0);
        }
        else
        {

            if (newRoute.compare(Constant::CARRY_ON) == 0)
            {
                if (prevRoute.length() == 0)
                {
                    // This case happends as: (1) AGV follows the original Dijsktra
                    //(2) messages from RSU were lost
                    double sp = traciVehicle->getSpeed();
                    if (sp == 0 && pausingTime + Constant::PAUSING_TIME < simTime().dbl())
                    {
                        // if waiting to long
                        this->runAfterStuck();
                        return;
                    }
                    else
                    {
                        return;
                    }
                }
                newRoute = prevRoute;
            }

            prevRoute = newRoute;

            this->runAfterStuck();

            if (!Constant::SHORTEST_PATH)
            {
                std::vector<std::string> v = split(newRoute, " ");

                parseLanes(0, v);
                if (l.size() == 0 || goAround(v))
                {
                    EV_TRACE << "ERRR";
                }
                bool change = traciVehicle->changeVehicleRoute(l);
                if (!change)
                {
                    expectedRoute = newRoute;
                    emergencyLanes.clear();
                }
                else
                {
                    expectedRoute = "";
                    v.clear();
                    v.shrink_to_fit();
                    l.clear();
                }
            }
        }
    }
    else
    {
        double sp = traciVehicle->getSpeed();
        std::string laneID = traciVehicle->getLaneId();
        if (sp == 0 && laneID[0] == ':')
        {
            saveBeginningOfStuck(laneID);
            if (this->stuckAtJunc[laneID] + Constant::PAUSING_TIME < simTime().dbl())
            {
                this->runAfterStuck();
            }
        }
    }
}

void AGVControlApp::parseLanes(int start, std::vector<std::string> v)
{
    l.clear();
    emergencyLanes.clear();
    for (int i = start; i < v.size(); i++)
    {
        if (v[i][0] == '^')
        {
            l.push_back(v[i].substr(1));
            emergencyLanes.push_back(v[i].substr(1));
        }
        else
        {
            l.push_back(v[i]);
        }
    }
}

void AGVControlApp::saveBeginningOfStuck(std::string junc)
{
    std::map<std::string, double>::iterator it;
    it = this->stuckAtJunc.find(junc);
    if (it == this->stuckAtJunc.end())
    {
        this->stuckAtJunc[junc] = simTime().dbl();
    }
}

void AGVControlApp::runAfterStuck()
{
    if (velocityBeforeHalt != -1)
    {
        traciVehicle->setSpeed(velocityBeforeHalt);
        velocityBeforeHalt = -1;
    }
    else
    {
        if (pausingTime + Constant::PAUSING_TIME < simTime().dbl())
        {
            return;
        }
        if (traciVehicle->getSpeed() == 0)
        {
            if (this->checkEmergencySituation())
            {
                this->state->startEmergencyMode();
                traciVehicle->setSpeedMode(0);
                traciVehicle->setSpeed(Constant::MAX_SPEED + 2);
            }
            else
            {
                this->state->stopEmergencyMode();
                traciVehicle->setSpeedMode(0x1f);
                traciVehicle->setSpeed(Constant::MAX_SPEED - 0.1);
            }
        }
    }
}

bool AGVControlApp::checkEmergencySituation()
{
    std::string laneID = traciVehicle->getLaneId();
    for (int i = 0; i < emergencyLanes.size(); i++)
    {
        if (laneID.find(emergencyLanes[i] + "_") != std::string::npos)
        {
            return true;
        }
    }
    return false;
}
