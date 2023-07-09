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
// along with this program. If not, see http://www.gnu.org/licenses/.
//

#include "HospitalControlApp.h"
#include <stdio.h>
#include <iostream>
#include "Zone.h"
#include "Graph.h"
#include "jute.h"
#include <boost/algorithm/string.hpp>
#include <algorithm>
#include <omp.h>
#include <string>
#include <regex>

using namespace boost::algorithm;

using namespace veins;

// #include <sys/stat.h>
// #include <fstream>
// #include "boost/lexical_cast.hpp"
// using boost::lexical_cast;

Register_Class(HospitalControlApp);

void HospitalControlApp::initialize(int stage)
{
    TraCIDemoRSU11p::initialize(stage);

    // test

    // std::string cmdFindTravelingInfo =
    //     "find traveling_time/ -exec basename {} ';'";
    // std::string travelingInfo = Utility::exec(cmdFindTravelingInfo.c_str());
    // travelingInfo = Utility::trim(travelingInfo);
    // std::vector<std::string> timeList = split(travelingInfo, "\n");
    // timeList.erase(
    //     std::remove(timeList.begin(), timeList.end(), "traveling_time"),
    //     timeList.end());

    // std::sort(timeList.begin(), timeList.end()); // sort the vector
    // timeList.erase(std::unique(timeList.begin(), timeList.end()), timeList.end());

    // std::vector<std::vector<std::string>> timeListAfterProcessed = divideByAnyPrefix(timeList);
    // for (const auto& sub_v: timeListAfterProcessed) {
    //     float max = 0;
    //     float min = 100000;
    //     std::string srcJunc = split(sub_v[0], "_")[0];
    //     std::string dstJunc = split(sub_v[0], "_")[1];
    //     for (size_t i = 0; i < sub_v.size(); i++) {
    //         std::string item = split(sub_v[i], "_")[2];
    //         float requiredTime = std::stof(item);
    //         if (requiredTime > max) {
    //             max = requiredTime;
    //         }
    //         if (requiredTime < min) {
    //             min = requiredTime;
    //         }
    //     }
    //     std::cout << srcJunc << " - " << dstJunc << " - Longest time:  " << max << " - Shortest time: " << min << std::endl;
    // }

    //    EV<<"1234"<<endl;
    //    std::ofstream("input/test");
    //    string path = std::string("input/") + "16" + "_" + "-E121" + "_" + lexical_cast<std::string>(30.25);
    //    int tt = std::system("find output/ -name 16_* -exec basename {} ';'");
    //    string path = "find output/ -name " + lexical_cast<std::string>(16) + "_* -exec basename {} ';' -quit";
    //    string tt = this->exec(path.c_str());
    //    tt.erase(std::remove(tt.begin(), tt.end(), '\n'), tt.cend());
    //    tt.erase(0, tt.find("_") + std::string("_").length());

    // std::string laneIdsStr = "E1 E2 E3 E4 E5";
    // float length = 15;
    // std::vector<std::string> laneIds = split(laneIdsStr, " ");
    // float segLength = length / laneIds.size();

    // std::vector<json> rs;
    // std::string cmdFindAGVsSameDirection = "find input/ -name ab_cd_* -exec basename {} ';'";
    // std::string sameDirection = Utility::exec(cmdFindAGVsSameDirection.c_str());
    // sameDirection = Utility::trim(sameDirection);
    // std::vector<std::string> sameDList = split(sameDirection, "\n");
    // for(size_t i = 0; i < sameDList.size(); i++) {
    //     std::string endWithMyId = "_12";
    //     if (sameDList[i].rfind (endWithMyId) != sameDList[i].length () - endWithMyId.length ())
    //     {
    //         std::vector<std::string> tokenList = split(sameDList[i], "_");
    //         std::string inEdge = tokenList[2];
    //         std::cout << inEdge << std::endl;

    //         for (size_t j = 0; j < laneIds.size(); j++) {
    //             if(laneIds[j].compare(inEdge) == 0) {
    //                 float pos = j * segLength;
    //                 rs.push_back({ { "src", 0 }, { "position", pos }, { "main", 0 } });
    //                 break;
    //             }
    //         }
    //     }
    // }

    // std::string cmdFindAGVsOpDirection = "find input/ -name cd_ab_* -exec basename {} ';'";
    // std:: string opDirection = Utility::exec(cmdFindAGVsOpDirection.c_str());
    // opDirection = Utility::trim(opDirection);
    // std::vector<std::string> opList = split(opDirection, "\n");
    // for(size_t i = 0; i < opList.size(); i++) {
    //     std::string endWithMyId = "_12";
    //     if (opList[i].rfind (endWithMyId) != opList[i].length () - endWithMyId.length ())
    //     {
    //         std::vector<std::string> tokenList = split(opList[i], "_");
    //         std::string inEdge = tokenList[2];

    //         for (size_t j = 0; j < laneIds.size(); j++) {
    //             if(laneIds[j].compare(inEdge) == 0) {
    //                 float pos = j * segLength;
    //                 rs.push_back({ { "src", 2 }, { "position", (laneIds.size() - 1 - j) * segLength }, { "main", 0 } });
    //                 break;
    //             }
    //         }
    //     }
    // }

    // std::cout << rs[0] << std::endl;
    // std::cout << rs[1] << std::endl;
    // std::cout << "Hello" << std::endl;

    //    std::vector<json> mapData;
    //        std::vector<std::string> cyclicalData;
    //    // Read map data
    //        std::ifstream jsonFile("mapData.json");
    //        std::string fileContent((std::istreambuf_iterator<char>(jsonFile)),
    //                (std::istreambuf_iterator<char>()));
    //
    //        // Parse the string into a json object
    //        json data = json::parse(fileContent);
    //
    //        // Check if the object is an array
    //        if (data.is_array()) {
    //            // Loop through the elements of the array
    //            for (size_t i = 0; i < data.size(); i++) {
    //                // Access the element by index
    //                json element = data.at(i);
    //                mapData.push_back(element);
    //                // Do something with the element
    //    //        std::cout << "Element " << i << ": " << element.dump() << "\n";
    //            }
    //        } else {
    //            // The object is not an array
    //            std::cout << "The object is not an array.\n";
    //        }
    //
    //        // Read cyclicalData file
    //        ifstream file("cyclicalData.txt");
    //        // Check if the file is open
    //        if (file.is_open()) {
    //            // Define a string variable to store each line
    //            std::string line;
    //
    //            // Loop through each line of the file
    //            while (getline(file, line)) {
    //                // Add the line to the vector
    //                cyclicalData.push_back(trim(line));
    //            }
    //
    //            // Close the file
    //            file.close();
    //        } else {
    //            // Handle the error
    //            cout << "Unable to open file\n";
    //        }
    //
    //        std::string laneId = "E129";
    //
    //        json objectData;
    //            for (size_t i = 0; i < mapData.size(); i++) {
    //                std::string startingEdge = to_string(mapData[i]["startingEdge"]);
    //                removeChar(startingEdge, '"');
    //                if (startingEdge.compare(laneId) == 0) {
    //                    objectData = mapData[i];
    //                    break;
    //                }
    //            }
    //            std::cout << objectData << std::endl;
    //            std::string src = objectData["src"];
    //            removeChar(src, '"');
    //            std::string dest = objectData["dest"];
    //            removeChar(dest, '"');
    //            for (size_t i = 0; i < cyclicalData.size(); i++) {
    //                if (cyclicalData[i].compare(src) == 0
    //                        || cyclicalData[i].compare(dest) == 0) {
    //                    std::string length = objectData["length"];
    //                    removeChar(length, '"');
    //                    std::cout << "Length: " << stod(length) << std::endl;
    //                }
    //            }
    //            std::cout << "-2" << std::endl;

    // test

    if (graphGenerator == NULL)
    {
        graphGenerator = new Parser();
    }
    if (stage == 0)
    {
        sendBeacon = new cMessage("send Beacon");
        graph = new Graph();
        djisktra = // new LatencyEmergencyTime();
            //                new AntShortestPathSystem();
            // new DecisionDijkstra();
            // new HarmfulnessDijkstra();
            // new ArrivalDijkstra();
            new Djisktra();
        this->readCrossing();
    }
    else if (stage == 1)
    {
        // Initializing members that require initialized other modules goes here
    }
}

void HospitalControlApp::readCrossing()
{
    if (Constant::SHORTEST_PATH)
        return;
    // Always call after this->djisktra != NULL
    std::string line;
    std::ifstream MyReadFile("crossing.txt");
    getline(MyReadFile, line);
    // int numberOfCrossing =std::stoi(line);

    int k = 0;

    while (getline(MyReadFile, line))
    {
        size_t pos;
        std::string token;
        Crossing tmp;

        for (int i = 0; i < 2; i++)
        {
            pos = line.find(" ");
            token = line.substr(0, pos);

            if (i == 0)
                tmp.id = token;
            if (i == 1)
                tmp.name = token; // std::atof(token.c_str());
            line.erase(0, pos + 1);
        }
        tmp.rec = new CustomRectangle(line);
        crossings.push_back(tmp);
        k++;
    }

    MyReadFile.close();
    areas = (double *)malloc(this->djisktra->numIVertices * sizeof(double));
    for (int i = 0; i < this->djisktra->numIVertices; i++)
    {
        areas[i] = 0;
    }
    this->aroundIntersections.resize(this->djisktra->numIVertices);
    for (int i = 0; i < crossings.size(); i++)
    {
        for (int j = 0; j < this->djisktra->numIVertices; j++)
        {
            if (crossings[i].id.compare(this->djisktra->vertices[j]) == 0)
            {
                this->aroundIntersections[j].push_back(i);
                areas[j] += crossings[i].rec->getArea();
                break;
            }
        }
    }
}

void HospitalControlApp::finish()
{
    // Duoc goi khi RSU ket thuc cong viec
    TraCIDemoRSU11p::finish();
    if (traci == NULL)
    {
        EV << "NULL eventually" << endl;
    }
    if (Constant::activation == NULL)
    {
        EV << "Constant is helpless eventually" << endl;
    }

    EV << "#calling traci->getPersonIds(): " << count << endl;
    EV << "1) As 10 AGVs => T: 3022(3796), W: 241(1811), %: 8(48)%" << endl;
    EV
        << "2) As 10 + 1(flow 11) AGVs => T: 3177(4521), W: 463(2334), %: 15(52)%"
        << endl;
    EV
        << "3) As 10 + 1(flow 10) + 1(flow11) AGVs => T: 2982(4793), W: 76(2416), %: 3(50)%"
        << endl;
    EV
        << "4) As 10 + 1(flow 11) + 1(flow 2) AGVs => T: 3020(5175), W: 119(2782), %: 4(54)%"
        << endl;
    EV
        << "5) As 10 + 1(flow10) + 1(flow 11) + 1(flow 2) AGVs => T: 3220(5455), W: 76(2871), %: 2.3(53)%"
        << endl;
    EV
        << "6) As 10 + 2(flow10) + 1(flow 11) + 1(flow 2) AGVs => T: 3409(6846), W: 81(3616), %: 2.4(53)%"
        << endl;
    EV
        << "7) As 10 + 2(flow10) + 2(flow 11) + 1(flow 2) AGVs => T: 3988(5863), W: 133(2850), %: 3.3(49)%"
        << endl;
    EV
        << "8) As 10 + 2(flow10) + 2(flow 11) + 1(flow 2) + 1(flow0) AGVs => T: 4096(6734), W: 83.1(3517), %: 2(52)%"
        << endl;
    EV
        << "9) As 10 + 2(flow10) + 2(flow 11) + 3(flow0-2) AGVs => T: 4271(7649), W: 87(4202), %: 2(55)%"
        << endl;
    EV
        << "10) As 10 + 2(flow10) + 2(flow 11) + 4(flow0-3) AGVs => T: 5214(7261), W: 492(3621), %: 9.5(50)%"
        << endl;
    EV
        << "11) As 10 + 2(flow10) + 2(flow 11) + 5(flow0-4) AGVs => T: 4900(6488), W: 203(2644), %: 4(41)%"
        << endl;
    EV
        << "12) As 10 + 2(flow10) + 2(flow 11) + 6(flow0-5) AGVs => T: 5255(8027), W: 270(3973), %: 5(49.5)%"
        << endl;
    EV
        << "13) As 10 + 2(flow10) + 2(flow 11) + 7(flow0-6) AGVs => T: 5808(9578), W: 583(5289), %: 10(55)%"
        << endl;
    EV
        << "14) As 10 + 2(flow10) + 2(flow 11) + 8(flow0-7) AGVs => T: 6569(10471), W: 1049(5970), %: 16(57)%"
        << endl;
    EV
        << "15) As 10 + 2(flow10) + 2(flow 11) + 9(flow0-8) AGVs => T: 6439(9538), W: 694(4830), %: 11(51)%"
        << endl;
    EV
        << "16) As 10 + 2(flow10) + 2(flow 11) + 10(flow0-9) AGVs => T: 11155(12158), W: 3914(5910), %: 35(49)%"
        << endl;
    EV
        << "17) As case 16 + 24(flow 12) AGVs => T: 7168(10482), W: 622(5369), %: 9(51)%"
        << endl;
    EV << "As 10 + 1(flow 8) AGVs => T: 2709, W: 547, %: 20%" << endl;
    EV << "As 10 + 1(flow 2) AGVs => T: 3045.7, W: 875, %: 29%" << endl;
    // EV<<"As 11 AGVs => T: 2720.8, W: 547, %: 20%"<<endl;
    EV << "As 11 AGVs + 1(10) => T: 3239.9, W: 887.7, %: 27.4%" << endl;

    EV << "Reproduce case " << 17 << ") Total waiting time: "
       << Constant::TOTAL_WAITING_TIME * 0.1 << "(s)" << endl;
    EV << "Total travelling time: " << Constant::TOTAL_TRAVELLING_TIME << "(s)"
       << endl;
    double percentage = Constant::TOTAL_WAITING_TIME * 10 / Constant::TOTAL_TRAVELLING_TIME;
    EV << "% of waiting time: " << percentage << endl;
    EV << "Average Mean Absolute Percentage Error: "
       << 100 * Constant::TOTAL_APE / Constant::TOTAL_AGV << endl;
    if (Constant::SHORTEST_PATH)
    {
        EV << "Shortest Path: " << endl;
    }
    else
    {
        EV << "Our proposal - " << this->djisktra->getName() << ":" << endl;
    }
    EV << "\t Global harmfulness: " << Constant::GLOBAL_HARMFULNESS << ". ";
    EV << "Sooner: " << Constant::GLOBAL_SONNER << ". Later "
       << Constant::GLOBAL_LATER << endl;
    // statistics recording goes here

    std::string cmdFindTravelingInfo =
        "find traveling_time/ -exec basename {} ';'";
    std::string travelingInfo = Utility::exec(cmdFindTravelingInfo.c_str());
    travelingInfo = Utility::trim(travelingInfo);
    std::vector<std::string> timeList = split(travelingInfo, "\n");
    timeList.erase(
        std::remove(timeList.begin(), timeList.end(), "traveling_time"),
        timeList.end());

    std::sort(timeList.begin(), timeList.end()); // sort the vector
    timeList.erase(std::unique(timeList.begin(), timeList.end()), timeList.end());

    std::vector<std::vector<std::string>> timeListAfterProcessed = divideByAnyPrefix(timeList);
    for (const auto& sub_v: timeListAfterProcessed) {
        float max = 0;
        float min = 100000;
        std::string srcJunc = split(sub_v[0], "_")[0];
        std::string dstJunc = split(sub_v[0], "_")[1];
        for (size_t i = 0; i < sub_v.size(); i++) {
            std::string item = split(sub_v[i], "_")[2];
            float requiredTime = std::stof(item);
            if (requiredTime > max) {
                max = requiredTime;
            }
            if (requiredTime < min) {
                min = requiredTime;
            }
        }
        std::cout << srcJunc << " - " << dstJunc << " - Longest time:  " << max << " - Shortest time: " << min << std::endl;
    }

    std::string cmdRemoveFile = "rm traveling_time/*";
    std::system(cmdRemoveFile.c_str());

    cmdRemoveFile = "rm agv_info/*";
    std::system(cmdRemoveFile.c_str());
}

void HospitalControlApp::onBSM(DemoSafetyMessage *bsm)
{
    // for my own simulation circle
}

void HospitalControlApp::onWSM(BaseFrame1609_4 *wsm)
{
    // Your application has received a data message from another car or RSU
    // code for handling the message goes here, see TraciDemo11p.cc for examples
    cPacket *enc = wsm->getEncapsulatedPacket();
    if (TraCIDemo11pMessage *bc = dynamic_cast<TraCIDemo11pMessage *>(enc))
    {
        // test
        //  std::stringstream streamData(bc->getDemoData());
        //  std::string tmp;
        //  std::string str = "";
        //  while (getline(streamData, tmp))
        //      str += tmp;

        // std::cout << "==========> Ready to receive ++++++++++ " << str << std::endl;
        // test

        if (sendBeacon != NULL)
        {
            if (sendBeacon->isScheduled())
            {
                cancelEvent(sendBeacon);
            }

            if (traci == NULL)
            {
                if (Constant::activation != NULL)
                    traci = Constant::activation->getCommandInterface();
            }

            if (simTime().dbl() - lastUpdate >= 1 && !Constant::SHORTEST_PATH)
            {
                count++;
                try
                {
                    std::list<std::string> allPeople = traci->getPersonIds();
                    for (int i = 0; i < crossings.size(); i++)
                    {
                        crossings[i].count = 0;
                    }

                    double x, y;
                    for (auto elem : allPeople)
                    {
                        std::string personId = elem;
                        Coord peoplePosition = traci->getPersonPosition(
                            personId);
                        std::pair<double, double> coordTraCI =
                            traci->getTraCIXY(peoplePosition);
                        x = coordTraCI.first;
                        y = coordTraCI.second;
                        // newCoord.z = 0;
                        for (int i = 0; i < crossings.size(); i++)
                        {
                            if (crossings[i].rec->checkInside(x, y))
                            {
                                crossings[i].count++;
                                break;
                            }
                            else if (crossings[i].rec->checkAround(x, y))
                            {
                                break;
                            }
                        }
                    }
                }
                catch (std::exception &e)
                {
                }

                predictDispearTime();
                lastUpdate = simTime().dbl();
            }

            TraCIDemo11pMessage *rsuBeacon = new TraCIDemo11pMessage();

            char *ret = mergeContent(bc->getSenderAddress());

            // std::cout << "Content: " << ret << std::endl;
            rsuBeacon->setDemoData(ret);
            rsuBeacon->setSenderAddress(myId);
            try
            {
                BaseFrame1609_4 *WSM = new BaseFrame1609_4();
                WSM->encapsulate(rsuBeacon);
                populateWSM(WSM);
                send(WSM, lowerLayerOut);
            }
            catch (std::exception &e)
            {
                EV << e.what() << endl;
            }
        }

        // std::cout << "HERE..." << std::endl;

        double t = simTime().dbl();
        if (t > 224.5)
        {
            EV << "TRRTRE" << endl;
        }
        std::string newRoute = readMessage(bc);
        if (newRoute.length() != 0)
        {
            try
            {

                if (checkCycle(newRoute) || checkInvalidRoute(newRoute))
                {
                    EV << t << endl;
                }

                sendToAGV(newRoute);
            }
            catch (std::exception &e1)
            {
                EV << e1.what() << endl;
            }
        }
    }
}

void HospitalControlApp::predictDispearTime()
{
    for (int i = 0; i < this->djisktra->numIVertices; i++)
    {
        int sum = 0;
        for (int j = 0; j < this->aroundIntersections[i].size(); j++)
        {
            int index = this->aroundIntersections[i][j];
            sum += this->crossings[index].count;
        }
        double density = sum / areas[i];
        double velocity = getAverageVelocityByDensity(density);
        double predict = Constant::LENGTH_CROSSING * sum * 0.5 / velocity;
        if (this->djisktra->timeWeightVertices[i] < predict || this->djisktra->expSmoothing->raisedTime[i] < 0)
        {
            if (predict == predict)
            {
                this->djisktra->expSmoothing->fromPedestrians[i] = predict;
            }
            else
            {
                this->djisktra->expSmoothing->fromPedestrians[i] = 0;
            }
        }
    }
}

double HospitalControlApp::getAverageVelocityByDensity(double density)
{
    //    y = 0.2 * x^2 - 1.1 * x + 1.7
    return 0.2 * density * density - 1.1 * density + 1.7;
}

void HospitalControlApp::sendToAGV(std::string content)
{
    if (content.length() != 0)
    {
        TraCIDemo11pMessage *rsuBeacon = new TraCIDemo11pMessage();
        if (sendBeacon != NULL)
        {
            if (sendBeacon->isScheduled())
            {
                cancelEvent(sendBeacon);
            }
            const char *ret = content.c_str(); // mergeContent(bc->getSenderAddress());
            rsuBeacon->setDemoData(ret);
            rsuBeacon->setSenderAddress(myId);
            BaseFrame1609_4 *WSM = new BaseFrame1609_4();
            WSM->encapsulate(rsuBeacon);
            populateWSM(WSM);
            send(WSM, lowerLayerOut);
        }
    }
}

void HospitalControlApp::onWSA(DemoServiceAdvertisment *wsa)
{
    // Your application has received a service advertisement from another car or RSU
    // code for handling the message goes here, see TraciDemo11p.cc for examples
}

void HospitalControlApp::handleSelfMsg(cMessage *msg)
{
    TraCIDemoRSU11p::handleSelfMsg(msg);
}

void HospitalControlApp::handlePositionUpdate(cObject *obj)
{
    TraCIDemoRSU11p::handlePositionUpdate(obj);
    // the vehicle has moved. Code that reacts to new positions goes here.
    // member variables such as currentPosition and currentSpeed are updated in the parent class
}

void HospitalControlApp::readLane(AGV *cur, std::string str)
{
    double localWait = cur->itinerary->localWait * 0.1;
    str.erase(str.find("_"));
    cur->itinerary->laneId = str;
    if (cur->itinerary->prevLane.compare(cur->itinerary->laneId) != 0)
    {
        if (cur->itinerary->prevLane.compare(cur->itinerary->station) == 0)
        {
            cur->passedStation = cur->itinerary->prevLane.length() > 0;
        }
        int idVertex = this->djisktra->findVertex(str);
        if (cur->itinerary->prevVertex != idVertex)
        {
            if (cur->itinerary->prevVertex != -1)
            {
                this->djisktra->expSmoothing->exponentialSmooth(
                    cur->itinerary->prevVertex,
                    this->djisktra->timeWeightVertices[cur->itinerary->prevVertex]);
                this->djisktra->expSmoothing->addWait(
                    cur->itinerary->prevVertex, -localWait);
            }
            cur->itinerary->localWait = 0;
            cur->itinerary->prevVertex = idVertex;
        }
        cur->itinerary->prevLane = str;
        if (cur->itinerary->prevLane[0] != ':')
        {
            cur->itinerary->prevEdge = str;
        }
    }
}

std::string HospitalControlApp::readMessage(TraCIDemo11pMessage *bc)
{
    double t = simTime().dbl();
    std::stringstream streamData(bc->getDemoData());
    std::string str = "";
    AGV *cur = NULL;
    for (auto a : vhs)
    {
        if (a->id.compare(std::to_string(bc->getSenderAddress())) == 0)
            cur = a;
    }
    if (cur == NULL)
    {
        cur = new AGV(); // 3 dong sau ghep thanh 1 phan ptkd cua AGV co tham so truyen vao
        cur->id = std::to_string(bc->getSenderAddress());
        cur->itinerary = new ItineraryRecord();
        cur->itinerary->localWait = 0;
        cur->createdTime = t;
        vhs.push_back(cur);
    }

    int i = 0;
    std::string newRoute = "";
    std::string tmp;
    while (getline(streamData, tmp))
        str += tmp;
    jute::jValue v = jute::parser::parse(str);
    std::string speed = v["speed"].as_string();
    std::string laneId = v["laneId"].as_string();
    std::string originalRouteId = v["originalRouteId"].as_string();
    if (cur->indexOfRoute == -1)
    {
        cur->indexOfRoute = getIndexInFlow(cur->id, originalRouteId);
    }
    double ratio = std::stod(v["ratio"].as_string());
    cur->ratio = ratio;
    double now = std::stod(v["now"].as_string());
    cur->now = now;
    std::string idMess = v["idMess"].as_string();
    std::string timeAtStation = v["atStation"].as_string();
    if (cur->atStation == 0 && timeAtStation.length() > 0)
    {
        cur->atStation = std::stod(timeAtStation);
        cur->passedStation = true;
    }
    readLane(cur, laneId);
    if (std::stod(speed) == 0)
    {
        cur->itinerary->localWait++;
        int currentIndex = cur->itinerary->prevVertex;
        this->djisktra->expSmoothing->addWait(currentIndex, 0.1);
        if (this->djisktra->expSmoothing->getWait(currentIndex) > this->djisktra->timeWeightVertices[currentIndex])
        {
            this->djisktra->expSmoothing->exponentialSmooth(currentIndex,
                                                            this->djisktra->timeWeightVertices[currentIndex]);
        }
    }
    newRoute = reRoute(cur, originalRouteId);
    //     if (newRoute.length() == 0) {
    // //        updateAGVSFMInfo(cur->id, cur->itinerary->prevLane);
    //         updateAGVSFMInfo(cur->id, laneId);
    //         std::cout << agvInfo.dump() << std::endl;
    //         string agvInfoString = agvInfo.dump();
    //         std::replace(agvInfoString.begin(), agvInfoString.end(), '"', '@');
    //         newRoute = "{\"agvInfo\" : \"" + agvInfoString + "\"}";
    //     }
    return newRoute;
}

bool HospitalControlApp::checkCycle(std::string route)
{
    std::vector<std::string> v = split(route, " ");
    for (int i = 0; i < v.size(); i++)
    {
        if (v[i].length() > 2)
        {
            for (int j = i + 1; j < v.size(); j++)
            {
                if (v[j].compare(v[i]) == 0 || ("^" + v[j]).compare(v[i]) == 0 || (v[j]).compare("^" + v[i]) == 0)
                {
                    return true;
                }
            }
        }
    }
    return false;
}

std::string HospitalControlApp::reRoute(AGV *cur,
                                        std::string routeId /*, double t*/)
{
    double t = simTime().dbl();
    if (this->djisktra->vertices[0][0] == cur->itinerary->laneId[0])
    {
        EV_TRACE << cur->itinerary->prevEdge << endl;
        if (!cur->passedStation || (cur->atStation + Constant::PAUSING_TIME > simTime().dbl()))
        {
            return ""; // skip this case, too complex as AGV is on an intersection
        }
    }

    int idOfI_Vertex = 0;
    if (cur->itinerary->laneId[0] == ':')
    {
        idOfI_Vertex = this->djisktra->findI_Vertex(cur->itinerary->prevEdge,
                                                    false);
    }
    else
    {
        idOfI_Vertex = this->djisktra->findI_Vertex(cur->itinerary->laneId,
                                                    false);
    }

    int src = -1, station = -1, exit = -1;
    int i = -1;
    if (idOfI_Vertex != cur->reRouteAt)
    {
        for (i = 0; i < this->djisktra->itineraries.size(); i++)
        {
            if (routeId.compare(std::get<0>(this->djisktra->itineraries[i])) == 0)
            {
                src = std::get<1>(this->djisktra->itineraries[i]);
                station = std::get<2>(this->djisktra->itineraries[i]);

                if (cur->itinerary->station.length() == 0)
                {
                    cur->itinerary->station = this->djisktra->vertices[station];
                    cur->itinerary->indexStation = station;
                }
                exit = std::get<3>(this->djisktra->itineraries[i]);
                cur->itinerary->exit = exit;
                break;
            }
        }
    }
    else
    {
        // cut from here
        return "";
    }

    if (cur->passedStation && Constant::STOP_AT_STATION)
    {
        bool stop = cur->atStation > 0;
        if (cur->atStation == 0)
        {
            cur->atStation = simTime().dbl();
            stop = true;
        }
        else
        {
            if (cur->atStation + Constant::PAUSING_TIME > simTime().dbl())
            {
                stop = true;
            }
            else
            {
                stop = false;
            }
        }
        if (stop)
        {
            return "{\"id\" : \"" + cur->id +
                   //"\", \"station\" : \"" + cur->itinerary->station +
                   "\", \"newRoute\" : \"0\"}";
        }
        else
        {
            return "{\"id\" : \"" + cur->id +
                   //"\", \"station\" : \"" + cur->itinerary->station +
                   "\", \"newRoute\" : \"" + Constant::CARRY_ON + "\"}";
        }
    }

    int nextDst = (cur->passedStation) ? exit : station;
    if (nextDst > -1)
    {
        if (idOfI_Vertex == nextDst)
        {
            return "";
        }
        this->djisktra->planOut(idOfI_Vertex, nextDst, cur->itinerary->laneId,
                                cur);

        std::string newRoute = this->djisktra->getRoute(
            /*this->djisktra*/ cur->traces[nextDst], cur->itinerary->laneId,
            idOfI_Vertex, nextDst, exit);
        if (checkInvalidRoute(newRoute))
        {
            EV << "TRACE" << endl;
        }
        if (nextDst != exit)
        {
            std::string futureLane = ""; // this->djisktra->vertices[nextDst];
            trim_right(newRoute);
            // Cam xoa comment nay: O day can dam bao la newRoute khong co space o cuoi
            for (int i = newRoute.length() - 1; i >= 0; i--)
            {
                if (newRoute[i] != ' ')
                {
                    futureLane = newRoute[i] + futureLane;
                }
                else
                {
                    break;
                }
            }
            this->djisktra->planOut(nextDst, exit, futureLane, cur);

            std::string lastPath = this->djisktra->getRoute(
                /*this->djisktra*/ cur->traces[exit], futureLane, nextDst,
                exit, exit);
            lastPath = this->excludeDuplication(futureLane, lastPath);
            newRoute = newRoute + " " + lastPath;
            if (checkInvalidRoute(newRoute))
            {
                EV << "TRACE" << endl;
            }
        }

        newRoute = removeAntidromic(newRoute);
        if (checkInvalidRoute(newRoute))
        {
            EV << "TRACE" << endl;
        }
        newRoute = removeLoop(newRoute);
        if (checkInvalidRoute(newRoute))
        {
            EV << "TRACE" << endl;
        }

        std::string weights = "";
        weights = this->djisktra->getWeights(newRoute, cur); //->ratio, cur->now, cur->itinerary->laneId);
        trim(newRoute);

        if (newRoute.length() == 0)
            return "";
        std::vector<std::string> list = split(newRoute, " ");
        if (list.size() <= 2)
        {
            // return "";
        }
        if (weights.length() > 2)
        {
            weights = ", \"weights\" : " + weights;
        }

        //     updateAGVSFMInfo(cur->id, cur->itinerary->prevLane);
        //    std::cout << agvInfo.dump() << std::endl;

        newRoute = "{\"id\" : \"" + cur->id + "\", \"newRoute\" : \"" + newRoute + "\"" + weights + "}";
        // string agvInfoString = agvInfo.dump();
        // std::replace(agvInfoString.begin(), agvInfoString.end(), '"', '@');
        // newRoute = "{\"id\" : \"" + cur->id +
        //           "\", \"agvInfo\" : \"" + agvInfoString +
        //           "\", \"newRoute\" : \"" + newRoute + "\"" + weights + "}";

        return newRoute;
    }
    return "";
}

void HospitalControlApp::updateAGVSFMInfo(string agvId, string laneId)
{
    if (agvInfo[agvId] == nullptr)
    {
        agvInfo[agvId] = laneId;
    }
    else
    {
        vector<string> tokens = Utility::splitSFM(agvInfo[agvId], " ");
        if (tokens.size() == 2)
        {
            if (laneId.compare(tokens[1]) != 0)
            {
                agvInfo[agvId] = tokens[1] + " " + laneId;
            }
        }
        else
        {
            string temp = agvInfo[agvId];
            Utility::removeChar(temp, '"');
            agvInfo[agvId] = temp + " " + laneId;
        }
    }
}

std::vector<std::vector<std::string>> HospitalControlApp::divideByAnyPrefix(
    const std::vector<std::string> &strings)
{
    std::vector<std::vector<std::string>> result;
    std::unordered_map<std::string, std::vector<std::string>> prefixMap;

    for (const std::string& str : strings) {
        std::size_t firstUnderscorePos = str.find('_');
        if (firstUnderscorePos != std::string::npos) {
            std::size_t secondUnderscorePos = str.find('_', firstUnderscorePos + 1);
            if (secondUnderscorePos != std::string::npos) {
                std::string prefix = str.substr(0, secondUnderscorePos);
                prefixMap[prefix].push_back(str);
            }
        }
    }

    for (auto it = prefixMap.begin(); it != prefixMap.end(); ++it) {
        result.push_back(it->second);
    }

    return result;
}

std::string HospitalControlApp::excludeDuplication(std::string futureLane,
                                                   std::string lastRoute)
{
    std::string lastPath = lastRoute;
    // bool emergencyFutureLane = false;
    bool emergencyLastPath = false;
    if (futureLane[0] == '^')
    {
        futureLane = futureLane.substr(1);
        // emergencyFutureLane = true;
    }
    if (lastPath[0] == '^')
    {
        lastPath = lastPath.substr(1);
        emergencyLastPath = true;
    }
    if (lastPath.find(futureLane + " ") != std::string::npos
        //&& newRoute.find(" " + futureLane) != std::string::npos
    )
    {
        if (!emergencyLastPath)
        {
            lastPath = lastPath.substr(futureLane.length() + 1);
        }
        else
        {
            lastPath = lastPath.substr(futureLane.length() + 2);
        }
    }
    return lastPath;
}

std::string HospitalControlApp::removeAntidromic(std::string input)
{
    std::vector<std::string> list = split(input, " ");
    int i = 0;
    std::string temp1, temp2;
    bool foundAntidromic = false;
    bool foundDuplication = false;
    bool change = false;
    int size = list.size();
    while (i < size - 1)
    {
        foundAntidromic = false;
        foundDuplication = false;
        for (i = 0; i < list.size() - 1; i++)
        {
            temp1 = list[i];
            temp2 = list[i + 1];
            if (temp1.compare(temp2) == 0)
            {
                foundDuplication = true;
                break;
            }
            if (list[i][0] == '-' && list[i + 1][0] != '-')
            {
                temp2 = '-' + temp2;
            }
            if (list[i][0] != '-' && list[i + 1][0] == '-')
            {
                temp1 = '-' + temp1;
            }
            if (temp1.compare(temp2) == 0)
            {
                foundAntidromic = true;
                break;
            }
        }
        if (foundDuplication)
        {
            change = true;
            list.erase(list.begin() + i);
            size--;
            i = 0;
        }
        if (foundAntidromic)
        {
            change = true;
            list.erase(list.begin() + i);
            list.erase(list.begin() + i);
            size -= 2;
            i = 0;
        }
    }
    if (!change)
        return input;
    if (list.size() >= 1)
    {
        std::string result = list[0];
        for (int j = 1; j < list.size(); j++)
        {
            result = result + " " + list[j];
        }
        return result;
    }
    else
    {
        return "";
    }
}
