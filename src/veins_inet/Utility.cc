#include "Utility.h"

#include <bits/stdc++.h>

#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <string>
#include <vector>
#include <regex>

#include "Constant.h"

using namespace std;
using namespace Utility;

// random float number between particular range
float Utility::randomFloat(float lowerBound, float upperBound) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(lowerBound, upperBound);
    return dis(gen);
}

// read map data file
std::map<std::string, std::vector<float>> Utility::readMapData(
        const char *fileName) {
    map<std::string, std::vector<float>> map;
    ifstream input(fileName);

    std::string delimiter = " ";

    int lineNo = 1;

    for (std::string line; getline(input, line);) {
        // cout << "Line: " << line << endl;
        vector<float> v;
        if (lineNo == 1) {
            v.push_back(stof(line));
            map["numI"] = v;
        } else if (lineNo == 2) {
            v.push_back(stof(line));
            map["walkwayWidth"] = v;
        } else {
            // cout << "Line: " <<line << endl;
            size_t pos = 0;
            std::string token;
            int count = 0;
            std::string juncId;
            while ((pos = line.find(delimiter)) != std::string::npos) {
                token = line.substr(0, pos);
                // std::cout << token << std::endl;
                if (count == 0) {
                    juncId.assign(token);
                }

                if (count > 1) {
                    v.push_back(stof(token));
                }
                line.erase(0, pos + delimiter.length());
                count++;
            }
            // cout << line << endl;
            v.push_back(stof(line));
            map[juncId] = v;
        }
        lineNo++;
    }

    return map;
}

std::vector<json> Utility::convertMapData(
        std::map<std::string, std::vector<float>> mapData) {
    std::vector<json> data;
    for (auto elem : mapData) {
        if (elem.first == "numI" || elem.first == "walkwayWidth") {
            continue;
        }
        int numOfHallway = elem.second.size();
        for (int i = 0; i < numOfHallway; i++) {
            if (!hallwaySameCharExists(elem.second[i], data)) {
                json temp;
                temp[elem.first + "_" + std::to_string(i)] = elem.second[i];
                data.push_back(temp);
            }
        }
    }
    return data;
}

bool Utility::hallwaySameCharExists(float hallwayLength,
        std::vector<json> data) {
    for (auto elem : data) {
        if (fabs(hallwayLength - (float) (elem.items().begin().value()))
                < 0.1) {
            return true;
        }
    }
    return false;
}

// read input file
json Utility::readInputData(const char *fileName) {
    std::ifstream f(fileName);
    json data = json::parse(f);

    return data;
}

// write end file
void Utility::writeResult(const char *fileName, std::vector<SFMAGV*> agvs,
        int totalAgents, int totalRunningTime) {
    ofstream output(fileName, ios::app);

    std::string delimiter = " - ";
    std::time_t now = std::chrono::system_clock::to_time_t(
            std::chrono::system_clock::now());

    if (true) {
        output << "\n\t*#* Completed on " << std::ctime(&now);

        for (SFMAGV *agv : agvs) {
            if (agv->getMainAgv() == 1) {
                if (agv->getNumOfCollision() == 0) {
                    agv->setTotalStopTime(0);
                }

                string message = "AGV ID " + std::to_string(agv->getId())
                        + " - " + convertTime(agv->getTravelingTime())
                        + " - Collisions "
                        + std::to_string(agv->getNumOfCollision()) + " - With "
                        + std::to_string(totalAgents)
                        + " agents - Total stop time "
                        + convertTime(agv->getTotalStopTime());
                output << message << endl;
            }
        }
    }

    output << "\t==> Total running time:  " << convertTime(totalRunningTime)
            << endl;

    output.close();
}

// calculate number of people in each flow
std::vector<int> Utility::getNumAgentsFlow(int totalPedestrian) {
    int numFlow = 6;

    std::vector<int> v(numFlow, 0);
    int j = 0;
    for (int i = 0; i < totalPedestrian; i++) {
        v[j] = v[j] + 1;
        j = j + 1;
        if (j == numFlow) {
            j = 0;
        }
    }
    return v;
}

// get list velocity of all pedestrians: type 0 - Discrete distribution, type 1
// - T distribution
std::vector<double> Utility::getAgentsSpeed(json inputData, int noAgents,
        float deviationParam) {
    vector<double> v;
    float perNoDisabilityWithoutOvertaking = float(inputData["p1"]["value"])
            * deviationParam;
    float perNoDisabilityWithOvertaking = float(inputData["p2"]["value"])
            * deviationParam;
    float perWalkingWithCrutches = float(inputData["p3"]["value"])
            * deviationParam;
    float perWalkingWithSticks = float(inputData["p4"]["value"])
            * deviationParam;
    float perWheelchairs = float(inputData["p5"]["value"]) * deviationParam;
    // float perTheBlind = inputData["p6"]["value"];
    float perTheBlind = 100
            - (perNoDisabilityWithoutOvertaking + perNoDisabilityWithOvertaking
                    + perWalkingWithCrutches + perWalkingWithSticks
                    + perWheelchairs);

    const int nrolls = 10000; // number of experiments

    std::default_random_engine generator;
    std::discrete_distribution<int> distribution {
            perNoDisabilityWithoutOvertaking, perNoDisabilityWithOvertaking,
            perWalkingWithCrutches, perWalkingWithSticks, perWheelchairs,
            perTheBlind };

    int p[6] = { };

    for (int i = 0; i < nrolls; ++i) {
        int number = distribution(generator);
        ++p[number];
    }

    std::map<int, float> map;
    map[0] = Constant::V1;
    map[1] = Constant::V2;
    map[2] = Constant::V3;
    map[3] = Constant::V4;
    map[4] = Constant::V5;
    map[5] = Constant::V6;

    for (int i = 0; i < 6; ++i) {
        // std::cout << i << ": " << p[i] * noAgents / nrolls << std::endl;
        // std::cout << i << ": " << std::string(p[i] * noAgents / nrolls, '*') <<
        // std::endl;
        for (int j = 0; j < p[i] * noAgents / nrolls; j++) {
            v.push_back(map[i]);
        }
    }
    int curSize = v.size();
    for (int i = 0; i < noAgents - curSize; i++) {
        v.push_back(map[0]);
    }

    return v;
}

std::vector<float> Utility::getWallCoordinates(float walkwayWidth,
        std::vector<float> juncData) {
    std::vector<float> v;

    std::vector<float> mapLimit = getMapLimit(walkwayWidth, juncData);

    float temp = walkwayWidth / 2;

    float leftWidthLimit = mapLimit[0];
    float rightWidthLimit = mapLimit[1];

    // Upper Wall
    v.insert(v.end(), { leftWidthLimit, temp, rightWidthLimit, temp });
    // Lower Wall
    v.insert(v.end(), { leftWidthLimit, -temp, rightWidthLimit, -temp });
    return v;
}

// Convert miliseconds to pretty form
std::string Utility::convertTime(int ms) {
    // 3600000 milliseconds in an hour
    long hr = ms / 3600000;
    ms = ms - 3600000 * hr;

    // 60000 milliseconds in a minute
    long min = ms / 60000;
    ms = ms - 60000 * min;

    // 1000 milliseconds in a second
    long sec = ms / 1000;
    ms = ms - 1000 * sec;

    return std::to_string(hr) + std::string("h ") + std::to_string(min)
            + std::string("m ") + std::to_string(sec) + std::string("s ")
            + std::to_string(ms) + std::string("ms");
}

// Position 0/1/2/3: Left/Right/Lower/Upper Limit
std::vector<float> Utility::getMapLimit(float walkwayWidth,
        std::vector<float> juncData) {
    std::vector<float> v;
    float leftWidthLimit = -1;
    float rightWidthLimit = -1;
    float lowerHeightLimit = -1;
    float upperHeightLimit = -1;

    float temp = walkwayWidth / 2;

    if (juncData.size() == 2) {
        leftWidthLimit = -juncData[0] - temp;
        rightWidthLimit = juncData[1] + temp;
    } else {
        leftWidthLimit = -juncData[0] - temp;
        rightWidthLimit = juncData[2] + temp;

        lowerHeightLimit = -juncData[1] - temp;
        upperHeightLimit = juncData[3] + temp;
    }

    v.insert(v.end(), { leftWidthLimit, rightWidthLimit, lowerHeightLimit,
            upperHeightLimit });

    return v;
}

// direction: 0 To Right, 1 To Left, 2 To Bottom, 3 To Top
// side: 0 Left side, 1 Center, 2 Right side
std::vector<float> Utility::getAgentDest(int direction, int side,
        float walkwayWidth, std::vector<float> juncData, bool stopAtCorridor) {
    std::vector<float> v;

    std::vector<float> mapLimit = getMapLimit(walkwayWidth, juncData);
    float leftWidthLimit = mapLimit[0];
    float rightWidthLimit = mapLimit[1];
    float lowerHeightLimit = mapLimit[2];
    float upperHeightLimit = mapLimit[3];

    float radius = 0.5;

    float latitude;
    float longitude;

    if (stopAtCorridor) {
    }

    switch (direction) {
    // To Right
    case 0: {
        if (stopAtCorridor) {
            latitude = Utility::randomFloat(0, rightWidthLimit - 2);
            longitude = Utility::randomFloat(-walkwayWidth / 3 + 0.3,
                    walkwayWidth / 3 - 0.3);
            v.insert(v.end(), { latitude, longitude, radius });
            return v;
        } else {
            latitude = Utility::randomFloat(rightWidthLimit + 2,
                    rightWidthLimit + 3);
        }
        switch (side) {
        case 0: {
            longitude = Utility::randomFloat(
                    walkwayWidth / 2 - walkwayWidth / 3, walkwayWidth / 2);
            v.insert(v.end(), { latitude, longitude, radius });
            return v;
            break;
        }
        case 1: {
            longitude = Utility::randomFloat(
                    -walkwayWidth / 2 + walkwayWidth / 3,
                    walkwayWidth / 2 - walkwayWidth / 3);
            v.insert(v.end(), { latitude, longitude, radius });
            return v;
            break;
        }
        case 2: {
            longitude = Utility::randomFloat(-walkwayWidth / 2,
                    -walkwayWidth / 2 + walkwayWidth / 3);
            v.insert(v.end(), { latitude, longitude, radius });
            return v;
            break;
        }
        default:
            break;
        }
        break;
    }
        // To Left
    case 1: {
        if (stopAtCorridor) {
            latitude = Utility::randomFloat(leftWidthLimit + 2, 0);
            longitude = Utility::randomFloat(-walkwayWidth / 3 + 0.3,
                    walkwayWidth / 3 - 0.3);
            v.insert(v.end(), { latitude, longitude, radius });
            return v;
        } else {
            latitude = Utility::randomFloat(leftWidthLimit - 3,
                    leftWidthLimit - 2);
        }
        switch (side) {
        case 0: {
            longitude = Utility::randomFloat(-walkwayWidth / 2,
                    -walkwayWidth / 2 + walkwayWidth / 3);
            v.insert(v.end(), { latitude, longitude, radius });
            return v;
            break;
        }
        case 1: {
            longitude = Utility::randomFloat(
                    -walkwayWidth / 2 + walkwayWidth / 3,
                    walkwayWidth / 2 - walkwayWidth / 3);
            v.insert(v.end(), { latitude, longitude, radius });
            return v;
            break;
        }
        case 2: {
            longitude = Utility::randomFloat(
                    walkwayWidth / 2 - walkwayWidth / 3, walkwayWidth / 2);
            v.insert(v.end(), { latitude, longitude, radius });
            return v;
            break;
        }
        default:
            break;
        }
        break;
    }
        // To Bottom
    case 2: {
        if (stopAtCorridor) {
            latitude = Utility::randomFloat(-walkwayWidth / 3 + 0.3,
                    walkwayWidth / 3 - 0.3);
            longitude = Utility::randomFloat(lowerHeightLimit + 2, 0);
            v.insert(v.end(), { latitude, longitude, radius });
            return v;
        } else {
            longitude = Utility::randomFloat(lowerHeightLimit - 2,
                    lowerHeightLimit - 1);
        }
        switch (side) {
        case 0: {
            latitude = Utility::randomFloat(walkwayWidth / 2 - walkwayWidth / 3,
                    walkwayWidth / 2);
            v.insert(v.end(), { latitude, longitude, radius });
            return v;
            break;
        }
        case 1: {
            latitude = Utility::randomFloat(
                    -walkwayWidth / 2 + walkwayWidth / 3,
                    walkwayWidth / 2 - walkwayWidth / 3);
            v.insert(v.end(), { latitude, longitude, radius });
            return v;
            break;
        }
        case 2: {
            latitude = Utility::randomFloat(-walkwayWidth / 2,
                    -walkwayWidth / 2 + walkwayWidth / 3);
            v.insert(v.end(), { latitude, longitude, radius });
            return v;
            break;
        }
        default:
            break;
        }
        break;
    }
        // To Top
    case 3: {
        if (stopAtCorridor) {
            latitude = Utility::randomFloat(-walkwayWidth / 3 + 0.3,
                    walkwayWidth / 3 - 0.3);
            longitude = Utility::randomFloat(0, upperHeightLimit - 2);
            v.insert(v.end(), { latitude, longitude, radius });
            return v;
        } else {
            longitude = Utility::randomFloat(upperHeightLimit + 1,
                    upperHeightLimit + 2);
        }
        switch (side) {
        case 0: {
            latitude = Utility::randomFloat(-walkwayWidth / 2,
                    -walkwayWidth / 2 + walkwayWidth / 3);
            v.insert(v.end(), { latitude, longitude, radius });
            return v;
            break;
        }
        case 1: {
            latitude = Utility::randomFloat(
                    -walkwayWidth / 2 + walkwayWidth / 3,
                    walkwayWidth / 2 - walkwayWidth / 3);
            v.insert(v.end(), { latitude, longitude, radius });
            return v;
            break;
        }
        case 2: {
            latitude = Utility::randomFloat(walkwayWidth / 2 - walkwayWidth / 3,
                    walkwayWidth / 2);
            v.insert(v.end(), { latitude, longitude, radius });
            return v;
            break;
        }
        default:
            break;
        }
        break;
    }
    default:
        return v;
        break;
    }
    return v;
}

// direction: 0 From Left, 1 From Right, 2 From Top, 3 From Bottom
std::vector<float> Utility::getAgentSrc(int direction, float totalLength,
        float subLength, float caravanWidth, float walkwayWidth,
        std::vector<float> juncData) {
    std::vector<float> v;
    float totalArea = totalLength * caravanWidth;
    float centerLength = totalLength - 2 * subLength;

    std::mt19937 gen(std::random_device { }());
    std::vector<double> chances { subLength * caravanWidth / totalArea,
            centerLength * caravanWidth / totalArea, subLength * caravanWidth
                    / totalArea };
    // Initialize to same length.
    std::vector<int> choices(chances.size());
    choices = { 0, 1, 2 };
    // size_t is suitable for indexing.
    std::discrete_distribution<std::size_t> d { chances.begin(), chances.end() };

    int sampled_value = choices[d(gen)];
    // cout << "sampled_value " << sampled_value << endl;

    std::vector<float> mapLimit = getMapLimit(walkwayWidth, juncData);
    float rightWidthLimit = mapLimit[1];
    float upperHeightLimit = mapLimit[3];

    float horLandmark = rightWidthLimit;
    float verLandmark = upperHeightLimit;
    if (totalLength > 40) {
        horLandmark = totalLength / 2;
        verLandmark = totalLength / 2;
    }
    // Calculate coordinates from Head to Tail of each Caravan
    switch (direction) {
    // From Left to Right
    case 0:
        switch (sampled_value) {
        case 0:
            v.insert(v.end(),
                    { Utility::randomFloat(-horLandmark + centerLength / 2,
                            -horLandmark + totalLength / 2),
                            Utility::randomFloat(-caravanWidth / 2,
                                    caravanWidth / 2) });
            return v;
            break;
        case 1:
            v.insert(v.end(),
                    { Utility::randomFloat(-horLandmark - centerLength / 2,
                            -horLandmark + centerLength / 2),
                            Utility::randomFloat(-caravanWidth / 2,
                                    caravanWidth / 2) });
            return v;
            break;
        case 2:
            v.insert(v.end(),
                    { Utility::randomFloat(-horLandmark - totalLength / 2,
                            -horLandmark - centerLength / 2),
                            Utility::randomFloat(-caravanWidth / 2,
                                    caravanWidth / 2) });
            return v;
            break;
        default:
            break;
        }
        break;

        // From Right to Left
    case 1:
        switch (sampled_value) {
        case 0:
            v.insert(v.end(),
                    { Utility::randomFloat(horLandmark - totalLength / 2,
                            horLandmark - centerLength / 2),
                            Utility::randomFloat(-caravanWidth / 2,
                                    caravanWidth / 2) });
            return v;
            break;
        case 1:
            v.insert(v.end(),
                    { Utility::randomFloat(horLandmark - centerLength / 2,
                            horLandmark + centerLength / 2),
                            Utility::randomFloat(-caravanWidth / 2,
                                    caravanWidth / 2) });
            return v;
            break;
        case 2:
            v.insert(v.end(),
                    { Utility::randomFloat(horLandmark + centerLength / 2,
                            horLandmark + totalLength / 2),
                            Utility::randomFloat(-caravanWidth / 2,
                                    caravanWidth / 2) });
            return v;
            break;
        default:
            break;
        }
        break;

        // From Top to Bottom
    case 2:
        switch (sampled_value) {
        case 0:
            v.insert(v.end(),
                    { Utility::randomFloat(-caravanWidth / 2, caravanWidth / 2),
                            Utility::randomFloat(verLandmark - totalLength / 2,
                                    verLandmark - centerLength / 2) });
            return v;
            break;
        case 1:
            v.insert(v.end(),
                    { Utility::randomFloat(-caravanWidth / 2, caravanWidth / 2),
                            Utility::randomFloat(verLandmark - centerLength / 2,
                                    verLandmark + centerLength / 2) });
            return v;
            break;
        case 2:
            v.insert(v.end(),
                    { Utility::randomFloat(-caravanWidth / 2, caravanWidth / 2),
                            Utility::randomFloat(verLandmark + centerLength / 2,
                                    verLandmark + totalLength / 2) });
            return v;
            break;
        default:
            break;
        }
        break;

        // From Bottom to Top
    case 3:
        switch (sampled_value) {
        case 0:
            v.insert(v.end(),
                    { Utility::randomFloat(-caravanWidth / 2, caravanWidth / 2),
                            Utility::randomFloat(
                                    -verLandmark + centerLength / 2,
                                    -verLandmark + totalLength / 2) });
            return v;
            break;
        case 1:
            v.insert(v.end(),
                    { Utility::randomFloat(-caravanWidth / 2, caravanWidth / 2),
                            Utility::randomFloat(
                                    -verLandmark - centerLength / 2,
                                    -verLandmark + centerLength / 2) });
            return v;
            break;
        case 2:
            v.insert(v.end(),
                    { Utility::randomFloat(-caravanWidth / 2, caravanWidth / 2),
                            Utility::randomFloat(-verLandmark - totalLength / 2,
                                    -verLandmark - centerLength / 2) });
            return v;
            break;
        default:
            break;
        }
        break;
    default:
        return v;
        break;
    }
    return v;
}

std::vector<float> Utility::getAgentColor(float desiredSpeed) {
    std::vector<float> v;

    // cout << fabs(V1 - desiredSpeed)  << endl;
    if (fabs(Constant::V1 - desiredSpeed) < 0.01) {
        v.insert(v.end(), { 0.0, 128.0, 0.0 });
        return v;
    } else if (fabs(Constant::V2 - desiredSpeed) < 0.01) {
        v.insert(v.end(), { 133.0, 22.0, 255.0 });
        return v;
    } else if (fabs(Constant::V3 - desiredSpeed) < 0.01) {
        v.insert(v.end(), { 216.0, 32.0, 42.0 });
        return v;
    } else if (fabs(Constant::V4 - desiredSpeed) < 0.01) {
        v.insert(v.end(), { 224.0, 183.0, 92.0 });
        return v;
    } else if (fabs(Constant::V5 - desiredSpeed) < 0.01) {
        v.insert(v.end(), { 130.0, 130.0, 130.0 });
        return v;
    } else {
        v.insert(v.end(), { 0.0, 0.0, 0.0 });
        return v;
    }
}

float getCoor(float x, float verAsymtote, float horAsymtote) {
    return horAsymtote * x / (x - verAsymtote);
}

std::vector<json> Utility::getAGVSrcDestCode(std::vector<float> juncData) {
    std::vector<json> v;
    int juncType = juncData.size();
    for (int i = 0; i < juncType; i++) {
        if (juncType == 4) {
            v.push_back( { { "src", i }, { "dest", 0 } });
            v.push_back( { { "src", i }, { "dest", 1 } });
            v.push_back( { { "src", i }, { "dest", 2 } });
        } else if (juncType == 3) {
            if (i == 0) {
                v.push_back( { { "src", i }, { "dest", 1 } });
                v.push_back( { { "src", i }, { "dest", 2 } });
            } else if (i == 1) {
                v.push_back( { { "src", i }, { "dest", 0 } });
                v.push_back( { { "src", i }, { "dest", 2 } });
            } else if (i == 2) {
                v.push_back( { { "src", i }, { "dest", 0 } });
                v.push_back( { { "src", i }, { "dest", 1 } });
            }
        } else if (juncType == 2) {
            v.push_back( { { "src", i }, { "dest", 1 } });
        }
    }

    return v;
}

// src = {0, 1, 2} ~ Go from Left, Bottom, Right side
std::vector<Point3f> Utility::getRouteAGV(int src, float position,
        float walkwayWidth, std::vector<float> juncData) {
    std::vector<Point3f> v;

    float leftWidthLimit = -juncData[0] - walkwayWidth / 2;
    float rightWidthLimit = juncData[1] + walkwayWidth / 2;

    if (src == 0) {
        if (fabs(position - 0) < 0.01) {
            v.insert(v.end(),
                    { Point3f(leftWidthLimit - 1, -walkwayWidth / 3, 0.0) });
        } else {
            v.insert(v.end(),
                    { Point3f(leftWidthLimit + position, -walkwayWidth / 3, 0.0) });
        }
        v.insert(v.end(),
                { Point3f(rightWidthLimit + 1, -walkwayWidth / 3, 0.0), Point3f(
                        rightWidthLimit + 2, -walkwayWidth / 3, 0.0) });
        return v;
    } else {
        if (fabs(position - 0) < 0.01) {
            v.insert(v.end(),
                    { Point3f(rightWidthLimit + 1, walkwayWidth / 3, 0.0) });
        } else {
            v.insert(v.end(),
                    { Point3f(rightWidthLimit - position, walkwayWidth / 3, 0.0) });
        }
        v.insert(v.end(), { Point3f(leftWidthLimit - 1, walkwayWidth / 3, 0.0),
                Point3f(leftWidthLimit - 2, walkwayWidth / 3, 0.0) });
        return v;
    }

    return v;
}

Point3f Utility::getIntermediateDes(Point3f position, float verWalkwayWidth,
        float horWalkwayWidth) {
    if (position.x > 0 && position.y > 0) {
        return Point3f(verWalkwayWidth / 3, horWalkwayWidth / 3, 0.0);
    } else if (position.x < 0 && position.y > 0) {
        return Point3f(-verWalkwayWidth / 3, horWalkwayWidth / 3, 0.0);
    } else if (position.x < 0 && position.y < 0) {
        return Point3f(-verWalkwayWidth / 3, -horWalkwayWidth / 3, 0.0);
    } else {
        return Point3f(verWalkwayWidth / 3, -horWalkwayWidth / 3, 0.0);
    }
}

bool Utility::isPositionErr(Point3f position, float hallwayWidth,
        int junctionType, std::vector<SFMAGV*> agvs) {
    float posLimit = hallwayWidth / 2;
    float negLimit = -hallwayWidth / 2;
    float x = position.x;
    float y = position.y;
    if (junctionType == 4) {
        bool con1 = x >= posLimit && y >= posLimit;
        bool con2 = x >= posLimit && y <= negLimit;
        bool con3 = x <= negLimit && y >= posLimit;
        bool con4 = x <= negLimit && y <= negLimit;
        if (con1 || con2 || con3 || con4) {
            return true;
        }
    } else if (junctionType == 3) {
        bool con1 = y >= posLimit;
        bool con2 = x >= posLimit && y <= negLimit;
        bool con3 = x <= negLimit && y <= negLimit;
        if (con1 || con2 || con3) {
            return true;
        }
    } else if (junctionType == 2) {
        if (y >= posLimit || y <= negLimit) {
            return true;
        }
    }

    for (SFMAGV *agv : agvs) {
        if (!agv->getIsMoving()) {
            continue;
        }
        float distance =
                (agv->getWidth() > agv->getLength()) ?
                        agv->getWidth() : agv->getLength();
        if (position.distance(agv->getPosition()) < distance / 2) {
            return true;
        }
    }

    return false;
}

int Utility::getNumAGVCompleted(std::vector<SFMAGV*> agvs) {
    int count = 0;
    for (SFMAGV *agv : agvs) {
        if (!agv->getIsMoving() && agv->getTravelingTime() != 0) {
            count = count + 1;
        }
    }
    return count;
}

int Utility::getNumTotalAgents(int min, int max) {
    int initValue = randomInt(min, max);
    // std::cout << "initValue: " << initValue << std::endl;
    float randomFactor1 = randomFloat(0.2, 0.8);
    int minNoAgents = (int) (1 - randomFactor1) * initValue;
    float randomFactor2 = randomFloat(0.2, 0.8);
    int maxNoAgents = (int) (1 + randomFactor2) * initValue;
    int side = randomInt(0, 1);
    if (side == 0) {
        return randomInt((minNoAgents + initValue) / 2, initValue);
    } else {
        return randomInt(initValue, (maxNoAgents + initValue) / 2);
    }
}

int Utility::randomInt(int from, int to) {
    std::random_device rd;  // obtain a random number from hardware
    std::mt19937 gen(rd()); // seed the generator
    std::uniform_int_distribution<> distr(from, to);
    return distr(gen);
}

std::vector<std::string> Utility::splitSFM(const std::string &s,
        const std::string &re) {
    std::regex regex(re);
    std::sregex_token_iterator it(s.begin(), s.end(), regex, -1);
    std::sregex_token_iterator end;
    return {it, end};
}

// Define a function to trim a string
std::string Utility::trim(const std::string &str) {
    // Define a variable to store the index of the first non-space character
    size_t first = str.find_first_not_of(" \n\r\t");

    // Check if the string is all spaces
    if (first == std::string::npos) {
        // Return an empty string
        return "";
    }

    // Define a variable to store the index of the last non-space character
    size_t last = str.find_last_not_of(" \n\r\t");

    // Return a substring from the first to the last non-space character
    return str.substr(first, last - first + 1);
}

void Utility::removeChar(string &str, char c) {
    // Define a regular expression pattern that matches the character
    regex pattern(string(1, c));

    // Use the regex_replace() function to replace it with an empty string
    str = regex_replace(str, pattern, "");
}

std::string Utility::exec(const char *cmd) {
    char buffer[128];
    std::string result = "";
    FILE *pipe = popen(cmd, "r");
    if (!pipe)
        throw std::runtime_error("popen() failed!");
    try {
        while (fgets(buffer, sizeof buffer, pipe) != NULL) {
            result += buffer;
        }
    } catch (...) {
        pclose(pipe);
        throw;
    }
    pclose(pipe);
    return result;
}

std::string Utility::cmdFindFile(std::string folderName, std::string endingStr) {
        std::string cmdFindFile = "find " + folderName +"/ -name *_" + endingStr + " -exec basename {} ';' -quit";
        std::string fileName = exec(cmdFindFile.c_str());
        return trim(fileName);
    }

vector<int> Utility::getMinMaxAgents(json timeFrameData, int hour,
        string juncId) {
    vector<int> v;
    int min = 0;
    int max = 0;
    for (auto &timeFrame : timeFrameData.items()) {
        vector<string> timePoints = splitSFM(timeFrame.key(), "_");
        int startTime = stoi(timePoints[0]);
        int endTime = stoi(timePoints[1]);
        if (startTime <= hour && endTime > hour) {
            try {
                min = timeFrameData[timeFrame.key()][juncId]["min"];
                max = timeFrameData[timeFrame.key()][juncId]["max"];
            } catch (const std::exception &e) {
                min = 2;
                max = 5;
            }
            break;
        }
    }
    if (min == 0 && max == 0) {
        min = 2;
        max = 5;
    }
    v.insert(v.end(), { min, max });
    return v;
}
