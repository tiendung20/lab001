#ifndef UTILITES_H
#define UTILITES_H

#include <vector>
#include <iostream>
#include <array>
#include <lib/nlohmann/json.hpp>
#include "lib/vecmath/vecmath.h"
#include <map>

#include "SFMAGV.h"

using json = nlohmann::json;

#pragma once
namespace Utility
{
    float randomFloat(float lowerBound, float upperBound);

    std::map<std::string, std::vector<float>> readMapData(const char *fileName);

    std::vector<json> convertMapData(std::map<std::string, std::vector<float>> mapData);

    bool hallwaySameCharExists(float hallwayLength, std::vector<json> data);

    json readInputData(const char *fileName);

    void writeResult(
        const char *fileName, std::vector<SFMAGV *> data, int totalAgents, int totalRunningTime);

    std::vector<int> getNumAgentsFlow(int totalPedestrian);

    std::vector<double> getAgentsSpeed(json inputData, int noAgents, float deviationParam);

    std::vector<float> getWallCoordinates(float walkwayWidth, std::vector<float> juncData);

    std::string convertTime(int milliseconds);

    std::vector<float> getMapLimit(float walkwayWidth, std::vector<float> juncData);

    std::vector<float> getAgentDest(int direction, int side, float walkwayWidth, std::vector<float> juncData, bool stopAtCorridor);

    std::vector<float> getAgentSrc(int direction, float totalLength, float subLength, float caravanWidth, float walkwayWidth, std::vector<float> juncData);

    std::vector<float> getAgentColor(float desiredSpeed);

    std::vector<json> getAGVSrcDestCode(std::vector<float> juncData);

    std::vector<Point3f> getRouteAGV(int src, float position, float walkwayWidth, std::vector<float> juncData);

    Point3f getIntermediateDes(Point3f position, float verWalkwayWidth, float horWalkwayWidth);

    bool isPositionErr(Point3f position, float walkwayWidth, int junctionType, std::vector<SFMAGV *> agvs);

    int getNumAGVCompleted(std::vector<SFMAGV *> agvs);

    int getNumTotalAgents(int min, int max);

    int randomInt(int from, int to);

    std::vector<std::string> splitSFM(const std::string& s, const std::string& re);

    std::string exec(const char* cmd);

    std::string cmdFindFile(std::string folderName, std::string endingStr);

    std::string trim(const std::string &str);

    void removeChar(string& str, char c);

    vector<int> getMinMaxAgents(json timeFrameData, int hour, string juncId);

    
};

#endif
