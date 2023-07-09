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

#ifndef VEINS_INET_SOCIALFORCE_H_
#define VEINS_INET_SOCIALFORCE_H_

#include "Agent.h"
#include "Wall.h"
#include <vector>
#include <algorithm>
#include <map>
#include "SFMAGV.h"

#pragma once
class SocialForce {
private:
    std::vector<Agent *> crowd;
    std::vector<Wall *> walls;
    std::vector<SFMAGV *> agvs;

public:
    // SocialForce();
    ~SocialForce();

    void addAgent(Agent *agent);

    void addWall(Wall *wall);

    void addAGV(SFMAGV *agv);

    const std::vector<Agent *> getCrowd() const { return crowd; }

    int getCrowdSize() const { return crowd.size(); }

    const std::vector<Wall *> getWalls() const { return walls; }

    int getNumWalls() const { return walls.size(); }

    const std::vector<SFMAGV *> getAGVs() const { return agvs; }

    int getNumAGVs() const { return agvs.size(); }

    void removeAgent(int agentId);
    void removeCrowd(); // Remove all individuals and groups
    void removeWalls();

    void removeAGV();
    void removeAGVs();

    void moveCrowd(float stepTime);

    void moveAGVs(float stepTime);
};

#endif /* VEINS_INET_SOCIALFORCE1_H_ */
