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

#ifndef VEINS_INET_AGENT_H_
#define VEINS_INET_AGENT_H_

#include "lib/vecmath/vecmath.h"
#include <deque>
#include <vector>
#include "Wall.h"
#include "MovingObject.h"
#include "SFMAGV.h"

#pragma once
class Agent : public MovingObject
{
private:
    static int crowdIdx; // Keep track of 'crowd' vector index in 'SocialForce.h'
    float radius;
    float impatient;
    bool stopAtCorridor;

public:
    Agent();

    ~Agent();

    std::vector<Point3f> interDes;

    void setRadius(float radius);

    float getRadius() const { return radius; }

    void setImpatient(float impatient);

    float getImpatient() const { return impatient; }

    void setStopAtCorridor(bool stopAtCorridor);

    float getStopAtCorridor() const { return stopAtCorridor; }

    float getMinDistanceToWalls(vector<Wall *> walls, Point3f position, float radius);

    Vector3f getDrivingForce(const Point3f destination);
    Vector3f getAgentInteractForce(std::vector<Agent *> agents);
    Vector3f getWallInteractForce(std::vector<Wall *> walls);
    Vector3f getAgvInteractForce(std::vector<SFMAGV *> agvs);

    using MovingObject::move;
    void move(std::vector<Agent *> agents, std::vector<Wall *> walls, std::vector<SFMAGV *> agvs, float stepTime);
};


#endif /* VEINS_INET_AGENT1_H_ */
