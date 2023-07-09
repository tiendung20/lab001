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

#ifndef VEINS_INET_SFMAGV_H_
#define VEINS_INET_SFMAGV_H_

#include "lib/vecmath/vecmath.h"
#include <deque>
#include <vector>
#include "MovingObject.h"

#pragma once
class SFMAGV : public MovingObject
{
private:
    static int agvIdx;
    float width, length;
    int travelingTime;
    int numOfCollision;
    float acceleration;
    float thresholdDisToPedes;
    Point3f pointA, pointB, pointC, pointD;
    Vector3f direction;
    bool isCollision;
    int totalStopTime;
    int collisionStartTime;
    int prevTime;
    std::vector<float> hallwayCharcs;
    int mainAgv;

public:
    SFMAGV();

    ~SFMAGV();

    void setAgvSize(float width, float length);

    float getWidth() const { return width; }

    float getLength() const { return length; }

    void setTravelingTime(int travelingTime);

    int getTravelingTime() const { return travelingTime; }

    void setNumOfCollision(int numOfCollision);

    int getNumOfCollision() const { return numOfCollision; }

    void setAcceleration(float acceleration);

    double getAcceleration() const { return acceleration; }

    void setThresholdDisToPedes(float thresholdDisToPedes);

    float getThresholdDisToPedes() const { return thresholdDisToPedes; }

    int getTotalStopTime() const { return totalStopTime; }

    void setTotalStopTime(int totalStopTime);

    int getCollisionStartTime() const { return collisionStartTime; }

    void setCollisionStartTime(int collisionStartTime);

    int getPrevTime() const { return prevTime; }

    void setPrevTime(int prevTime);

    vector<float> getHallwayCharcs() const { return hallwayCharcs; }

    void setHallwayCharcs(vector<float> hallwayCharcs);

    void setPoints(Point3f pointA, Point3f pointB, Point3f pointC, Point3f pointD);

    void setDirection(float x, float y);

    int getMainAgv() const { return mainAgv; }

    void setMainAgv(int mainAgv);

    Vector3f getDirection() const { return direction; }

    Point3f getNearestPoint(Point3f position_i);

    bool isNearOtherObject(std::vector<Point3f> objectsPosition);

    using MovingObject::move;
    void move(float stepTime, std::vector<Point3f> agentsPosition, std::vector<Point3f> agvsPosition);
};


#endif /* VEINS_INET_SFMAGV_H_ */
