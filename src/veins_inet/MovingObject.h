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

#ifndef VEINS_INET_MOVINGOBJECT_H_
#define VEINS_INET_MOVINGOBJECT_H_

#include "lib/vecmath/vecmath.h"
#include <deque>
#include <vector>

struct Waypoint
{
    Point3f position;
    float radius;
};

#pragma once
class MovingObject
{
protected:
    int id;
    Color3f color;
    float desiredSpeed;
    std::deque<Waypoint> path;
    Point3f destination;
    Point3f position;
    Vector3f velocity;
    bool isMoving;

public:
    MovingObject();
    ~MovingObject();

    int getId() const { return id; }

    Color3f getColor() const { return color; }

    void setColor(float red, float green, float blue);

    Point3f getPosition() const { return position; }

    void setPosition(float x, float y);

    Point3f getPath();

    void setPath(float x, float y, float radius);

    Point3f getDestination() const { return destination; };

    void setDestination(float x, float y);

    float getDesiredSpeed() const { return desiredSpeed; }

    void setDesiredSpeed(float speed);

    bool getIsMoving() const { return isMoving; }

    void setIsMoving(bool isMoving);

    Vector3f getVelocity() const { return velocity; }

    Point3f getAheadVector() const;

    float getOrientation();

    virtual void move(){};
};

#endif /* VEINS_INET_MOVINGOBJECT_H_ */
