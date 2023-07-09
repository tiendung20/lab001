#include "SFMAGV.h"

using namespace std;

int SFMAGV::agvIdx = -1;

SFMAGV::SFMAGV()
{
    MovingObject();
    id = ++agvIdx;

    setAgvSize(0.35F, 0.75F);
    travelingTime = 0;
    numOfCollision = 0;
    thresholdDisToPedes = 0;
    isMoving = false;
    velocity.set(0.0, 0.0, 0.0);
    collisionStartTime = 0;
    totalStopTime = 0;
    mainAgv = 0;
}

SFMAGV::~SFMAGV()
{
    agvIdx--;
}

void SFMAGV::setAgvSize(float width, float length)
{
    this->width = width;
    this->length = length;
}

void SFMAGV::setTravelingTime(int travelingTime)
{
    this->travelingTime = travelingTime;
}

void SFMAGV::setNumOfCollision(int numOfCollision)
{
    this->numOfCollision = numOfCollision;
}

void SFMAGV::setAcceleration(float acceleration)
{
    this->acceleration = acceleration;
}

void SFMAGV::setThresholdDisToPedes(float thresholdDisToPedes)
{
    this->thresholdDisToPedes = thresholdDisToPedes;
}

void SFMAGV::setTotalStopTime(int totalStopTime)
{
    this->totalStopTime = totalStopTime;
};

void SFMAGV::setCollisionStartTime(int collisionStartTime)
{
    this->collisionStartTime = collisionStartTime;
};

void SFMAGV::setPrevTime(int prevTime)
{
    this->prevTime = prevTime;
};

void SFMAGV::setHallwayCharcs(vector<float> hallwayCharcs)
{
    this->hallwayCharcs = hallwayCharcs;
}

void SFMAGV::setPoints(Point3f pointA, Point3f pointB, Point3f pointC, Point3f pointD)
{
    this->pointA = pointA;
    this->pointB = pointB;
    this->pointC = pointC;
    this->pointD = pointD;
}

void SFMAGV::setDirection(float x, float y)
{
    this->direction.set(x, y, 0.0F);
}

void SFMAGV::setMainAgv(int mainAgv)
{
    this->mainAgv = mainAgv;
}

Point3f SFMAGV::getNearestPoint(Point3f positionAgent)
{
    vector<Point3f> points;
    points.push_back(pointA);
    points.push_back(pointB);
    points.push_back(pointC);
    points.push_back(pointD);
    Vector3f relativeEnd, relativePos, relativeEndScal, relativePosScal;
    float dotProduct;
    vector<Point3f> nearestPoint;
    Point3f point;
    for (int i = 0; i < points.size(); i++)
    {
        if (i < points.size() - 1)
        {
            relativeEnd = points[i + 1] - points[i];
        }
        else
        {
            relativeEnd = points[0] - points[i];
        }

        relativePos = positionAgent - points[i];

        relativeEndScal = relativeEnd;
        relativeEndScal.normalize();

        relativePosScal = relativePos * (1.0F / relativeEnd.length());

        dotProduct = relativeEndScal.dot(relativePosScal);

        if (dotProduct < 0.0)
            nearestPoint.push_back(points[i]);
        else if (dotProduct > 1.0)
        {
            if (i < points.size() - 1)
            {
                nearestPoint.push_back(points[i + 1]);
            }
            else
            {
                nearestPoint.push_back(points[0]);
            }
        }
        else
            nearestPoint.push_back((relativeEnd * dotProduct) + points[i]);
    }

    nearestPoint.push_back(position);

    point = nearestPoint[0];

    for (int i = 0; i < nearestPoint.size(); i++)
    {
        if (positionAgent.distance(nearestPoint[i]) < positionAgent.distance(point))
        {
            point = nearestPoint[i];
        }
    }

    return point;
}

bool SFMAGV::isNearOtherObject(vector<Point3f> objectsPosition)
{
    for (Point3f p : objectsPosition)
    {
        if (getNearestPoint(p).distance(p) < thresholdDisToPedes)
        {
            return true;
        }
    }
    return false;
}

void SFMAGV::move(float stepTime, vector<Point3f> agentsPosition, std::vector<Point3f> agvsPosition)
{
    Vector3f velocityDiff, desiredVelocity, e_ij;

    e_ij = getPath() - position;
    e_ij.normalize();

    desiredVelocity = e_ij * desiredSpeed;
    velocityDiff = e_ij * acceleration * stepTime;

    vector<Point3f> objectsPosition = agentsPosition;
    objectsPosition.insert(objectsPosition.end(), agvsPosition.begin(), agvsPosition.end());

    if (isNearOtherObject(objectsPosition))
    {
        if (!isCollision)
        {
            numOfCollision++;
        }
        isCollision = true;
        if (abs(velocity.x) >= abs(desiredVelocity.x) &&
            abs(velocity.y) >= abs(desiredVelocity.y))
        {
            position = position + velocity * stepTime;
            velocity = velocity - velocityDiff;
        }
        else
        {
            velocity.set(0, 0, 0);
        }
    }
    else
    {
        isCollision = false;
        if (velocity.length() < desiredVelocity.length())
        {
            position = position + velocity * stepTime;
            velocity = velocity + velocityDiff;
        }
        else
        {
            position = position + desiredVelocity * stepTime;
            velocity = desiredVelocity;
        }
    }
}
