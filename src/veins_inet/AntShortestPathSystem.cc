/*
 * AcoPath: Shortest path calculation using Ant Colony Optimization
 * Copyright (C) 2014-2021 by Constantine Kyriakopoulos
 * zfox@users.sourceforge.net
 * @version 0.9.1
 *
 * @section LICENSE
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "AntShortestPathSystem.h"

/**
 * Constructor initialising the topology from external file.
 *
 * @param filename A file containing the topology in JSON format
 * @param ants Number of ants to unlease in each iteration
 * @param iterations Number of iterations
 */
AntShortestPathSystem::AntShortestPathSystem(const std::string& filename, int ants, int iterations)
{
    try
    {
        //initTopo(filename);
        for(auto& edge : adaptiveEdges)
            edge2phero.insert(std::make_pair(edge, static_cast<double>(PHERO_QUANTITY)));
    }
    catch(std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }

    init(ants, iterations);
}

/**
 * Constructor initialising the topology from external graph.
 *
 */
AntShortestPathSystem::AntShortestPathSystem()
{
    try
    {
        //create a clone of adjList from updated graph
        initAdaptiveEdges(adjList, vertices, timeWeightVertices, &countEdges);
        for(auto& edge : adaptiveEdges)
            edge2phero.insert(std::make_pair(edge, static_cast<double>(PHERO_QUANTITY)));
    }
    catch(std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }

    init(ANTS, ITERATIONS);
}

/**
 * Constructor w/out initialising the topology.
 *
 * @param ants Number of ants to unlease in each iteration
 * @param iterations Number of iterations
 */
AntShortestPathSystem::AntShortestPathSystem(int ants, int iterations)
{
    init(ants, iterations);
}

/**
 * Empty destructor.
 */
AntShortestPathSystem::~AntShortestPathSystem() { }

/**
 * Initialiser for ants, iterations and random numner generator.
 *
 * @param ants Number of ants to unlease in each iteration
 * @param iterations Number of iterations
 */
void AntShortestPathSystem::init(int ants, int iterations)
{
    if(ants > 0 && iterations > 0)
    {
        this->ants = ants;
        this->iterations = iterations;
    }
    else
    {
        this->ants = ANTS;
        this->iterations = ITERATIONS;
    }

    std::random_device rd;
    gen = std::mt19937_64(rd());
}

/**
 * Finds the best path from a source node to a destination using the Ant System.
 *
 * @param start Path's starting point
 * @param end Path's end point
 * @return std::vector<int> The best path
 */
std::vector<int> AntShortestPathSystem::path(int start, int end)
{
    std::vector<int> bestPath;
    double shortest = std::numeric_limits<double>::max();
    // For the predefined number of iterations

    for(int i = 0; i < iterations; ++i)
    {
        std::map<int, std::vector<int> > antTraces;
        std::map<int, double> tourLengths;
        // Release ants from source node and let them traverse the graph
        // structure to reach a destination
        for(int j = 1; j <= ants; ++j)
        {
            // This trace will be used by this ant to store its node sequence
            std::vector<int> antTrace;
            goAnt(start, end, antTrace);

            if(antTrace.size() > 1 && antTrace.front() == start
                    && antTrace.back() == end)
            {
                // Destination reached, so calculate tour length and keep the shortest one
                antTraces.insert(std::make_pair(j, antTrace));
                tourLengths[j] = calcTourLength(antTrace);
                if(tourLengths[j] > 0 && tourLengths[j] < shortest)
                {
                    shortest = tourLengths[j];
                    bestPath = antTrace;
                }
            }
            else
            {
                // Well, this ant failed to reach its destination
                antTraces.insert(std::make_pair(j, std::vector<int>()));
                tourLengths[j] = 0;
            }
        }

        // Update pheromone trails upon the correct node sequences
        updateTrails(antTraces, tourLengths);
    }

    if(bestPath.size() > 0){
        EV<<"found something!!!"<<endl;
    }

    return bestPath;
}

/**
 * Finds the best path from a source node to a destination using the Ant System.
 *
 * @param start Path's starting point
 * @param end Path's end point
 * @return std::vector<int> The best path
 */
void AntShortestPathSystem::planOut(//std::vector <Quad> adjList[],
        int source, int target, std::string currLane, AGV* cur){
    cur->memset(numVertices);
    double tempW; int tempIndex; std::string tempTrace;
    for (std::vector<Quad>::iterator it = adjList[source].begin(); it != adjList[source].end(); it++){
        tempW = std::get<0>(*it);
        tempTrace = std::get<3>(*it);
        tempIndex = std::get<2>(*it);
        if(!Constant::SHORTEST_PATH){
            timeWeightVertices[tempIndex] = this->expSmoothing->getDampingValue(tempIndex, timeWeightVertices[tempIndex], vertices[tempIndex]);
        }
    }
    if(!isWorking()){
        cur->ShortestPath[source] = 0;
    }
    this->insertRequest(source, target, cur->id);
    if(target != cur->itinerary->exit){
        this->insertRequest(target, cur->itinerary->exit, cur->id);
    }
    assert(!(this->hasIdenticalReq()));
    if(this->canExecuteReqs()){
        EV<<"prepare for parallelization ACO";
        this->updateWeights(this->timeWeightVertices);
        std::vector<Request> work = this->kickOff();
        int N = work.size();
        Request jobs[N];
        for(int i = 0; i < N; i++){
            jobs[i] = work[i];
        }
        int src, dst;
        //std::vector<int> r;
        for(int i = 0; i < N; i++){
            src = std::get<0>(jobs[i]);
            dst = std::get<1>(jobs[i]);
            this->path(src, dst);
        }
        markFinishedReqs();
    }

} // ACO

/*
 * This func will mark that all WAITING_FOR_PROCESSING reqs will become FINISHED
 * @pre: the kick off must collect all non-expired WAITING_FOR_PROCESSING reqs
 * as well as remove all expired WAITING_FOR_PROCESSING reqs
 */
void AntShortestPathSystem::markFinishedReqs(){
    double t = simTime().dbl();
    for(std::vector<Request>::iterator it = allRequests.begin(); it != allRequests.end(); it++){
        STATE_OF_REQUEST state = std::get<4>(*it);
        if(state == WAITING_FOR_PROCESSING){
            std::get<4>(*it) = FINISHED;
            std::get<3>(*it) = t;
        }
    }
}

/*
 * Mark all waiting requests that they are being processed by changing the state
 * @pre: the requests are not being worked, and there is already several waiting requests
 * @post: calling the path of AntShortestPathSystem and all expired reqs disappear
 */
std::vector<Request> AntShortestPathSystem::kickOff(){
    double t = simTime().dbl();
    std::vector<Request> result;
    std::vector<int> expiredReqs;
    int i = 0;
    for(std::vector<Request>::iterator it = allRequests.begin(); it != allRequests.end(); it++){
        STATE_OF_REQUEST state = std::get<4>(*it);
        double createdTime = std::get<3>(*it);
        if(state == WAITING_FOR_PROCESSING){
            if(t - createdTime < Constant::DELAY){
                std::get<4>(*it) = BEING_PROCESSED;
                std::get<3>(*it) = t;
                result.push_back(*it);
            }
            else{
                expiredReqs.push_back(i);
            }
        }
        i++;
    }
    this->removeExpiredRequests(&expiredReqs);
    return result;
}

std::string AntShortestPathSystem::getRoute(std::string trace, std::string currentLane, int currentVertex, int nextVertex, int exitVertex){
    return "";
}

/**
 * Clears instance's state
 */
void AntShortestPathSystem::clear()
{
    edge2phero.clear();
    adaptiveEdges.clear();
}

/**
 * Returns the amount of pheromone difference according to tour's length.
 *
 * @param length Tour's length produced by an ant
 * @return double Amount of pheromone
 */
double AntShortestPathSystem::diffPheromone(double length)
{
    return PHERO_QUANTITY / length;
}

/**
 * Updates pheromone levels upon all graph edges.
 *
 * @param antTraces Created traces by ants
 * @param tourLengths The length of traces
 */
void AntShortestPathSystem::updateTrails(std::map<int, std::vector<int>>& antTraces,
                             std::map<int, double>& tourLengths)
{
    // First, evaporate all existing pheromone levels
    for(auto& pair : edge2phero)
        pair.second *= (1 - EVAPO_RATE);

    // Then, increase pheromone level upon correct paths
    for(auto& pair : edge2phero)
    {
        int edgeStart = pair.first.edgeStart;
        int edgeEnd = pair.first.edgeEnd;
        std::map<int, std::vector<int>>::iterator ait = antTraces.begin();
        while(ait != antTraces.end())
        {
            // For every ant trace
            std::vector<int> trace = (*ait).second;
            if(trace.size() <= 1)
            {
                ait++;
                continue;
            }

            // In case it's valid, add an amount of pheromone that depends on
            // each tour length
            for(unsigned int i = 0; i < trace.size() - 1; ++i)
                if(trace.at(i) == edgeStart && trace.at(i + 1) == edgeEnd)
                    edge2phero[pair.first] += diffPheromone(tourLengths[(*ait).first]);
            ait++;
        }
    }
}

/**
 * Recursive method that finds a suitable trace from a starting
 * point to a specific destination by unleashing an ant.
 *
 * @param start Path's starting point
 * @param end Path's destination
 * @param trace Container where path's nodes will be stored
 */
void AntShortestPathSystem::goAnt(int start, int end, std::vector<int>& trace)
{
    // Detect cycles and give up this attempt
    if(isCyclic(start, trace))
    {
        trace.clear();
        return;
    }
    // Destination reached
    if(start == end && trace.size() > 0)
    {
        trace.push_back(start);
        return;
    }

    // Get available physical neighbours
    std::vector<int> neighs = availNeighbours(start);
    double probs[neighs.size()];
    int index = 0;
    // Produce a transition probability to each one
    for(int neigh : neighs)
        probs[index++] = prob(start, neigh, neighs);

    std::uniform_real_distribution<> distro(0, 1);
    double value = distro(gen);
    // Sort probabilities in range [0, 1] and use a uniform dice to
    // pick up an index domain
    index = 0; double sum = 0;
    for(; index < (int)neighs.size(); ++index)
    {
        sum += probs[index];
        if(value <= sum)
            break;
    }

    // This index belongs to the chosen neighbour
    int chosenNeighbour = (neighs.size() > 0) ? neighs[index] : -1;
    if(chosenNeighbour == -1)
    {
        // No available neighbour found, so give up
        trace.clear();
        return;
    }

    // Recurse to the next neighbour
    trace.push_back(start);
    goAnt(chosenNeighbour, end, trace);
}

/**
 * Calculates path's length.
 *
 * @param tour Container with path's nodes
 * @return double Tour's length
 */
double AntShortestPathSystem::calcTourLength(std::vector<int>& tour)
{
    if(tour.size() <= 1)
        return 0;

    double weightSum = 0;
    for(unsigned int i = 0; i < tour.size() - 1; ++i)
    {
        // Find the edge that starts with current trace node
        auto it = std::find_if(edge2phero.cbegin(), edge2phero.cend(),
                [&tour, i](std::pair<Edge, double> pair)
                {
                    return pair.first.edgeStart == tour[i]
                            && pair.first.edgeEnd == tour[i + 1];
                });

        if(it != edge2phero.cend()){
            //ThanhNT 12th Nov
            weightSum += (*it).first.weightSrc;
            //Endof ThanhNT 12th Nov
            weightSum += (*it).first.weightEdge;
        }
    }

    return weightSum;
}

/**
 * Returns the probability of selecting the second node as destination, from
 * the first one.
 *
 * @param edgeStart The edge's starting point
 * @param edgeEnd The edge's end point
 * @return double The probability
 */
double AntShortestPathSystem::prob(int edgeStart, int edgeEnd, std::vector<int>& neighs)
{
    double numerator = std::pow(pheromone(edgeStart, edgeEnd), A_PAR)
            * std::pow(heuInfo(edgeStart, edgeEnd), B_PAR);

    double denumerator = 0;
    //std::vector<int> neighs = availNeighbours(edgeStart);
    for(int neigh : neighs)
        denumerator += std::pow(pheromone(edgeStart, neigh), A_PAR)
                * std::pow(heuInfo(edgeStart, neigh), B_PAR);

    return numerator / denumerator;
}

/**
 * Returns the 'amount' of heuristic information from first input node to
 * the second.
 *
 * @param edgeStart The edge's starting point
 * @param edgeEnd The edge's end point
 * @return double The amount of heuristic information
 */
double AntShortestPathSystem::heuInfo(int edgeStart, int edgeEnd)
{
    // Find the edge with this lambda function and use its weight
    auto it = std::find_if(edge2phero.cbegin(), edge2phero.cend(),
            [edgeStart, edgeEnd](std::pair<Edge, double> pair)
            {
                return pair.first.edgeStart == edgeStart
                        && pair.first.edgeEnd == edgeEnd;
            });

    //ThanhNT 12th Nov
    //set comment to the original statement
    //return it != edge2phero.cend() ? 1 / (*it).first.weightEdge : 0;
    //declare a denominator
    double denominator = (*it).first.weightSrc + (*it).first.weightEdge;
    return it != edge2phero.cend() ? 1 / denominator : 0;
    //Endof ThanhNT 12thNov
}

/**
 * Returns the amount of pheromone from first input node to the second.
 *
 * @param edgeStart The edge's starting point
 * @param edgeEnd The edge's end point
 * @return double The amount of pheromone
 */
double AntShortestPathSystem::pheromone(int edgeStart, int edgeEnd)
{
    // Find the edge with this lambda function and return its pheromone level
    auto it = std::find_if(edge2phero.cbegin(), edge2phero.cend(),
            [edgeStart, edgeEnd](std::pair<Edge, double> pair)
            {
                return pair.first.edgeStart == edgeStart
                        && pair.first.edgeEnd == edgeEnd;
            });

    return it != edge2phero.cend() ? (*it).second : 0;
}

/**
 * Finds all available neighbours of input node.
 *
 * @param node The input node
 * @return std::vector<int> Container with nodes
 */
std::vector<int> AntShortestPathSystem::availNeighbours(int node)
{
    std::vector<int> neighbours;
    //std::vector<std::string> traces;

    // Find all edges that start from the input node and return its
    // other endpoints
    std::for_each(edge2phero.cbegin(), edge2phero.cend(),
            [&neighbours, //&traces,
                 node](std::pair<Edge, double> pair)
            {
                if(pair.first.edgeStart == node){
                    neighbours.push_back(pair.first.edgeEnd);
                    //traces.push_back(pair.first.dst);
                }
            });

    return neighbours;
}

/**
 * Detects if a cycle is formed inside the sequence of nodes.
 *
 * @param nodes The sequence of nodes
 * @return bool The indication of a cyclic sequence
 */
bool AntShortestPathSystem::isCyclic(int nd, const std::vector<int>& nodes)
{
    std::set<int> uniqueNodes;
    uniqueNodes.insert(nd);
    for(int node : nodes)
        uniqueNodes.insert(node);

    return nodes.size() + 1 != uniqueNodes.size();
}

/**
 * Inserts an edge.
 *
 * @param src Source node
 * @param dest Destination node
 * @param weight Weight for the edge
 */
/*void AntShortestPathSystem::insertEdge(int src, int dest, double weight)
{
    AdaptiveSystem::insertEdge(src, dest, weight);
    edge2phero.clear();
    for(auto& edge : adaptiveEdges)
        edge2phero.insert({edge, static_cast<double>(PHERO_QUANTITY)});
}*/
