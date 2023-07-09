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

#include "NishidaDStarLite.h"

DStarLite::DStarLite(std::vector<std::vector<TimeSpace>> *graph, double start,
        Objective *target, double dt, Node s_start, Node s_goal) {
    this->graph = graph;
    this->start = start;
    this->target = target;
    this->dt = dt;
    this->s_start = s_start;
    this->s_goal = s_goal;
}

double DStarLite::getG(Node u) {
    if (g.count(u) > 0) {
        return g[u];
    }
    return 1.0 / 0.0;
}

double DStarLite::getRHS(Node u) {
    if (rhs.count(u) > 0) {
        return rhs[u];
    }
    return 1.0 / 0.0;
}

double DStarLite::weightEdge(Node a, Node b) {
    double alpha = target->alpha;
    double beta = target->beta;
    double gamma = target->gamma;
    double w = 0;
    if (std::get<1>(a) != std::get<1>(b)) {
        w += alpha * std::max(0.0, target->earliness - std::get<0>(b) * dt);
        w += gamma * std::max(0.0, std::get<0>(b) * dt - target->tardiness);
    } else {
        w += beta * (std::get<0>(b) * dt - start);
        w += gamma * std::max(0.0, std::get<0>(b) * dt - target->tardiness);
    }
    return 0;
}

double DStarLite::getH(Node a, Node b) {
    return fabs(a.first - b.first);
}

void DStarLite::setG(Node u, double v) {
    if (g.count(u) > 0) {
        g[u] = v;
    } else {
        g.insert( { u, v });
    }
}

void DStarLite::setRHS(Node u, double v) {
    if (rhs.count(u) > 0) {
        rhs[u] = v;
    } else {
        rhs.insert( { u, v });
    }
}

key DStarLite::calculateKey(Node u) {
    double val = std::min(getRHS(u), getG(u));
    return key(val + getH(s_start, u) + k_m, val);
}

void DStarLite::initialize() {
    k_m = 0;
    if (getRHS(s_goal) != 0) {
        setRHS(s_goal, 0);
        U.insert( { s_goal, key(getH(s_start, s_goal), 0) });
    }
}

std::vector<Node> DStarLite::getSucc(Node u) {
    std::vector<Node> list;
    for (Index i : std::get<3>(graph->at(std::get<0>(u)).at(std::get<1>(u)))) {
        list.push_back(Node(std::get<1>(i), std::get<0>(i)));
    }
    return list;
}

std::vector<Node> DStarLite::getPred(Node u) {
    std::vector<Node> list;
    for (Index i : std::get<4>(graph->at(std::get<0>(u)).at(std::get<1>(u)))) {
        list.push_back(Node(std::get<1>(i), std::get<0>(i)));
    }
    return list;
}

Node DStarLite::top() {
    key tmp = { 1.0 / 0.0, 1.0 / 0.0 };
    Node temp;
    for (auto &pair : U) {
        if (pair.second < tmp) {
            tmp = pair.second;
            temp = pair.first;
        }
    }
    return temp;
}

void DStarLite::updateVertex(Node u) {
    if (getG(u) != getRHS(u) && U.count(u) > 0) {
        U[u] = calculateKey(u);
    } else if (getG(u) != getRHS(u) && U.count(u) <= 0) {
        U.insert( { u, calculateKey(u) });
    } else if (getG(u) == getRHS(u) && U.count(u) > 0) {
        U.erase(u);
    }
}

void DStarLite::computeShortestPath() {

    while (true) {
        Node u = top();

        if (U[u] >= calculateKey(s_start) && getRHS(s_start) <= getG(s_start)) {
            break;
        }

        key k_old = U[u];
        key k_new = calculateKey(u);

        if (k_old < k_new) {
            U[u] = k_new;
        } else if (getG(u) > getRHS(u)) {
            setG(u, getRHS(u));
            U.erase(u);
            for (Node n : getPred(u)) {
                if (n != s_goal) {
                    setRHS(n,
                            std::min(getRHS(n),
                                    abs(u.first - n.first) + getG(u)));
                }
                updateVertex(n);
            }
        } else {
            double g_old = getG(u);
            setG(u, 1.0 / 0.0);
            updateVertex(u);
            for (Node n : getPred(u)) {
                if (getRHS(n) == abs(u.first - n.first) + g_old) {
                    if (n != s_goal) {
                        double tmp = 1.0 / 0.0;
                        double tmp2;
                        for (Node nd : getSucc(n)) {
                            tmp2 = getG(nd) + abs(nd.first - n.first);
                            if (tmp2 < tmp) {
                                tmp = tmp2;
                            }
                        }
                        setRHS(n, tmp);
                    }
                }
                updateVertex(n);
            }
        }
    }
}

Node DStarLite::searchGoalNode() {
    std::list<Node> queue;

    queue.push_back(s_start);

    while (!queue.empty()) {
        Node k = queue.front();
        if (k.second == s_goal.second) {
            return k;
        }
        queue.pop_front();
        for (const Node &n : getSucc(k)) {
            if (n.second != k.second) {
                queue.push_back(n);
            }
        }
    }
    return Node( { -1, -1 });
}

void DStarLite::main() {
    s_last = s_start;
    Node goal = searchGoalNode();
    if (goal.first == -1 && goal.second == -1)
        return;
    if (goal.first > s_goal.first) {
        s_goal = goal;
    }
    initialize();
    computeShortestPath();
    while (s_last.second != s_goal.second) {
        path.push_back(s_last);
        double tmp = 1.0 / 0.0;
        double tmp2;
        Node i = Node( { -1, -1 });
        for (const Node &n : getSucc(s_last)) {
            tmp2 = abs(n.first - s_last.first) + getG(n)
                    + weightEdge(s_last, n);
            if (tmp2 < tmp) {
                i = n;
                tmp = tmp2;
            }
        }
        s_last = i;
        if (s_last.first == -1 && s_last.second == -1) {
            path.clear();
            return;
        }
    }
}

void DStarLite::reMain() {
    s_last = s_start;
    Node goal = searchGoalNode();
    if (goal.first == -1 && goal.second == -1)
        return;
    if (goal.first > s_goal.first) {
        return;
    }
    while (s_last.second != s_goal.second) {
        path.push_back(s_last);
//        std::cout << s_last.first << " and " << s_last.second << endl;
        double tmp = 1.0 / 0.0;
        double tmp2;
        Node i = Node( { -1, -1 });
        for (const Node &n : getSucc(s_last)) {
            tmp2 = abs(n.first - s_last.first) + getG(n)
                    + weightEdge(s_last, n);
            if (tmp2 < tmp) {
                i = n;
                tmp = tmp2;
            }
        }
        s_last = i;
        if (s_last.first == -1 && s_last.second == -1) {
            path.clear();
            return;
        }
    }
}

void DStarLite::update(Node u) {
    double infinity = 1.0 / 0.0;
    if (getG(u) == infinity) {
        return;
    }
    if (u != s_goal) {
        double tmp = 1.0 / 0.0;
        double tmp2;
        for (Node ud : getSucc(u)) {
            tmp2 = getG(ud) + abs(ud.first - u.first);
            if (tmp2 < tmp) {
                tmp = tmp2;
            }
        }
        setRHS(u, tmp);
        std::cout << u.first << " --- " << u.second << " +++ " << getG(u)
                << endl;
        updateVertex(u);
    }
}

std::vector<std::string> DStarLite::convert() {
    std::vector<std::pair<std::string, double>> result;
    for (int i = 0; i < path.size(); i++) {
        int index0 = path.at(i).first;
        double t = index0 * dt;
        int index1 = path.at(i).second;
        std::string name = std::get<0>(graph->at(index0).at(index1));
        result.push_back(std::make_pair(name, t));
    }
    path.clear();
    std::vector<std::string> pathResult;
    std::string prevPath = "";
    for (int i = 0; i < result.size(); i++) {
        std::string formatStr = "";
        // Iterate through the characters of the original string.
        for (int j = 0; j < result[i].first.length(); j++) {
            // If the current character is not an underscore, add it to the new string.
            if (result[i].first[j] != '_') {
                formatStr += result[i].first[j];
            } else {
                break;
            }
        }
        if (prevPath != formatStr) {
            pathResult.push_back(formatStr);
            edgeStartTime[formatStr] = result[i].second;
            prevPath = formatStr;
        }
    }
    return pathResult;
}

std::vector<std::string> NishidaDStarLite::findPath(TimeSpaceGraph *graph,
        std::string name, double start, Objective *target, double dt, AGV *cur,
        std::string identifier, std::string new_identifier) {

    int i = 0, source = 0, tar = 0;

    for (TimeSpace t : graph->getGraph()->at(0)) {
        if (std::get<0>(t).compare(target->name) == 0) {
            tar = i;
        }
        if (std::get<0>(t).compare(name) == 0) {
            source = i;
        }
        i++;
    }

    int t = (int) (start / dt);

    int j = (int) ((target->tardiness + target->earliness) / (2 * dt));

    DStarLite *dstar = new DStarLite(graph->getGraph(), start, target, dt,
            Node(t, source), Node(j, tar));

    std::vector<std::string> path;

    dstar->main();
    path = dstar->convert();
    allDStarLite.insert(DStarLitePair(new_identifier, dstar));
    if (path.size() > 0) {
        allEdgeStartTime[identifier] = dstar->edgeStartTime;
        dstar->edgeStartTime.clear();
    }
    return path;
}

std::vector<std::string> NishidaDStarLite::reFindPath(TimeSpaceGraph *graph,
        std::string name, double start, Objective *target, double dt, AGV *cur,
        std::string identifier, std::string new_identifier) {

    int i = 0, source = 0, tar = 0;

    for (TimeSpace t : graph->getGraph()->at(0)) {
        if (std::get<0>(t).compare(target->name) == 0) {
            tar = i;
        }
        if (std::get<0>(t).compare(name) == 0) {
            source = i;
        }
        i++;
    }

    int t = (int) (start / dt);

    int j = (int) ((target->tardiness + target->earliness) / (2 * dt));

    std::vector<std::string> path;

    DStarLite *dstar = allDStarLite.at(new_identifier);
    dstar->start = start;
    dstar->dt = dt;
    dstar->target = target;
    dstar->s_start = Node(t, source);
//    dstar->s_goal = Node(j, tar);

    dstar->reMain();
    path = dstar->convert();
    if (path.size() > 0) {
        allEdgeStartTime[identifier] = dstar->edgeStartTime;
        dstar->edgeStartTime.clear();
    }
    return path;
}

NishidaDStarLite::NishidaDStarLite() {
    // TODO Auto-generated constructor stub
    this->initialize();
    getItineraries("itinerary.txt");

    std::string fileName = "AllParts.txt";
    std::vector<TimeSpace> *init;
    init = readAllParts(fileName, 1.56);
    this->graph = new TimeSpaceGraph(1.56, 850, 3.6);
    graph->setGraph(init);
}

void NishidaDStarLite::getItineraries(std::string itineraryFile) {
    std::ifstream file(itineraryFile);
    std::string line;
    std::string nameRoute;
    std::string nameSrc, nameStation, nameDest, nameJunc;
    std::string period, bestTime, amplitude;
    int source, indexOfStation, dst;
    if (this->itineraries.size() > 0) {
        this->itineraries.clear();
    }

    while (getline(file, line)) {
        if (line[0] != '#') {
            std::stringstream ss(line);
            getline(ss, nameRoute, ' ');
            getline(ss, nameSrc, ' ');
            getline(ss, nameStation, ' ');
            getline(ss, nameJunc, ' ');
            getline(ss, nameDest, ' ');
            getline(ss, period, ' ');
            getline(ss, bestTime, ' ');
            getline(ss, amplitude, ' ');
            source = findI_Vertex(nameSrc, false);
            indexOfStation = findI_Vertex(nameStation, false);
            Station *station = new Station(vertices[indexOfStation], bestTime,
                    amplitude, period);

            Itinerary *itinerary = new Itinerary(nameSrc, nameStation,
                    nameDest);

            allSs[vertices[indexOfStation]] = station;
            allItinerary[vertices[indexOfStation]] = itinerary;
            dst = findI_Vertex(nameDest, true);
            itineraries.push_back(
                    std::make_tuple(nameRoute, source, indexOfStation, dst));
        }
    }
    file.close();
}

std::string NishidaDStarLite::getRoute(std::string trace, std::string currLane,
        int currentVertex, int nextVertex, int exitVertex) {

    if (currLane.length() > 0) {
        if (currLane[0] == '^') {
            currLane = currLane.substr(1);
        }
    }

    std::string route = (currLane[0] == ':') ? "" : (currLane + " ");
    std::string temp = "";
    std::string prevTemp = "";
    std::string strCurrVertex = this->vertices[currentVertex];
    std::string strNextVertex = this->vertices[nextVertex];
    for (int i = 0; i < edges.size(); i++) {
        if (edges[i].first.find("$" + currLane + "$") != std::string::npos) {
            std::vector<std::string> list = split(edges[i].first,
                    "$" + currLane + "$");
            std::string remaining = list[list.size() - 1];
            for (int j = 0; j < remaining.length(); j++) {
                if (remaining[j] != '$' && remaining[j] != '_') {
                    temp = temp + remaining[j];
                } else {

                    if (temp[0] != ':'
                            && route.find(temp + " ") == std::string::npos) {
                        route = route + temp + " ";
                    } else if (temp[0] != ':' && temp.compare(prevTemp) != 0
                            && (temp[0] == 'E' || temp[1] == 'E')) {
                        route = route + temp + " ";
                    }
                    prevTemp = temp;
                    temp = "";
                }
            }
            break;
        }
    }

    temp = "";
    prevTemp = "";
    int start = locateLast(route, trace);
    start = (start < 0) ? 1 : start;
    for (int i = start; i < trace.length(); i++) {
        if (trace[i] != '$' && trace[i] != '_') {
            temp = temp + trace[i];
        } else {

            if (temp[0] != ':' && route.find(temp + " ") == std::string::npos) {
                route = route + temp + " ";
            } else if (temp[0] != ':' && temp.compare(prevTemp) != 0
                    && (temp[0] == 'E' || temp[1] == 'E')) {
                route = route + temp + " ";
            }
            prevTemp = temp;
            temp = "";
        }
    }
    if (nextVertex == exitVertex)
        route = route + this->getFinalSegment(trace);
    return route;
}

void NishidaDStarLite::planOut(int source, int target, std::string currLane,
        AGV *cur) {
    bool isUpdate = false;
    bool needUpdate = false;
    AllUpdateNode list;
    if (cur->waitingTimeEdge == -1 && cur->speed == 0 && cur->allowMovement) {
        cur->waitingTimeEdge = cur->travellingTime;
    } else if (cur->waitingTimeEdge != -1 && cur->allowMovement) {
        double waitingTimeEdge = cur->travellingTime - cur->waitingTimeEdge;
        if (cur->speed > 0 && waitingTimeEdge >= 1) {
            isUpdate = true;
            list = this->graph->updateEdge(currLane, waitingTimeEdge);
            cur->waitingTimeEdge = -1;
            cur->liveUpdateWaitingTimeEdge = 1;
        } else if ((int) (waitingTimeEdge / 5)
                == cur->liveUpdateWaitingTimeEdge) {
            isUpdate = true;
            list = this->graph->updateEdge(currLane, waitingTimeEdge);
            cur->liveUpdateWaitingTimeEdge++;
        }
    }
    if (isUpdate) {
        DStarLiteMap::iterator it;
        for (it = allDStarLite.begin(); it != allDStarLite.end(); it++) {
            DStarLite *dstar = it->second;
            for (int i = 0; i < list.size(); i++) {
                dstar->update(list.at(i));
            }
            dstar->computeShortestPath();
        }
        listUpdateAGV.clear();
        updateEdge = currLane;
    }
    Objective obj;
    obj.alpha = obj.gamma = 1;
    obj.beta = 2;
    std::vector<std::string> path;
    std::map<std::string, double> edgeStartTime;
    if (target == cur->itinerary->indexStation) {
        double start = 0;
        std::string startEdge = std::get<0>(
                *allItinerary[vertices[cur->itinerary->indexStation]]);
        obj.name = std::get<1>(
                *allItinerary[vertices[cur->itinerary->indexStation]]);
        obj.earliness = this->allSs[cur->itinerary->station]->bestTime - 30;
        obj.tardiness = this->allSs[cur->itinerary->station]->bestTime + 30;
        std::string identifier = cur->id + startEdge + "$" + obj.name;
        std::string new_identifier = startEdge + "$" + obj.name;
        if (!cur->passedStation) {
            startEdge = currLane;
            start = cur->travellingTime;
        }
        if (allDStarLite.count(new_identifier) <= 0) {
            path = findPath(graph, startEdge, start, &obj, graph->getDt(), cur,
                    identifier, new_identifier);
            paths[identifier] = path;
            paths[new_identifier] = path;
            edgeStartTime = allEdgeStartTime[identifier];
        } else {
            if (paths.count(identifier) <= 0) {
                path = reFindPath(graph, startEdge, start, &obj, graph->getDt(),
                        cur, identifier, new_identifier);
                if (path.size() > 0) {
                    paths[identifier] = path;
                } else {
                    std::cout << "ERROR 1" << endl;
                    path = paths[new_identifier];
                    paths[identifier] = path;
                }
            } else {
                path = paths[identifier];
            }
            if (updateEdge.length() > 0 && currLane.compare(updateEdge) != 0) {
                auto index = std::find(path.begin(), path.end(), currLane) + 1;
                auto indexOfEdge = std::find(index, path.end(), updateEdge);
                if (indexOfEdge != path.end()) {
                    needUpdate = true;
                }
            }
            if (needUpdate) {
                if (listUpdateAGV.count(identifier) <= 0) {
                    listUpdateAGV[identifier] = updateEdge;
                    path = reFindPath(graph, startEdge, start, &obj,
                            graph->getDt(), cur, identifier, new_identifier);
                    if (path.size() > 0) {
                        paths[identifier] = path;
                    } else {
                        std::cout << "ERROR 2" << endl;
                        path = paths[identifier];
                    }
                }
            }
            edgeStartTime = allEdgeStartTime[identifier];
        }
    } else {
        double start = 0;
        std::string startEdge = std::get<1>(
                *allItinerary[vertices[cur->itinerary->indexStation]]);
        obj.name = std::get<2>(
                *allItinerary[vertices[cur->itinerary->indexStation]]);
        obj.earliness = 0;
        obj.tardiness = 0;
        std::string identifier = cur->id + startEdge + "$" + obj.name;
        std::string new_identifier = startEdge + "$" + obj.name;
        if (cur->passedStation) {
            startEdge = currLane;
            start = cur->travellingTime;
        }
        if (allDStarLite.count(new_identifier) <= 0) {
            path = findPath(graph, startEdge, start, &obj, graph->getDt(), cur,
                    identifier, new_identifier);
            paths[identifier] = path;
            paths[new_identifier] = path;
            edgeStartTime = allEdgeStartTime[identifier];
        } else {
            if (paths.count(identifier) <= 0) {
                path = reFindPath(graph, startEdge, start, &obj, graph->getDt(),
                        cur, identifier, new_identifier);
                if (path.size() > 0) {
                    paths[identifier] = path;
                } else {
                    std::cout << "ERROR 1" << endl;
                    path = paths[new_identifier];
                    paths[identifier] = path;
                }
            } else {
                path = paths[identifier];
            }
            if (updateEdge.length() > 0 && currLane.compare(updateEdge) != 0) {
                auto index = std::find(path.begin(), path.end(), currLane) + 1;
                auto indexOfEdge = std::find(index, path.end(), updateEdge);
                if (indexOfEdge != path.end()) {
                    needUpdate = true;
                }
            }
            if (needUpdate) {
                if (listUpdateAGV.count(identifier) <= 0) {
                    listUpdateAGV[identifier] = updateEdge;
                    path = reFindPath(graph, startEdge, start, &obj,
                            graph->getDt(), cur, identifier, new_identifier);
                    if (path.size() > 0) {
                        paths[identifier] = path;
                    } else {
                        std::cout << "ERROR 2" << endl;
                        path = paths[identifier];
                    }
                }
            }
            edgeStartTime = allEdgeStartTime[identifier];
        }
    }

    std::string traces = "$";
    for (std::string str : path) {
        traces += str + "$";
    }

    if (edgeStartTime.find(cur->itinerary->laneId) != edgeStartTime.end()) {
        double startTime = edgeStartTime[cur->itinerary->laneId];
        if (startTime > cur->travellingTime) {
            cur->allowMovement = false;
        } else {
            cur->allowMovement = true;
        }
    }

    cur->init(numVertices);
    cur->ShortestPath[source] = 0;
    cur->traces[target] = traces;
}
