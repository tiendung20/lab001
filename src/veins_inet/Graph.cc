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

#include "Graph.h"

void assignNeighbors(std::vector<TimeSpace> *all, int index) {
    int index_neighbor = 0;
    for (const TimeSpace &t : *all) {
        if (std::get<1>(std::get<2>(all->at(index)))
                == std::get<0>(std::get<2>(t))) {
            (std::get<3>(all->at(index))).push_back( { index_neighbor, 0 });
        }
        index_neighbor++;
    }
}

std::vector<TimeSpace>* readAllParts(std::string fileName, double lengthPath) {
    std::vector<TimeSpace> *init = new std::vector<TimeSpace>;
    std::map<std::pair<std::string, int>, bool> list;
    std::ifstream input(fileName);
    for (std::string line; getline(input, line);) {
        TimeSpace t;
        std::string name;
        int id;
        Point p1, p2;
        double l = lengthPath + 1, x1, y1, x2, y2;
        int i, count, pred, c;
        count = pred = c = 0;
        for (i = 0; i < line.length(); i++) {
            if (count == 0 && line[i] == ' ') {
                name = line.substr(0, i);
                if (c == 0) {
                    c++;
                }
            } else if (count == 1 && line[i] == '_') {
                id = std::stoi(line.substr(pred, i - pred));
                if (c == 1) {
                    c++;
                }
            } else if (count == 2 && line[i] == ',') {
                x1 = std::stod(line.substr(pred, i - pred));
                if (c == 2) {
                    c++;
                }
            } else if (count == 3 && line[i] == '_') {
                y1 = std::stod(line.substr(pred, i - pred));
                if (c == 3) {
                    c++;
                }
            } else if (count == 4 && line[i] == ',') {
                x2 = std::stod(line.substr(pred, i - pred));
                if (c == 4) {
                    c++;
                }
            }
            if (line[i] == ' ' || line[i] == ',' || line[i] == '_') {
                if (count == 0 && line[i] == '_') {
                    continue;
                }
                pred = i + 1;
                count++;
                if (count > 4) {
                    break;
                }
            }
        }
        if (count == 5 && c == 5) {
            y2 = std::stod(line.substr(pred, i - pred));
            l = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
        }
        if (l > (lengthPath + 0.005 * lengthPath) || list[ { name, id }]) {
            input.close();
            list.clear();
            delete init;
            return NULL;
        }
        list[ { name, id }] = true;
        std::get<0>(t) = name;
        std::get<1>(t) = id;
        p1 = Point(x1, y1);
        p2 = Point(x2, y2);
        std::get<2>(t) = Shape(p1, p2, l);
        std::get<3>(t) = std::get<4>(t) = { };
        std::get<5>(t) = 0.0;
        init->push_back(t);
    }
    input.close();
    list.clear();
    int index = 0;
    for (auto it = init->begin(); it != init->end(); it++, index++) {
        assignNeighbors(init, index);
    }
    return init;
}

bool checkValid(std::vector<TimeSpace> *all, double lengthPath,
        double epsilon) {
    double L;
    for (const TimeSpace &t : *all) {
        L = std::get<2>(std::get<2>(t));
        if (fabs(L - lengthPath) / lengthPath > epsilon) {
            return false;
        }
    }
    return true;
}

TimeSpaceGraph::TimeSpaceGraph(double lengthPath, double H, double V) {
    this->lengthPath = lengthPath;
    this->H = H;
    this->V = V;
    this->dt = lengthPath / V;
}

void TimeSpaceGraph::setGraph(std::vector<TimeSpace> *init) {
    graph = new std::vector<std::vector<TimeSpace>>;
    int N = ceil(H / dt);
    for (int i = 0; i <= N; i++) {
        std::vector<TimeSpace> temp;
        std::copy(init->cbegin(), init->cend(), std::back_inserter(temp));
        for (int j = 0; j < temp.size(); j++) {
            std::vector<Index> *indices = &(std::get<3>(temp.at(j)));
            int S = indices->size() - 1;
            double x = std::get<2>(std::get<2>(temp.at(j)));
            int min = round(x / lengthPath);

            for (int k = S; k >= 0; k--) {
                if (i + min <= N) {
                    indices->at(k) = std::make_pair(std::get<0>(indices->at(k)),
                            i + min);
                } else {
                    indices->erase(indices->begin() + k);
                }
            }

            if ((i + 1) <= N) {
                indices->push_back(std::make_pair(j, i + 1));
            }
            std::get<5>(temp.at(j)) = i;
        }
        graph->push_back(temp);
    }
    int i = 0, j;
    for (std::vector<TimeSpace> &v : *graph) {
        j = 0;
        for (TimeSpace &t : v) {
            std::vector<Index> *indices = &(std::get<3>(t));
            for (int k = 0; k < indices->size(); k++) {
                std::get<4>(
                        graph->at(std::get<1>(indices->at(k)))[std::get<0>(
                                indices->at(k))]).push_back( { j, i });
            }
            j++;
        }
        i++;
    }
}

std::vector<std::vector<TimeSpace>>* TimeSpaceGraph::getGraph() {
    return graph;
}

double TimeSpaceGraph::getDt() {
    return dt;
}

AllUpdateNode TimeSpaceGraph::updateEdge(std::string name, double waitingTime) {
    int length = ceil(waitingTime / dt);
//    std::cout << waitingTime << " and " << length << std::endl;
    int N = ceil(H / dt);
    int t = -1;
    AllUpdateNode list;
    for (int i = 0; i <= N; i++) {
        if (t == -1) {
            for (int j = 0; j < graph->at(i).size(); j++) {
                if (std::get<0>(graph->at(i)[j]).compare(name) == 0) {
                    t = j;
                    break;
                }
            }
        }
        if (t != -1) {
            std::vector<Index> *indices = &(std::get<3>(graph->at(i)[t]));
            int S = indices->size() - 1;
            for (int k = S; k >= 0; k--) {
                int e = std::get<0>(indices->at(k));
                int v = std::get<1>(indices->at(k));
                if (e != t) {
                    // Erase prev of succ Edge
//                    std::vector<Index> *indices_prev = &(std::get<4>(
//                            graph->at(v)[e]));
//                    int indexOfEdge = -1;
//                    for (int id = 0; id < indices_prev->size(); id++) {
//                        if (indices_prev->at(id).first == t)
//                            indexOfEdge = id;
//                    }
//                    if (indexOfEdge != -1) {
//                        indices_prev->erase(
//                                indices_prev->begin() + indexOfEdge);
//                    }

                    if (i + length <= N) {
                        std::get<1>(indices->at(k)) = i + length;

//                        indices_prev = &(std::get<4>(graph->at(i + length)[e]));
//                        indexOfEdge = -1;
//                        for (int id = 0; id < indices_prev->size(); id++) {
//                            if (indices_prev->at(id).first == t)
//                                indexOfEdge = id;
//                        }
//                        if (indexOfEdge != -1) {
//                            std::get<1>(indices_prev->at(indexOfEdge)) = i;
//                        } else {
//                            indices_prev->push_back( { t, i });
//                        }

                    } else {
                        indices->erase(indices->begin() + k);
                    }
                }
            }
            list.push_back(Node(i, t));
        }
    }
    int i = 0, j;
    for (std::vector<TimeSpace> &v : *graph) {
        for (TimeSpace &ts : v) {
            std::vector<Index> *indices = &(std::get<4>(ts));
            indices->clear();
        }
    }
    for (std::vector<TimeSpace> &v : *graph) {
        j = 0;
        for (TimeSpace &ts : v) {
            std::vector<Index> *indices = &(std::get<3>(ts));
            for (int k = 0; k < indices->size(); k++) {
                if (N > std::get<1>(indices->at(k))) {
                    std::get<4>(
                            graph->at(std::get<1>(indices->at(k)))[std::get<0>(
                                    indices->at(k))]).push_back( { j, i });
                }
            }
            j++;
        }
        i++;
    }
    return list;
}
