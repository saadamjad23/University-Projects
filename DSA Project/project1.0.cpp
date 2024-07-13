#include <iostream.h>
#include <vector.h>
#include <unordered_map.h>
#include <set.h>
#include <limits.h>
#include <queue.h>

class Edge {
public:
    int destination;
    double weight;

    Edge(int dest, double w) : destination(dest), weight(w) {}
};

class Node {
public:
    int id;
    std::vector<Edge> edges;

    Node(int id) : id(id) {}
};

class Graph {
private:
    std::unordered_map<int, Node> nodes;

public:
    void addNode(int id) {
        nodes[id] = Node(id);
    }

    void addEdge(int src, int dest, double weight) {
        nodes[src].edges.push_back(Edge(dest, weight));
    }

    std::vector<Edge> getEdges(int id) {
        return nodes[id].edges;
    }

    bool hasNode(int id) {
        return nodes.find(id) != nodes.end();
    }
};

std::unordered_map<int, double> dijkstra(Graph &graph, int start) {
    std::unordered_map<int, double> distances;
    for (auto &pair : graph) {
        distances[pair.first] = std::numeric_limits<double>::infinity();
    }
    distances[start] = 0;

    using NodeDistPair = std::pair<double, int>;
    std::priority_queue<NodeDistPair, std::vector<NodeDistPair>, std::greater<NodeDistPair>> pq;
    pq.push({0, start});

    while (!pq.empty()) {
        double currentDistance = pq.top().first;
        int currentNode = pq.top().second;
        pq.pop();

        for (Edge &edge : graph.getEdges(currentNode)) {
            double newDist = currentDistance + edge.weight;
            if (newDist < distances[edge.destination]) {
                distances[edge.destination] = newDist;
                pq.push({newDist, edge.destination});
            }
        }
    }

    return distances;
}

#include <cmath>

double heuristic(int node, int target) {
    // Implement your heuristic function here
    return std::abs(node - target); // Placeholder heuristic
}

std::vector<int> aStar(Graph &graph, int start, int goal) {
    std::unordered_map<int, double> gScore;
    std::unordered_map<int, double> fScore;
    std::unordered_map<int, int> cameFrom;
    for (auto &pair : graph) {
        gScore[pair.first] = std::numeric_limits<double>::infinity();
        fScore[pair.first] = std::numeric_limits<double>::infinity();
    }
    gScore[start] = 0;
    fScore[start] = heuristic(start, goal);

    using NodeDistPair = std::pair<double, int>;
    std::priority_queue<NodeDistPair, std::vector<NodeDistPair>, std::greater<NodeDistPair>> openSet;
    openSet.push({fScore[start], start});

    while (!openSet.empty()) {
        int current = openSet.top().second;
        openSet.pop();

        if (current == goal) {
            std::vector<int> path;
            while (cameFrom.find(current) != cameFrom.end()) {
                path.push_back(current);
                current = cameFrom[current];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (Edge &edge : graph.getEdges(current)) {
            double tentative_gScore = gScore[current] + edge.weight;
            if (tentative_gScore < gScore[edge.destination]) {
                cameFrom[edge.destination] = current;
                gScore[edge.destination] = tentative_gScore;
                fScore[edge.destination] = gScore[edge.destination] + heuristic(edge.destination, goal);
                openSet.push({fScore[edge.destination], edge.destination});
            }
        }
    }

    return std::vector<int>(); // Return an empty path if no path is found
}

int main() {
    Graph graph;
    // Add nodes and edges to the graph
    graph.addNode(1);
    graph.addNode(2);
    graph.addNode(3);
    graph.addEdge(1, 2, 1.0);
    graph.addEdge(2, 3, 2.0);
    graph.addEdge(1, 3, 2.5);

    int start = 1;
    int goal = 3;

    std::vector<int> path = aStar(graph, start, goal);
    if (!path.empty()) {
        std::cout << "Path found: ";
        for (int node : path) {
            std::cout << node << " ";
        }
        std::cout << std::endl;
    } else {
        std::cout << "No path found." << std::endl;
    }

    return 0;
}
