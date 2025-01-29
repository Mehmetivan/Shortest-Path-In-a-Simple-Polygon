#include <SFML/Graphics.hpp>
#include <vector>
#include <array>
#include <iostream>
#include <numeric>
#include <unordered_map>
#include "include/json.hpp"
#include <fstream> 
#include <queue>
#include <limits>
#include <sstream>

using json = nlohmann::json;  // json is needed to be able to read the edge weight from the file

// Function in order to check whether a point(selected) is inside the triangle
bool pointInTriangle(const sf::Vector2f& p, const sf::Vector2f& a, const sf::Vector2f& b, const sf::Vector2f& c) {
    auto sign = [](const sf::Vector2f& p1, const sf::Vector2f& p2, const sf::Vector2f& p3) {
        return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
        };

    float d1 = sign(p, a, b);
    float d2 = sign(p, b, c);
    float d3 = sign(p, c, a);

    bool has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
    bool has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);

    const float epsilon = 1e-6f;
    if (fabs(d1) < epsilon || fabs(d2) < epsilon || fabs(d3) < epsilon) return true;

    return !(has_neg && has_pos);
}

// Check if the angle formed by the three vertices is convex
bool isConvex(const sf::Vector2f& prev, const sf::Vector2f& curr, const sf::Vector2f& next) {
    float crossProduct = (curr.x - prev.x) * (next.y - curr.y) - (curr.y - prev.y) * (next.x - curr.x);
    return crossProduct > 0;
}

// This is where EAR-CUT triangulation happens by the help of adjacency list provided by the file.
std::vector<std::array<sf::Vector2f, 3>> earClippingTriangulation(const std::vector<sf::Vector2f>& vertices,
    std::unordered_map<int, std::vector<int>>& adjacencyList) {
    std::vector<std::array<sf::Vector2f, 3>> triangles;

    if (vertices.size() < 3) return triangles;

    std::vector<int> indices(vertices.size());
    std::iota(indices.begin(), indices.end(), 0);

    while (indices.size() > 3) {
        bool earFound = false;

        for (size_t i = 0; i < indices.size(); ++i) {
            int prev = indices[(i + indices.size() - 1) % indices.size()];
            int curr = indices[i];
            int next = indices[(i + 1) % indices.size()];

            const sf::Vector2f& pPrev = vertices[prev];
            const sf::Vector2f& pCurr = vertices[curr];
            const sf::Vector2f& pNext = vertices[next];

            if (!isConvex(pPrev, pCurr, pNext)) continue;

            bool isEar = true;
            for (int idx : indices) {
                if (idx != prev && idx != curr && idx != next && pointInTriangle(vertices[idx], pPrev, pCurr, pNext)) {
                    isEar = false;
                    break;
                }
            }

            if (isEar) {
                triangles.push_back({ pPrev, pCurr, pNext });

                //We are printing the triangle to see
                std::cout << "Triangle formed: "
                    << "(" << pPrev.x << ", " << pPrev.y << "), "
                    << "(" << pCurr.x << ", " << pCurr.y << "), "
                    << "(" << pNext.x << ", " << pNext.y << ")\n";

                // This is the ear removal stage of the ear-cut algorithm where we cut the edge of the used ear
                adjacencyList[prev].erase(std::remove(adjacencyList[prev].begin(), adjacencyList[prev].end(), curr), adjacencyList[prev].end());
                adjacencyList[curr].erase(std::remove(adjacencyList[curr].begin(), adjacencyList[curr].end(), next), adjacencyList[curr].end());
                adjacencyList[next].erase(std::remove(adjacencyList[next].begin(), adjacencyList[next].end(), prev), adjacencyList[next].end());

                // Add the new edges
                adjacencyList[prev].push_back(next);
                adjacencyList[next].push_back(prev);
                adjacencyList[curr].push_back(prev);
                adjacencyList[curr].push_back(next);

                
                indices.erase(indices.begin() + i);
                earFound = true;
                break;
            }
        }

        if (!earFound) break;
    }

    if (indices.size() == 3) {
        triangles.push_back({ vertices[indices[0]], vertices[indices[1]], vertices[indices[2]] });
    }

    return triangles;
}

// We are using this function in order to find the triangle where our selected points are located at.
int findTriangleContainingPoint(const sf::Vector2f& click, const std::vector<std::array<sf::Vector2f, 3>>& triangles) {
    for (size_t i = 0; i < triangles.size(); ++i) {
        const auto& triangle = triangles[i];
        if (pointInTriangle(click, triangle[0], triangle[1], triangle[2])) {
            std::cout << "Triangle " << i << " contains the point: "
                << "A(" << triangle[0].x << ", " << triangle[0].y << "), "
                << "B(" << triangle[1].x << ", " << triangle[1].y << "), "
                << "C(" << triangle[2].x << ", " << triangle[2].y << ")\n";
            return i;
        }
    }
    return -1;
}

// We are loading our edge-weigt infos from the json file
std::unordered_map<int, std::unordered_map<int, double>> loadJsonData(const std::string& fileName) {
    std::unordered_map<int, std::unordered_map<int, double>> data;

    
    std::ifstream file(fileName);
    if (file.is_open()) {
        json j;
        file >> j;  

        // Iterate through JSON objects
        for (auto& item : j.items()) {
            int key = std::stoi(item.key()); 
            for (auto& subItem : item.value().items()) {
                int subKey = std::stoi(subItem.key());
                double value = subItem.value();
                data[key][subKey] = value;  // We are storing it in a unordered_map
            }
        }
    }
    else {
        std::cerr << "Failed to open JSON file." << std::endl;
    }

    return data;
}


// Dijkstra's algorithm was preferred in this project ( A* was the other option) due to simplicity.
std::vector<int> dijkstra(int start, int end, const std::unordered_map<int, std::unordered_map<int, double>>& adjList) {
    std::unordered_map<int, double> dist;
    std::unordered_map<int, int> prev;
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<>> pq;

    // Initialize distances to infinity to begin with
    for (const auto& pair : adjList) {
        dist[pair.first] = std::numeric_limits<double>::infinity();
    }
    dist[start] = 0.0;
    pq.push({ 0.0, start });

    while (!pq.empty()) {
        int current = pq.top().second;
        double currentDist = pq.top().first;
        pq.pop();

        // we are stopping when we reach the end
        if (current == end) break;

        if (currentDist > dist[current]) continue;

        // Iterate through neighbors
        for (const auto& neighbor : adjList.at(current)) {
            int neighborTriangle = neighbor.first;
            double weight = neighbor.second;

            double newDist = currentDist + weight;
            if (newDist < dist[neighborTriangle]) {
                dist[neighborTriangle] = newDist;
                prev[neighborTriangle] = current;
                pq.push({ newDist, neighborTriangle });
            }
        }
    }

    // Reconstruct the path
    std::vector<int> path;
    for (int at = end; at != start; at = prev[at]) {
        if (prev.find(at) == prev.end()) {
            std::cerr << "No path found!" << std::endl;
            return {};
        }
        path.push_back(at);
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());
    return path;
}


int main() {
    // Load the infos about edgeweights that are stored in the json 
    auto jsonData = loadJsonData("triangle_edge_weights.json");

    
    for (const auto& item : jsonData) {
        for (const auto& subItem : item.second) {
            std::cout << "Key: " << item.first << ", SubKey: " << subItem.first << ", Value: " << subItem.second << std::endl;
        }
    }

    
    sf::RenderWindow polygonWindow(sf::VideoMode(1200, 600), "Original Polygon");
    sf::RenderWindow triangulationWindow(sf::VideoMode(1200, 600), "Triangulated Polygon");

    std::vector<sf::Vector2f> vertices = {
        {151, 149}, {201, 77}, {318, 158}, {378, 246}, {386, 95},
        {447, 39}, {540, 33}, {623, 140}, {648, 29}, {770, 45},
        {800, 90}, {875, 104}, {930, 35}, {1040, 102}, {1082, 8},
        {1167, 134}, {1122, 280}, {1028, 285}, {971, 181}, {874, 271},
        {800, 235}, {742, 147}, {696, 251}, {608, 323}, {551, 297},
        {528, 202}, {411, 336}, {313, 319}, {227, 244}, {202, 340},
        {131, 395}, {63, 337}, {36, 205}, {91, 72}
    };

    
    std::unordered_map<int, std::vector<int>> adjacencyList;

    
    auto triangles = earClippingTriangulation(vertices, adjacencyList);

    // selected points
    sf::Vector2f currentDestination, endDestination;
    int selectedStartTriangle = -1, selectedEndTriangle = -1;

    // to check whether shortest path has been printed to prevent infinite loop
    bool pathPrinted = false;
    std::vector<int> shortestPath;

    while (polygonWindow.isOpen() && triangulationWindow.isOpen()) {
        sf::Event event;

        // here we handle when users click on the windows to select the points
        while (polygonWindow.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                polygonWindow.close();
            if (event.type == sf::Event::MouseButtonPressed) {
                sf::Vector2i mousePos = sf::Mouse::getPosition(polygonWindow);
                sf::Vector2f clickPos(static_cast<float>(mousePos.x), static_cast<float>(mousePos.y));
                int triangleIndex = findTriangleContainingPoint(clickPos, triangles);

                if (triangleIndex != -1) {
                    if (currentDestination == sf::Vector2f(0, 0)) {
                        currentDestination = clickPos;
                        selectedStartTriangle = triangleIndex;
                        std::cout << "Current destination selected: (" << currentDestination.x << ", " << currentDestination.y << ")\n";
                        pathPrinted = false; 
                    }
                    else {
                        endDestination = clickPos;
                        selectedEndTriangle = triangleIndex;
                        std::cout << "End destination selected: (" << endDestination.x << ", " << endDestination.y << ")\n";
                        pathPrinted = false; // Reset the flag for next use
                    }
                }
            }
        }

        // after selecting the points we are running our shortest path algorithm
        if (selectedStartTriangle != -1 && selectedEndTriangle != -1 && !pathPrinted) {
            shortestPath = dijkstra(selectedStartTriangle, selectedEndTriangle, jsonData);

            // Output the shortest path. This outputs to the terminal in terms of triangles as we treat each triangle as the node
            std::cout << "Shortest path from triangle " << selectedStartTriangle << " to triangle " << selectedEndTriangle << ": ";
            for (int triangle : shortestPath) {
                std::cout << triangle << " ";
            }
            std::cout << std::endl;

            pathPrinted = true; 
        }

       
        polygonWindow.clear(sf::Color::White);
        sf::VertexArray polygon(sf::LineStrip, vertices.size() + 1);
        for (size_t i = 0; i < vertices.size(); ++i) {
            polygon[i].position = vertices[i];
            polygon[i].color = sf::Color::Blue;
        }
        polygon[vertices.size()].position = vertices[0];
        polygon[vertices.size()].color = sf::Color::Blue;

        polygonWindow.draw(polygon);
        polygonWindow.display();

        
        triangulationWindow.clear(sf::Color::White);
        for (size_t i = 0; i < triangles.size(); ++i) {
            sf::VertexArray tri(sf::LineStrip, 4);
            tri[0].position = triangles[i][0];
            tri[1].position = triangles[i][1];
            tri[2].position = triangles[i][2];
            tri[3].position = triangles[i][0];
            tri[0].color = sf::Color::Black;
            tri[1].color = sf::Color::Black;
            tri[2].color = sf::Color::Black;
            tri[3].color = sf::Color::Black;

            // here we are highlighting the selected triangles. Rendering issues may happen here!!!!!!!
            if (i == selectedStartTriangle || i == selectedEndTriangle) {
                tri[0].color = sf::Color::Blue;
                tri[1].color = sf::Color::Blue;
                tri[2].color = sf::Color::Blue;
            }

            triangulationWindow.draw(tri);
        }

        // Here we are highlighting the shortest path
        if (pathPrinted && !shortestPath.empty()) {
            
            if (shortestPath.size() > 1 && currentDestination != sf::Vector2f(0, 0)) {
                int from = shortestPath[0]; 
                int to = shortestPath[1];

                sf::VertexArray startToFirstEdge(sf::Lines, 2);
                startToFirstEdge[0].position = currentDestination; 
                startToFirstEdge[1].position = sf::Vector2f(triangles[to][0]); 
                startToFirstEdge[0].color = sf::Color::Red; 
                startToFirstEdge[1].color = sf::Color::Red;

                triangulationWindow.draw(startToFirstEdge);
            }









            for (size_t i = 1; i < shortestPath.size(); ++i) {
                int from = shortestPath[i - 1];
                int to = shortestPath[i];

                sf::VertexArray edge(sf::Lines, 2);
                edge[0].position = sf::Vector2f(triangles[from][0]);
                edge[1].position = sf::Vector2f(triangles[to][0]);
                edge[0].color = sf::Color::Red;
                edge[1].color = sf::Color::Red;

                triangulationWindow.draw(edge);
            }

            // As a finishing touch, we draw the line from the point to edge of the triangle. May have issues here!!!
            if (!shortestPath.empty() && endDestination != sf::Vector2f(0, 0)) {
                int lastTriangle = shortestPath.back();

                // Pick a vertex of the last triangle as the connection point.Here we have issues because we dont know which edge is best to connect so we just connect to first edge.
                sf::Vector2f connectionPoint = triangles[lastTriangle][0]; 

                sf::VertexArray extraLine(sf::Lines, 2);
                extraLine[0].position = connectionPoint;
                extraLine[1].position = endDestination;
                extraLine[0].color = sf::Color::Red; 
                extraLine[1].color = sf::Color::Red;

                triangulationWindow.draw(extraLine);
            }






        }
        // Draw the selected points on the triangulated polygon
        if (currentDestination != sf::Vector2f(0, 0)) {
            sf::CircleShape startPoint(5);
            startPoint.setFillColor(sf::Color::Red);
            startPoint.setPosition(currentDestination - sf::Vector2f(5, 5)); 
            triangulationWindow.draw(startPoint);
        }

        if (endDestination != sf::Vector2f(0, 0)) {
            sf::CircleShape endPoint(5);
            endPoint.setFillColor(sf::Color::Blue);
            endPoint.setPosition(endDestination - sf::Vector2f(5, 5)); 
            triangulationWindow.draw(endPoint);
        }

        triangulationWindow.display();
    }

    return 0;
}
