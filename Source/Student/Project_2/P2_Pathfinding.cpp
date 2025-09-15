#include <pch.h>
#include "Projects/ProjectTwo.h"
#include "P2_Pathfinding.h"

#pragma region Extra Credit 
bool ProjectTwo::implemented_floyd_warshall()
{
    return false;
}

bool ProjectTwo::implemented_goal_bounding()
{
    return false;
}
#pragma endregion

bool AStarPather::initialize()
{
    for (int i = 0; i < MAX_MAP_SIZE * MAX_MAP_SIZE; ++i) {
        int row = i / MAX_MAP_SIZE;
        int col = i % MAX_MAP_SIZE;
        nodePool[i].gridPos = { row, col };
        nodePool[i].bucketIndex = 0;
        nodePool[i].nextInBucket = nullptr;
    }

    return true;
}

void AStarPather::shutdown()
{
    /*
        Free any dynamically allocated memory or any other general house-
        keeping you need to do during shutdown.
    */
}

PathResult AStarPather::compute_path(PathRequest& request)
{
    GridPos startPos = terrain->get_grid_position(request.start);
    GridPos goalPos = terrain->get_grid_position(request.goal);

    Node* startNode = &nodePool[getIndex(startPos.row, startPos.col)];
    Node* goalNode = &nodePool[getIndex(goalPos.row, goalPos.col)];

    float weight = request.settings.weight;

    if (request.newRequest) {
        resetNodes();
        startNode->givenCost = 0.f;
        startNode->finalCost = weight * heuristic(startPos, goalPos, request.settings.heuristic);
        startNode->parent = nullptr;

        pushToOpenList(startNode);
    }

    while (!openList.empty()) {

        Node* parentNode = popCheapestNode();

        if (parentNode == goalNode) {
            Node* current = goalNode;
            while (current != nullptr) {
                request.path.push_front(terrain->get_world_position(current->gridPos));
                current = current->parent;
            }

            if (request.settings.rubberBanding || request.settings.smoothing) {
                std::vector<GridPos> gridPath;

                for (const auto& pos : request.path) {
                    gridPath.push_back(terrain->get_grid_position(pos));
                }

                if (request.settings.rubberBanding) {
                    RubberbandPath(gridPath);
                }

                if (request.settings.rubberBanding && request.settings.smoothing) {
                    FillPath(gridPath);
                }


                request.path.clear();
                for (const auto& pos : gridPath) {
                    request.path.push_back(terrain->get_world_position(pos));
                }

                if (request.settings.smoothing) {
                    SmoothPathCatmullRom(request.path);
                }
            }

            return PathResult::COMPLETE;
        }

        parentNode->list = LIST_CLOSED;

        if (request.settings.debugColoring) {
            terrain->set_color(parentNode->gridPos, Colors::Yellow);
        }

        Node* neighbors[8];
        int neighborCount = getNeighbors(parentNode, neighbors);

        for (int i = 0; i < neighborCount; ++i) {
            Node* child = neighbors[i];

            float newGivenCost = parentNode->givenCost + getCost(parentNode, child);
            float newFinalCost = newGivenCost + weight * heuristic(child->gridPos, goalPos, request.settings.heuristic);

            if (child->list == LIST_NONE) {
                child->parent = parentNode;
                child->givenCost = newGivenCost;
                child->finalCost = newFinalCost;
                pushToOpenList(child);

                if (request.settings.debugColoring) {
                    terrain->set_color(child->gridPos, Colors::Blue);
                }
            }
            else if (newGivenCost < child->givenCost) {
                child->parent = parentNode;
                child->givenCost = newGivenCost;

                if (child->list == LIST_CLOSED) {
                    child->list = LIST_NONE;
                    child->finalCost = newFinalCost;
                    pushToOpenList(child);
                }
                else if (child->list == LIST_OPEN) {
                    // Node is already in open list, update its position
                    openList.updateNode(child, newFinalCost);
                }

                if (request.settings.debugColoring) {
                    terrain->set_color(child->gridPos, Colors::Blue);
                }
            }
        }

        if (request.settings.singleStep) {
            return PathResult::PROCESSING;
        }
    }

    return PathResult::IMPOSSIBLE;
}

void AStarPather::resetNodes() {
    // Reset all nodes
    for (int i = 0; i < MAX_MAP_SIZE * MAX_MAP_SIZE; ++i) {
        nodePool[i].parent = nullptr;
        nodePool[i].givenCost = FLT_MAX;
        nodePool[i].finalCost = FLT_MAX;
        nodePool[i].list = LIST_NONE;
        nodePool[i].bucketIndex = 0;
        nodePool[i].nextInBucket = nullptr;
    }

    // Clear open list
    openList.clear();
}

void AStarPather::pushToOpenList(Node* node) {
    openList.push(node);
}

AStarPather::Node* AStarPather::popCheapestNode() {
    return openList.pop();
}

int AStarPather::getNeighbors(Node* node, Node** outNeighbors)
{
    int count = 0;
    int row = node->gridPos.row;
    int col = node->gridPos.col;

    auto tryAdd = [&](int dRow, int dCol, bool diagonal) {
        int newRow = row + dRow;
        int newCol = col + dCol;

        if (!terrain->is_valid_grid_position({ newRow, newCol }) || terrain->is_wall({ newRow, newCol }))   return;

        if (diagonal) {
            if (terrain->is_wall({ row + dRow, col }) || terrain->is_wall({ row, col + dCol }))     return;
        }

        outNeighbors[count++] = &nodePool[(newRow * MAX_MAP_SIZE + newCol)];
        };

    // Cardinal directions
    tryAdd(-1, 0, false); // N
    tryAdd(1, 0, false); // S
    tryAdd(0, -1, false); // W
    tryAdd(0, 1, false); // E

    // Diagonal directions
    tryAdd(-1, -1, true);  // NW
    tryAdd(-1, 1, true);  // NE
    tryAdd(1, -1, true);  // SW
    tryAdd(1, 1, true);  // SE

    return count;
}

bool AStarPather::IsAreaClear(const GridPos& a, const GridPos& b)
{
    int minRow = std::min(a.row, b.row);
    int maxRow = std::max(a.row, b.row);
    int minCol = std::min(a.col, b.col);
    int maxCol = std::max(a.col, b.col);

    for (int row = minRow; row <= maxRow; ++row)
    {
        for (int col = minCol; col <= maxCol; ++col)
        {
            if (terrain->is_wall(row, col))
                return false;
        }
    }

    return true;
}

void AStarPather::RubberbandPath(std::vector<GridPos>& gridPath)
{
    if (gridPath.size() < 3) return;

    int i = static_cast<int>(gridPath.size()) - 1;

    while (i - 2 >= 0)
    {
        const GridPos& first = gridPath[i - 2];
        const GridPos& middle = gridPath[i - 1];
        const GridPos& last = gridPath[i];

        if (IsAreaClear(first, last))
        {
            gridPath.erase(gridPath.begin() + i - 1);
            --i;
        }
        else
        {
            --i;
        }
    }
}

void AStarPather::SmoothPathCatmullRom(std::list<Vec3>& path)
{
    if (path.size() < 4) { return; }

    std::list<Vec3> smoothedPath;
    std::vector<Vec3> points(path.begin(), path.end());

    // Add the first point
    smoothedPath.push_back(points[0]);

    for (size_t i = 1; i + 2 < points.size(); ++i)
    {
        const Vec3& P0 = points[i - 1];
        const Vec3& P1 = points[i];
        const Vec3& P2 = points[i + 1];
        const Vec3& P3 = points[i + 2];

        // Add the control point P1
        smoothedPath.push_back(P1);

        // Add interpolated points with fixed parameter values
        smoothedPath.push_back(Vec3::CatmullRom(P0, P1, P2, P3, 0.25f));
        smoothedPath.push_back(Vec3::CatmullRom(P0, P1, P2, P3, 0.5f));
        smoothedPath.push_back(Vec3::CatmullRom(P0, P1, P2, P3, 0.75f));
    }

    // Add the final point
    smoothedPath.push_back(points.back());

    path = std::move(smoothedPath);
}

void AStarPather::FillPath(std::vector<GridPos>& gridPath) {
    std::vector<GridPos> filledPath;
    for (size_t j = 0; j + 1 < gridPath.size(); ++j) {
        filledPath.push_back(gridPath[j]);

        Vec3 posA = terrain->get_world_position(gridPath[j]);
        Vec3 posB = terrain->get_world_position(gridPath[j + 1]);
        float dist = (posB - posA).Length();

        if (dist > 1.5f ) {
            Vec3 mid = (posA + posB) * 0.5f;
            filledPath.push_back(terrain->get_grid_position(mid));
        }
    }
    filledPath.push_back(gridPath.back());
    gridPath = std::move(filledPath);
}