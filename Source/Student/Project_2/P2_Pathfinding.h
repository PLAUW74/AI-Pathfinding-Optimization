#pragma once
#include "Misc/PathfindingDetails.hpp"
#include <vector>
#include <array>

class AStarPather
{
public:
    bool initialize();
    void shutdown();
    PathResult compute_path(PathRequest& request);

private:
    static constexpr uint8_t LIST_NONE = 0;
    static constexpr uint8_t LIST_OPEN = 1;
    static constexpr uint8_t LIST_CLOSED = 2;

    struct Node {
        Node* parent;   // 8 bytes
        float finalCost; // 4 bytes
        float givenCost;    // 4 bytes
        GridPos gridPos;    // 8 bytes (2 bytes row + 2 bytes col)
        uint8_t list : 2;   // 2 bits for list type
        uint16_t bucketIndex : 14;  // 14 bits for bucket index
        Node* nextInBucket;  // 8 bytes - linked list pointer for bucket
    };


    // Bucket Open List Implementation
    static constexpr float BUCKET_SIZE = 0.12f;  // 0.1 seems to be averagely good.
    static constexpr int MAX_BUCKETS = 1024;

    struct BucketOpenList {
        std::array<Node*, MAX_BUCKETS> buckets;  // Array of bucket heads
        int lowestBucketIndex;                   // track lowest non-empty bucket
        int nodeCount;                           // Total nodes in open list

        BucketOpenList() : lowestBucketIndex(MAX_BUCKETS), nodeCount(0) {
            buckets.fill(nullptr);
        }

        void clear() {
            buckets.fill(nullptr);
            lowestBucketIndex = MAX_BUCKETS;
            nodeCount = 0;
        }

        bool empty() const {
            return nodeCount == 0;
        }

        void push(Node* node) {
            // Calculate bucket index based on final cost
            int bucketIdx = static_cast<int>(node->finalCost / BUCKET_SIZE);
            bucketIdx = std::min(bucketIdx, MAX_BUCKETS - 1);  // Clamp to valid range

            // Add node to front of bucket's linked list
            node->nextInBucket = buckets[bucketIdx];
            buckets[bucketIdx] = node;
            node->bucketIndex = bucketIdx;
            node->list = LIST_OPEN;

            // Update lowest bucket pointer if this bucket is lower
            if (bucketIdx < lowestBucketIndex) {
                lowestBucketIndex = bucketIdx;
            }

            nodeCount++;
        }

        Node* pop() {
            if (nodeCount == 0) return nullptr;

            // Find the lowest non-empty bucket starting from current pointer
            while (lowestBucketIndex < MAX_BUCKETS && buckets[lowestBucketIndex] == nullptr) {
                lowestBucketIndex++;
            }

            if (lowestBucketIndex >= MAX_BUCKETS) {
                // This shouldn't happen if nodeCount > 0, but safety check
                nodeCount = 0;
                return nullptr;
            }

            // Get the bucket
            Node* bucket = buckets[lowestBucketIndex];

            // Find the node with minimum f-cost in this bucket
            Node* cheapest = bucket;
            Node* cheapestPrev = nullptr;
            Node* current = bucket;
            Node* prev = nullptr;

            while (current != nullptr) {
                if (current->finalCost < cheapest->finalCost ||
                    (current->finalCost == cheapest->finalCost && current->givenCost < cheapest->givenCost)) {
                    cheapest = current;
                    cheapestPrev = prev;
                }
                prev = current;
                current = current->nextInBucket;
            }

            // Remove cheapest node from bucket
            if (cheapestPrev == nullptr) {
                // Cheapest is the first node in bucket
                buckets[lowestBucketIndex] = cheapest->nextInBucket;
            }
            else {
                cheapestPrev->nextInBucket = cheapest->nextInBucket;
            }

            cheapest->nextInBucket = nullptr;
            nodeCount--;

            return cheapest;
        }

        void remove(Node* nodeToRemove) {
            int bucketIdx = nodeToRemove->bucketIndex;

            Node* current = buckets[bucketIdx];
            Node* prev = nullptr;

            while (current != nullptr) {
                if (current == nodeToRemove) {
                    // Remove from linked list
                    if (prev == nullptr) {
                        buckets[bucketIdx] = current->nextInBucket;
                    }
                    else {
                        prev->nextInBucket = current->nextInBucket;
                    }
                    current->nextInBucket = nullptr;
                    nodeCount--;
                    return;
                }
                prev = current;
                current = current->nextInBucket;
            }
        }

        void updateNode(Node* node, float newFinalCost) {
            // Calculate new bucket index
            int newBucketIdx = static_cast<int>(newFinalCost / BUCKET_SIZE);
            newBucketIdx = std::min(newBucketIdx, MAX_BUCKETS - 1);

            // If bucket changed, remove from old and add to new
            if (newBucketIdx != node->bucketIndex) {
                remove(node);
                node->finalCost = newFinalCost;
                push(node);
            }
            else {
                // Just update the cost
                node->finalCost = newFinalCost;
            }
        }
    };

    //Functions
    void resetNodes();
    void pushToOpenList(Node* node);
    Node* popCheapestNode();
    int getNeighbors(Node* node, Node** outNeighbors);
    bool IsAreaClear(const GridPos& a, const GridPos& b);
    void RubberbandPath(std::vector<GridPos>& gridPath);
    void FillPath(std::vector<GridPos>& gridPath);
    void SmoothPathCatmullRom(std::list<Vec3>& path);

    //Variables
    static constexpr int MAX_MAP_SIZE = 40;
    Node nodePool[MAX_MAP_SIZE * MAX_MAP_SIZE];
    BucketOpenList openList;  // Replace vector with bucket-based open list

    //Inline functions remain the same
    inline float heuristic(const GridPos& a, const GridPos& b, Heuristic type) {
        int dx = abs(a.col - b.col);
        int dy = abs(a.row - b.row);

        if (type == Heuristic::OCTILE) {
            int minVal = dx + ((dy - dx) & ((dy - dx) >> 31));
            return static_cast<float>(dx + dy) + (-0.5857864376269049f) * static_cast<float>(minVal);
        }

        switch (type) {
        case Heuristic::INCONSISTENT: {
            float euclidean = sqrtf(static_cast<float>(dx * dx + dy * dy));
            return ((a.row + a.col) % 2) ? euclidean : 0.f;
        }
        case Heuristic::MANHATTAN:
            return static_cast<float>(dx + dy);
        case Heuristic::EUCLIDEAN: {
            int dist_sq = dx * dx + dy * dy;
            return sqrtf(static_cast<float>(dist_sq));
        }
        case Heuristic::CHEBYSHEV:
            return static_cast<float>((dx > dy) ? dx : dy);
        default:
            return 0.0f;
        }
    }

    inline float getCost(Node* from, Node* to) {
        static constexpr float costTable[2][2] = {
            { 0.0f, 1.0f },
            { 1.0f, 1.41421f }
        };

        int dRow = abs(from->gridPos.row - to->gridPos.row);
        int dCol = abs(from->gridPos.col - to->gridPos.col);

        return costTable[dRow][dCol];
    }

    inline int getIndex(int row, int col) const {
        return row * MAX_MAP_SIZE + col;
    }
};