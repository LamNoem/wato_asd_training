#ifndef PLANNER_STRUCTS_HPP_
#define PLANNER_STRUCTS_HPP_

#include <vector>
#include <cmath>
#include <queue>
#include <unordered_map>

struct CellIndex
{
    int x, y;

    CellIndex(int xx, int yy) : x(xx), y(yy) {}
    CellIndex() : x(0), y(0) {}

    bool operator==(const CellIndex &other) const
    {
        return x == other.x && y == other.y;
    }
};

struct CellIndexHash
{
    std::size_t operator()(const CellIndex &idx) const
    {
        return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
    }
};

struct AStarNode
{
    CellIndex index;
    double f_score;

    AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

struct CompareF
{
    bool operator()(const AStarNode &a, const AStarNode &b)
    {
        return a.f_score > b.f_score;
    }
};


#endif