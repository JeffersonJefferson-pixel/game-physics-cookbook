#ifndef _H_QUAD_TREE_
#define _H_QUAD_TREE_

#include "Geometry2D.h"
#include <vector>
using std::vector;

struct QuadTreeData
{
    void *object;
    Rectangle2D bounds;
    bool flag;

    inline QuadTreeData(void *o, const Rectangle2D &b) : object(o), bounds(b), flag(false) {}
};

class QuadTreeNode
{
protected:
    std::vector<QuadTreeNode> children;
    std::vector<QuadTreeData *> contents;
    int currentDepth;
    static int maxDepth;
    static int maxObjectsPerNode;
    Rectangle2D nodeBounds;

public:
    inline QuadTreeNode(const Rectangle2D &bounds) : nodeBounds(bounds), currentDepth(0) {}
    // check if a node is a leaf.
    bool IsLeaf();
    // count how many objects a node contains.
    int NumObjects();
    void Insert(QuadTreeData &data);
    void Remove(QuadTreeData &data);
    void Update(QuadTreeData &data);
    void Shake();
    void Split();
    // reset objects flag.
    void Reset();
    std::vector<QuadTreeData *> Query(const Rectangle2D &area);
};

typedef QuadTreeNode QuadTree;

#endif