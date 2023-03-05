#ifndef DFA_H
#define DFA_H
#include <iostream>
#include <list>
#include <map>
#include <set>
#include <cassert>
using namespace std;

enum NODE{
  EXP=0,
  SIM,
  BAG,
  DUMMY,
  APRILTAG,
  BEBOP,
  JOY,
  EKF,
  LOWPASS,
  RVIZ,
  START
};

class DFA
{
public:
    DFA();
   // DFS traversal of the vertices
   bool isValid(const list<int>& choices);
   // number of vertices
   size_t size();
   // default path
   set<int> defaultExpPath();
   set<int> defaultSimPath();
protected:
   // function to add an edge to graph
   void addEdge(int v, int w);
   // reachable from v
   bool DFS(int v, int target);
private:
   map<int, bool> visited;
   map<int, list<int> > adj;
   set<int> vertices, visitedVertices;
};

#endif // DFA_H
