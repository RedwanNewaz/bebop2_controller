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

/// @brief It provides methods for adding edges, checking if a given sequence of vertices is valid, and performing depth-first search traversals from specific starting vertices to find sets of visited vertices.

class DFA
{
public:

    /// @brief Constructor.
    DFA();
    
   /// @brief Checks if the given sequence of vertices is a valid path in the DFA or not, by performing DFS from the first vertex. 
   /// @param choices Sequence of vertices
   /// @return a boolean value indicating whether the given sequence of choices is a valid sequence or not.
   bool isValid(const list<int>& choices);
   
   /// number of vertices
   size_t size();
   /// @brief Does a DFS traversal from the EXP Node to the Start Node
   /// @return A set of integers representing visited node obtained during the DFS.
   set<int> defaultExpPath();
   
   /// @brief Does a DFS traversal from the SIM Node to the Start Node
   /// @return A set of integers representing visited node obtained during the DFS.
   set<int> defaultSimPath();
protected:
   /// @brief Method that add an edge to the graph
   /// @param v Current vertex
   /// @param w Next vertex to be added in v's list
   void addEdge(int v, int w);
   
    /// @brief Keeps track of the visited vertices. Basically finds out Reachability from v
   /// @param v Current vertex
   /// @param target target Vertex
   /// @return True if we have the target vertex else False
   bool DFS(int v, int target);
private:
   map<int, bool> visited;
   map<int, list<int> > adj;
   set<int> vertices, visitedVertices;
};

#endif // DFA_H
