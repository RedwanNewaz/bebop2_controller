#include "dfa.h"

DFA::DFA()
{
  addEdge(EXP, BEBOP);
  addEdge(BEBOP, JOY);
  addEdge(SIM, BAG);
  addEdge(SIM, DUMMY);
  addEdge(DUMMY, JOY);
  addEdge(BAG, APRILTAG);
  addEdge(JOY, APRILTAG);

//  // ekf filter

  addEdge(JOY, EKF);
  addEdge(APRILTAG, EKF);
  addEdge(EKF, RVIZ);
  addEdge(EKF, START);
  addEdge(RVIZ, START);

//  // lowpass filter
  addEdge(JOY, LOWPASS);
  addEdge(APRILTAG, LOWPASS);
  addEdge(LOWPASS, RVIZ);
  addEdge(LOWPASS, START);
  addEdge(RVIZ, START);



}

set<int> DFA::defaultExpPath()
{
  visitedVertices.clear();
  DFS(EXP, START);
  return visitedVertices;
}

set<int> DFA::defaultSimPath()
{
  visitedVertices.clear();
  DFS(SIM, START);
  return visitedVertices;
}

void DFA::addEdge(int v, int w)
{
    adj[v].push_back(w); // Add w to v’s list.
    vertices.insert(v);
    vertices.insert(w);
}

size_t DFA::size()
{
  return vertices.size();
}

bool DFA::isValid(const list<int>& choices)
{
    if(!(choices.front() == EXP || choices.front() == SIM))
        return false;
    if(choices.back() != START )
        return false;
//  assert(choices.back() == START && (choices.front() == EXP || choices.front() == SIM));
  //default is all nodes are visited
  for(auto v:vertices)
    {
      visited[v] = true;
    }
  // select a path
  for(auto v:choices)
    {
      visited[v] = false;
    }
  // clear previous result
  visitedVertices.clear();
  // perform depth first search
  bool result = DFS(choices.front(), choices.back());
  //make the DFA reuseable
  for(auto v:vertices)
  {
    visited[v] = false;
  }
  return result;
}

bool DFA::DFS(int v, int target)
{
  // Mark the current node as visited and
    // print it
    visited[v] = true;
    visitedVertices.insert(v);
    // cout << v << " ";
    if(v == target)
      return true;

    // Recur for all the vertices adjacent
    // to this vertex
    list<int>::iterator i;
    for (i = adj[v].begin(); i != adj[v].end(); ++i)
        if (!visited[*i])
            return DFS(*i, target);
    return false;
}
