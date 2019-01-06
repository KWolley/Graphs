//
// graph.cpp
//

#include <iostream>
#include <sstream>
#include <deque>
#include <queue>

#include "graph.hpp"

using namespace std;

// fwd declarations
string make_dot(Graph* g);
string what(int& v);

Graph::Graph() {
  // DONE FOR YOU
  directed = false; // graphs are undirected by default
}

Graph::~Graph() {
  // "DONE" FOR YOU
  // no implementation needed
}

vector<Node*> Graph::getNodes() {
  // DONE FOR YOU
  return nodes;
}

vector<Edge*> Graph::getEdges() {
  // DONE FOR YOU
  return edges;
}

int Graph::getClock() {
  // DONE FOR YOU
  return clock;
}

void Graph::addNode(Node& n) {
  // DONE FOR YOU
  nodes.push_back(&n);
}

void Graph::addEdge(Edge& e) {
  // DONE FOR YOU
  edges.push_back(&e);
}
 
void Graph::removeNode(Node& n) {
  // DONE FOR YOU
  for (vector<Node*>::iterator it = nodes.begin();
       it != nodes.end(); 
       it++) {
    if (&n == *it) {
      nodes.erase(it);
      break;
    }
  }
}
 
void Graph::removeEdge(Edge& e) {
  // DONE FOR YOU
  for (vector<Edge*>::iterator it = edges.begin();
       it != edges.end(); 
       it++) {
    if (&e == *it) {
      edges.erase(it);
      break;
    }
  }
}

void Graph::setDirected(bool val) {
  // DONE FOR YOU
  directed = val;
}

bool Graph::isDirected() {
  // DONE FOR YOU
  return directed;
}

set<Edge*> Graph::getAdjacentEdges(Node& n) {
  // DONE FOR YOU
  set<Edge*> ret;
  for (vector<Edge*>::iterator it = edges.begin(); it != edges.end(); it++) {
    Edge* edge = *it;
    if (edge->getStart() == &n) {
      ret.insert(edge);
    }
    if (!directed && edge->getEnd() == &n) {
      ret.insert(edge);
    }
  }
  return ret;
}

Node::Node(string s) {
  // DONE FOR YOU
  data = s;
}

Node::~Node() {
  // "DONE" FOR YOU
  //
  // This is equivalent to the default destructor the compiler will
  // generate for you. As a rule of thumb, you only need to `delete`
  // member variables that were created with `new` inside this
  // class. So, you don't need to do anything here. It could have been
  // left out of the header/impl file entirely.
}

string Node::getData() {
  // DONE FOR YOU
  return data;
}

void Node::setRank(int r) {
  // DONE FOR YOU
  rank = r;
}

// clear sets the color to WHITE, the discovery/finish time and rank
// to -1, and sets the predecessor to NULL.
void Node::clear() {
  color = WHITE;
  discovery_time = -1;
  completion_time = -1;
  rank = -1;
  predecessor = NULL;
}

Edge::Edge(Node& n1, Node& n2) {
  // DONE FOR YOU
  a = &n1;
  b = &n2;
}

Edge::~Edge() {
  // "DONE" FOR YOU
}


Node* Edge::getStart() {
  // DONE FOR YOU
  return a;
}

Node* Edge::getEnd() {
  // DONE FOR YOU
  return b;
}

int Edge::getType() {
  // DONE FOR YOU
  return type;
}

void Edge::setType(int edge_type) {
  type = edge_type;
}

// clear resets all nodes to have WHITE color, with -1 discovery and
// finish times and rank. Resets all edges to type
// UNDISCOVERED_EDGE. It resets the graph clock to 0.
void Graph::clear() {
  clock = 0;
  for (vector<Node*>::iterator it = nodes.begin(); it != nodes.end(); it++) {
    Node *node = *it;
    node -> clear();
  }
  for (vector<Edge*>::iterator it = edges.begin(); it != edges.end(); it++) {
    Edge *edge = *it;
    edge -> setType(UNDISCOVERED_EDGE);
  }
}

void Graph::tick(string message) {
  // DONE FOR YOU
  //
  // optional debugging function, disabled with if(false) to prevent
  // unused var warning. Change this however you want.
  //
  // recommended use: call it just after incrementing the clock.
  if (false) {
    cout << "// " << message << endl << make_dot(this) << endl;
  }
}

// setColor sets the 'color' of a node. Be sure to set the
// appropriate time variable depending on what the new color is: set
// discovery_time if the node is now GRAY, and completion_time if
// the node is now BLACK. If it is now WHITE, this should be
// equivalent to calling clear().
void Node::setColor(int search_color, int time) {
  color = search_color;
  if (search_color == WHITE){
    clear();
    //discovery_time = -1;
    //completion_time = -1;
  }
  if (search_color == GRAY){
    discovery_time = time;
  }
  if (search_color == BLACK){
    completion_time = time;
  }
}

// getDiscoveryInformation gets the color, discovery time, and
// finish time to 'color', 'disco_time', 'finish_time', and
// 'bfs_rank' respectively.
void Node::getDiscoveryInformation(int& color, int& disco_time, 
				   int& finish_time, int& bfs_rank) {
  color = this -> color;
  disco_time = discovery_time ;
  finish_time = completion_time   ;
  bfs_rank = rank ;
}

// Sets the predecessor node in the spanning tree. The predecessor
// node was the node that we were exploring when we first discovered
// a node (e.g. it was WHITE when we found it).
void Node::setPredecessor(Node& other) {
  predecessor = &other;
}

// isAncestor returns true if the given node is reachable by
// traversing the other node's predecessor list. It is like
// searching through a linked list. You can do this iteratively
// (with a cursor) or recursively by calling isAncestor on non-null
// predecessors. Try it both ways and impress your friends.
bool Node::isAncestor(Node& other) {
  if (this->data == other.data) {
    return true;
  }
  if (predecessor == nullptr){
    return false;
  }
  return predecessor->isAncestor(other);
}

// vector<Node*> getConnectedNodes (Graph* graph, Node* node) {
//   vector<Node*> nodes;
//   for (auto* edge : graph->getAdjacentEdges(*node)) {
//     if (edge->getStart() != node) {
//       nodes.push_back(edge->getStart());
//     } else if (edge->getEnd() != node) {
//       nodes.push_back(edge->getEnd());
//     }
//   }
//   return nodes;
// }

// void dfs_recursive(Graph* graph, Node* node, set<Node*>* visited, int* clock) {
//   node->setColor(GRAY, (*clock)++); // set current node to gray
//   cout<<"start" <<  ", clock: " << *clock << endl;
//   //node->setColor(GRAY, (*clock)); // set current node to gray
//   set<Edge *> adj_edges = getAdjacentEdges(node);

//   for (auto* child_node : getAdjacentEdges(graphnode)) {
//     if (visited->find(node) != visited->end()) {
//       node->setColor(BLACK, (*clock)++);
//       cout<<"if loop" <<  ", clock: " << *clock << endl;
//       //node->setColor(BLACK, (*clock));
//       continue;
//     }
//     visited->emplace(child_node);
//     cout<<"outside if loop" <<  ", clock: " << *clock << endl;
//     //node->setColor(BLACK, (*clock)++);
//     dfs_recursive(graph, child_node, visited, clock);
    
//     node->setColor(BLACK, (*clock)++);
//   }
//   cout<<"outside for loop" << ", clock: " << *clock << endl;
//   //node->setColor(BLACK, (*clock));
//   node->setColor(BLACK, (*clock)++);
// }

// void dfs_recursive(Graph* graph, Node* node, set<Node*>* visited, int* clock) {
//   node->setColor(GRAY, (*clock)++); // set current node to gray
//   for (auto* child_node : getConnectedNodes(graph, node)) {
//     if (visited->find(node) != visited->end()) {
//       cout<<"continue loop" << ", visited: " << visited <<  ", clock: " << *clock << endl;
//       // visited->emplace(child_node);
//       // dfs_recursive(graph, child_node, visited, clock);
//       continue;
//     }
//     visited->emplace(child_node);
//     cout << "visited " << visited << endl;
//     cout << "clock " << *clock << endl;
//     dfs_recursive(graph, child_node, visited, clock);
//   }
//   node->setColor(BLACK, (*clock)++);
// }

  // dfs(start) runs a depth-first search from the indicated start
  // node, and explores all reachable nodes. This ignores unreachable
  // nodes. When this function returns, all explored nodes are colored
  // BLACK, all unreachable nodes are WHITE. All explored nodes have
  // correct discovery/exit time information. All edges have correct
  // edge types (unfollowed edges should remain UNDISCOVERED).
  //
  // For a DFS, mark nodes GRAY when we first discover them, and BLACK
  // when we exit (finish) them.
void Graph::dfs(Node& start) {
  int c , discotime, compltime, r;
  start.getDiscoveryInformation(c,discotime,compltime,r);
  start.setColor(GRAY,this->clock);
  this->clock++;
  set<Edge *> adj_edges = getAdjacentEdges(start);
  for (auto child_nodes = adj_edges.begin(); child_nodes != adj_edges.end(); ++child_nodes){
    Edge *e = *child_nodes;
    Node *other = e->getEnd();
    other -> setPredecessor(start);
    other-> getDiscoveryInformation(c,discotime,compltime,r);
    if (c == WHITE){
      e->setType(TREE_EDGE); // 10
      this->dfs(*other);
    }
    if (c == GRAY){
      e->setType(BACK_EDGE); // 11
    }
    if (c == BLACK){
      if(start.isAncestor(*other)){
        e->setType(FORWARD_EDGE); //12
      } else {
        e -> setType(FORWARD_EDGE); // 12
      }
    }
  }
  start.setColor(BLACK,this->clock);
  this -> clock++;
  return;
}

  // bfs(start) runs a breadth-first search starting from the given
  // node. It sets the 'rank' value on all nodes to something
  // appropriate: -1 for unreachable nodes, 0 for the start node, 1
  // for nodes that are one edge from the start node, and so forth.
  //
  // For a BFS, mark nodes GRAY when they are enqueued, and BLACK when
  // they are dequeued.
void Graph::bfs(Node& start) {
  // create list to store visited nodes
  std::queue <Node*> visited;
  // set start rank to 0
  start.setRank(0);
  // add start to queue
  visited.emplace(&start);
  // mark start gray
  start.setColor(GRAY,this->clock);

  while(!visited.empty() ){
    // set current node to top of queue
    Node* node_current = visited.front();
    // pop first node from visited
    visited.pop();
    // initialize get discovery information
    int c, dt, ct, r;
    // visit node
    node_current -> getDiscoveryInformation(c,dt,ct,r);
    // mark node black
    node_current -> setColor(BLACK,this->clock);
    // get edges
    set<Edge *> e = getAdjacentEdges(*node_current);
    for (auto child_nodes = e.begin(); child_nodes != e.end(); ++child_nodes){
      // for each node
      Edge *tmp_e = *child_nodes;
      Node *other_node = tmp_e->getEnd();
      int c2, dt2, ct2, r2;
      other_node -> getDiscoveryInformation(c2,dt2,ct2,r2);
      // if other_node is unmarked:
      if (c2 == WHITE){
        other_node -> setRank(r+1);
        // add other_node to queue
        visited.emplace(other_node);
        // mark other_node gray
        other_node -> setColor(GRAY,this->clock);
      }
    }
  }
}

void Graph::bfs(Node& start, Node& finish) {
  // create list to store visited nodes
  std::queue <Node*> visited;
  // creat queue for finish info
  //std::queue <Node* > end_loop;

  // set start rank to 0
  start.setRank(0);
  // add start to queue
  visited.push(&start);
  // mark start gray
  start.setColor(GRAY,this->clock);
  // assign finish to the queue
  //end_loop.push(&finish);

  while(!visited.empty()){

    // set current node to top of queue
    Node* node_current = visited.front();
    // pop first node from visited
    visited.pop();
    // initialize get discovery information

    int c, dt, ct, r;
    // visit node
    node_current -> getDiscoveryInformation(c,dt,ct,r);

    if (!this -> isDirected()){
      node_current -> setRank(1);
      return;
    }
        
    if (node_current == &finish){
      node_current -> setRank(r);
      return; 
    }

    // mark node black
    node_current -> setColor(BLACK,this->clock);

    // get edges
    set<Edge *> e = getAdjacentEdges(*node_current);

    for (auto child_edges = e.begin(); child_edges != e.end(); ++child_edges){
      // for each node
      Edge *tmp_e = *child_edges;
      Node *other_node = tmp_e->getEnd();
      int c2, dt2, ct2, r2;
      other_node -> getDiscoveryInformation(c2,dt2,ct2,r2);
      // if other_node is unmarked:
      if (c2 == WHITE){
        other_node -> setRank(r+1);
        // add other_node to queue
        visited.emplace(other_node);
        // mark other_node gray
        other_node -> setColor(GRAY,this->clock);
      }
    }
  }
}



// overloading operator << lets you put a Graph object into an output
// stream.
ostream &operator << (ostream& out, Graph graph) {
  // DONE FOR YOU
  out << graph.nodes.size() << " Nodes:" << endl;
  out << "[";
  for (vector<Node*>::iterator it = graph.nodes.begin(); it != graph.nodes.end(); it++) {
    Node* n = *it;
    out << *n << ", ";
  }
  out << "]" << endl;
  out << graph.edges.size() << " Edges:" << endl;
  out << "[";
  for (vector<Edge*>::iterator it = graph.edges.begin(); it != graph.edges.end(); it++) {
    Edge* e = *it;
    out << *e << ", ";
  }
  out << "]";
  return out;
}

// overloading operator << lets you put a Node object into an output
// stream.
ostream &operator << (std::ostream& out, Node node) {
  // DONE FOR YOU
  out << node.data;
  return out;
}

// overloading operator << lets you put an Edge object into an output
// stream.
ostream &operator << (std::ostream& out, Edge edge) {
  // DONE FOR YOU
  out << *edge.a << " -- " << *edge.b;
  return out;
}

// what is a helper function that turns a constant (e.g. WHITE or
// FORWARD_EDGE) into a string for debugging goodness. 
string what(int& v) {
  // DONE FOR YOU
  string ret = "Unknown";
  if (v == WHITE) {
    ret = "white";
  } else if (v == GRAY) {
    ret = "gray";
  } else if (v == BLACK) {
    ret = "black";
  } else if (v == UNDISCOVERED_EDGE) {
    ret = "undiscovered";
  } else if (v == TREE_EDGE) {
    ret = "tree";
  } else if (v == BACK_EDGE) {
    ret = "back";
  } else if (v == FORWARD_EDGE) {
    ret = "forward";
  } else if (v == CROSS_EDGE) {
    ret = "cross";
  }
  return ret;
}


// make_dot generates a dotfile string for viewing your graph. Have
// tick() print out the results of this function, and then use
// http://www.webgraphviz.com/ (or get a copy of GraphViz) to view the
// result. If you're stuck, this can help you get un-stuck.
string make_dot(Graph* g) {
  // DONE FOR YOU
  stringstream ss;
  vector<Node*> nodes = g->getNodes();
  vector<Edge*> edges = g->getEdges();
  string gt = "graph";
  string con = "--";
  if (g->isDirected()) {
    con = "->";
    gt = "digraph";
  }
  ss << gt << " homework {" << endl;
  int c, dt, ft, r;
  string textColor = "black";
  for (auto it=nodes.begin(); it != nodes.end(); ++it) {
    Node* n = *it;
    n->getDiscoveryInformation(c, dt, ft, r);
    if (c == BLACK) {
      textColor = "white";
    } else {
      textColor = "black";
    }
    ss << "  " << n->getData() << " [ style=\"filled\" fontcolor=\"" << textColor << "\" fillcolor=\"" << what(c) << "\"" << " ]" << endl;
  }
  
  string edgeColor = "black";
  for (auto it=edges.begin(); it != edges.end(); ++it) {
    Edge* e = *it;
    if (e->getType() == TREE_EDGE) {
      edgeColor = "black";
    } else if (e->getType() == FORWARD_EDGE) {
      edgeColor = "purple";
    } else if (e->getType() == BACK_EDGE) {
      edgeColor = "blue";
    } else if (e->getType() == CROSS_EDGE) {
      edgeColor = "green";
    } else {
      edgeColor = "gray";
    }
    ss << "  " << e->getStart()->getData() << con << e->getEnd()->getData() << " [color=\"" << edgeColor << "\"]" << endl;
  }
  ss << "}" << endl;
  return ss.str();
}
