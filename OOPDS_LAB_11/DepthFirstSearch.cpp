#include "DepthFirstSearch.hpp"

#include <fstream>

using namespace std;
typedef Graph::Vertex Vertex;
typedef Graph::Edge Edge;
typedef std::list<Graph::Vertex> VertexList;
typedef std::list<Graph::Vertex>::iterator VertexItor;
typedef std::list<Graph::Edge> EdgeList;
typedef std::list<Graph::Edge>::iterator EdgeItor;

DepthFirstSearch::DepthFirstSearch(Graph& g) :graph(g)
{
	int num_nodes = graph.getNumVertices();
	pVrtxStatus = new VertexStatus[num_nodes];
	for (int i = 0; i < num_nodes; i++)
		pVrtxStatus[i] = UNEXPLORED;
	ppEdgeStatus = new EdgeStatus*[num_nodes];
	for (int i = 0; i < num_nodes; i++)
		ppEdgeStatus[i] = new EdgeStatus[num_nodes];
	for (int i = 0; i < num_nodes; i++)
		for (int j = 0; j < num_nodes; j++)
			ppEdgeStatus[i][j] = EDGE_UNEXPLORED;
	ppConnectivity = new int*[num_nodes];
	for (int i = 0; i < num_nodes; i++)
		ppConnectivity[i] = new int[num_nodes];
	for (int i = 0; i < num_nodes; i++)
		for (int j = 0; j < num_nodes; j++)
			ppConnectivity[i][j] = PLUS_INF; // initially not connected
	Vertex vtx_1, vtx_2;
	int vtx_1_ID, vtx_2_ID;
	EdgeList edges;
	edges.clear();
	graph.edges(edges);
	for (EdgeItor pe = edges.begin(); pe != edges.end(); ++pe)
	{
		vtx_1 = (*pe).getVertex_1();
		vtx_1_ID = vtx_1.getID();
		vtx_2 = (*pe).getVertex_2();
		vtx_2_ID = vtx_2.getID();
		ppConnectivity[vtx_1_ID][vtx_2_ID] = (*pe).getDistance();
	} for (
		int i = 0; i < num_nodes; i++)
		ppConnectivity[i][i] = 0; // distance of same node
}

void DepthFirstSearch::initialize()
{
	int num_nodes = graph.getNumVertices();
	VertexList verts;
	graph.vertices(verts);
	Vertex vtx_1, vtx_2;
	int vtx_1_ID, vtx_2_ID;
	done = false;
	for (int i = 0; i < num_nodes; i++)
		pVrtxStatus[i] = UNEXPLORED;
	for (int i = 0; i < num_nodes; i++)
		for (int j = 0; j < num_nodes; j++)
			ppEdgeStatus[i][j] = EDGE_UNEXPLORED;
}

void DepthFirstSearch::showConnectivity(ofstream& fout)
{
	int num_nodes = graph.getNumVertices();
	int dist;
	fout << "Connectivity of graph: " << endl;
	fout << " |";
	for (int i = 0; i < num_nodes; i++)
		fout << setw(5) << (char)(i + 'A');
	fout << endl;
	fout << "-----+";
	for (int i = 0; i < num_nodes; i++)
		fout << "-----";
	fout << endl;
	for (int i = 0; i < num_nodes; i++) {
		fout << " " << (char)(i + 'A') << " | ";
		for (int j = 0; j < num_nodes; j++) {
			dist = ppConnectivity[i][j];
			if (dist == PLUS_INF)
				fout << " +oo";
			else
				fout << setw(5) << dist;
		}
		fout << endl;
	}
}

void DepthFirstSearch::dfsTraversal(Vertex& v, Vertex& target, VertexList& path)
{
	visit(v); // mark v as VISITED
	if (v == target) { done = true; return; }
	EdgeList incidentEdges;
	incidentEdges.clear();
	graph.incidentEdges(v, incidentEdges);
	EdgeItor pe = incidentEdges.begin();
	while (!isDone() && pe != incidentEdges.end())
	{
		Edge e = *pe++;
		EdgeStatus eStat = getEdgeStatus(e);
		if (eStat == EDGE_UNEXPLORED) {
			visit(e); // mark edge e EDGE_VISITED
			Vertex w = e.opposite(v);
			if (!isVisited(w)) {
				//traverseDiscovery(e, v);
				path.push_back(w);
				setEdgeStatus(e, DISCOVERY);
				if (!isDone()) {
					dfsTraversal(w, target, path); // recursive call
					if (!isDone()) {
						//traverseBack(e, v); // erase some possible cycle
						Vertex last_pushed = path.back(); // for debugging
						path.pop_back();
					}
				}
			}
			else { setEdgeStatus(e, BACK); } // vertex w has been visited already
		}
	} // end of processing of all incident edges
}

void DepthFirstSearch::findPath(Vertex &start, Vertex &target, VertexList& path)
{
	initialize();
	path.clear();
	path.push_back(start);
	dfsTraversal(start, target, path);
}

void DepthFirstSearch::visit(Vertex& v)
{
	Graph::Vertex* pVtx;
	int numNodes = getGraph().getNumVertices();
	int vtx_ID = v.getID();
	if (vtx_ID >= 0 && vtx_ID < numNodes) {
		pVrtxStatus[vtx_ID] = VISITED;
	}
	else {
		cout << "Vertex (" << v << ") ID is out-of-range (";
		cout << numNodes << ")" << endl;
	}
}

void DepthFirstSearch::visit(Edge& e)
{
	Vertex vtx_1, vtx_2;
	int vtx_1_ID, vtx_2_ID;
	int numNodes = getGraph().getNumVertices();
	vtx_1 = e.getVertex_1();
	vtx_1_ID = vtx_1.getID();
	vtx_2 = e.getVertex_2();
	vtx_2_ID = vtx_2.getID();
	if ((vtx_1_ID >= 0 && vtx_1_ID < numNodes) &&
		(vtx_2_ID >= 0 && vtx_2_ID < numNodes)) {
		ppEdgeStatus[vtx_1_ID][vtx_2_ID] = EDGE_VISITED;
	}
	else {
		cout << "Vertex IDs (" << vtx_1_ID << ", " << vtx_2_ID;
		cout << ") of Edge (" << e << ") is out-of-range (" << numNodes << ")" << endl;
	}
}

void DepthFirstSearch::unvisit(Vertex& v)
{
	Graph::Vertex* pVtx;
	int numNodes = getGraph().getNumVertices();
	int vtx_ID = v.getID();
	if (vtx_ID >= 0 && vtx_ID < numNodes) {
		pVrtxStatus[vtx_ID] = UNEXPLORED;
	}
	else {
		cout << "Vertex (" << v << ") ID is out-of-range (" << numNodes << ")";
		cout << endl;
	}
}

void DepthFirstSearch::unvisit(Edge& e)
{
	Vertex vtx_1, vtx_2;
	int vtx_1_ID, vtx_2_ID;
	int numNodes = getGraph().getNumVertices();
	vtx_1 = e.getVertex_1();
	vtx_1_ID = vtx_1.getID();
	vtx_2 = e.getVertex_2();
	vtx_2_ID = vtx_2.getID();
	if ((vtx_1_ID >= 0 && vtx_1_ID < numNodes) &&
		(vtx_2_ID >= 0 && vtx_2_ID < numNodes)) {
		ppEdgeStatus[vtx_1_ID][vtx_2_ID] = EDGE_UNEXPLORED;
	}
	else {
		cout << "Vertex IDs (" << vtx_1_ID << ", " << vtx_2_ID << ") of Edge (";
		cout << e << ") is out-of-range (" << numNodes << ")" << endl;
	}
}

bool DepthFirstSearch::isVisited(Vertex& v)
{
	Graph::Vertex* pVtx;
	int numNodes = getGraph().getNumVertices();
	int vtx_ID = v.getID();
	if (vtx_ID >= 0 && vtx_ID < numNodes)
	{
		return (pVrtxStatus[vtx_ID] == VISITED);
	}
	else {
		cout << "Vertex (" << v << ") ID is out-of-range (";
		cout << numNodes << ")" << endl;
		return false;
	}
}

bool DepthFirstSearch::isVisited(Edge& e)
{
	Vertex vtx_1, vtx_2;
	int vtx_1_ID, vtx_2_ID;
	EdgeStatus eStat;
	int numNodes = getGraph().getNumVertices();
	vtx_1 = e.getVertex_1();
	vtx_1_ID = vtx_1.getID();
	vtx_2 = e.getVertex_2();
	vtx_2_ID = vtx_2.getID();
	if ((vtx_1_ID >= 0 && vtx_1_ID < numNodes) &&
		(vtx_2_ID >= 0 && vtx_2_ID < numNodes)) {
		eStat = ppEdgeStatus[vtx_1_ID][vtx_2_ID];
		if ((eStat == EDGE_VISITED) || (eStat == DISCOVERY) || (eStat == BACK))
			return true;
		else
			return false;
	}
	else {
		cout << "Vertex IDs (" << vtx_1_ID << ", " << vtx_2_ID << ") of Edge (";
		cout << e << ") is out-of-range (" << numNodes << ")" << endl;
	}
	return false;
}

void DepthFirstSearch::setEdgeStatus(Edge& e, EdgeStatus es)
{
	Vertex vtx_1, vtx_2;
	int vtx_1_ID, vtx_2_ID;
	int numNodes = getGraph().getNumVertices();
	vtx_1 = e.getVertex_1();
	vtx_1_ID = vtx_1.getID();
	vtx_2 = e.getVertex_2();
	vtx_2_ID = vtx_2.getID();
	if ((vtx_1_ID >= 0 && vtx_1_ID < numNodes) &&
		(vtx_2_ID >= 0 && vtx_2_ID < numNodes)) {
		ppEdgeStatus[vtx_1_ID][vtx_2_ID] = es;
	}
	else {
		cout << "Vertex IDs (" << vtx_1_ID << ", " << vtx_2_ID << ") of Edge (";
		cout << e << ") is out-of-range (" << numNodes << ")" << endl;
	}
}

EdgeStatus DepthFirstSearch::getEdgeStatus(Edge& e)
{
	Vertex vtx_1, vtx_2;
	int vtx_1_ID, vtx_2_ID;
	int numNodes = getGraph().getNumVertices();
	EdgeStatus eStat;
	vtx_1 = e.getVertex_1();
	vtx_1_ID = vtx_1.getID();
	vtx_2 = e.getVertex_2();
	vtx_2_ID = vtx_2.getID();
	if ((vtx_1_ID >= 0 && vtx_1_ID < numNodes) &&
		(vtx_2_ID >= 0 && vtx_2_ID < numNodes)) {
		eStat = ppEdgeStatus[vtx_1_ID][vtx_2_ID];
		return eStat;
	}
	else {
		cout << "Edge (" << e << ") was not found from AdjacencyList" << endl;
		return EDGE_NOT_FOUND;
	}
}