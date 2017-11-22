/** main.cpp */
#include <fstream>
#include <string>
#include "DepthFirstSearch.hpp"

#define NUM_NODES 20
#define NUM_EDGES 70

typedef Graph::Vertex Vertex;
typedef Graph::Edge Edge;
typedef std::list<Graph::Vertex> VtxList;
typedef std::list<Graph::Edge> EdgeList;
typedef std::list<Graph::Vertex>::iterator VtxItor;
typedef std::list<Graph::Edge>::iterator EdgeItor;

using namespace std;

void main()
{
	//.....
	Vertex v[NUM_NODES] ={
		Vertex("V00", 0, UNEXPLORED),
		Vertex("V01", 1, UNEXPLORED),
		//. . . . . .
	};
	Graph::Edge edges[NUM_EDGES] =
	{
		Edge(v[0], v[1], 1), Edge(v[1], v[0], 1),
		Edge(v[0], v[5], 1), Edge(v[5], v[0], 1),
		Edge(v[1], v[2], 1), Edge(v[2], v[1], 1),
		//. . . . . . .
	};
	int test_start = 0;
	int test_end = 24;
	

	Graph simpleGraph(NUM_NODES);

	ofstream fout;	//define ofstream fout
	fout.open("output.txt");
	if (fout.fail())
	{
		cout << "Fail to open output.txt file !!" << endl;
		exit(1);
	}

	fout << "Inserting vertices .." << endl;

	for (int i = 0; i<NUM_NODES; i++) {
		simpleGraph.insertVertex(v[i]);
	}

	VtxList vtxLst;
	simpleGraph.vertices(vtxLst);

	int count = 0;

	fout << "Inserted vertices: ";
	for (VtxItor vItor = vtxLst.begin(); vItor != vtxLst.end(); ++vItor) {
		fout << *vItor << ", ";
	}
	fout << endl;

	fout << "Inserting edges .." << endl;
	for (int i = 0; i<NUM_EDGES; i++)
	{
		simpleGraph.insertEdge(edges[i]);
	}
	fout << "Inserted edges: " << endl;

	count = 0;

	EdgeList egLst;
	simpleGraph.edges(egLst);
	for (EdgeItor p = egLst.begin(); p != egLst.end(); ++p)
	{
		count++;
		fout << *p << ", ";
		if (count % 5 == 0)
			fout << endl;
	}
	fout << endl;

	fout << "Print out Graph based on Adjacency List .." << endl;
	simpleGraph.printGraph();

	DepthFirstSearch dfsGraph(simpleGraph);
	VtxList path;

	dfsGraph.showConnectivity(fout);
	dfsGraph.findPath(v[test_start], v[test_end], path);

	fout << "Path (" << v[test_start] << " => " << v[test_end] << ") : ";
	for (VtxItor vItor = path.begin(); vItor != path.end(); ++vItor)
		fout << *vItor << " ";
	fout << endl;

	dfsGraph.findPath(v[test_end], v[test_start], path);

	fout << "Path (" << v[test_end] << " => " << v[test_start] << ") : ";
	for (VtxItor vItor = path.begin(); vItor != path.end(); ++vItor)
		fout << *vItor << " ";
	fout << endl;

	fout.close();
}