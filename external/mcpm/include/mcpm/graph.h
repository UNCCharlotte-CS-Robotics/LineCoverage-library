#pragma once

#include <iostream>
#include <list>
#include <vector>

namespace mcpm {
	class Graph {
		public:

			//n is the number of vertices
			//edges is a list of pairs representing the edges (default = empty list)
			Graph(int n, const std::list< std::pair<int, int> > & edges = std::list< std::pair<int, int> >()) :
				n(n),
				m(0),
				adjMat(n, std::vector<bool>(n, false)),
				adjList(n),
				edges(),
				edgeIndex(n, std::vector<int>(n, -1)) {
					for(std::list< std::pair<int, int> >::const_iterator it = edges.begin(); it != edges.end(); it++) {
						int u = (*it).first;
						int v = (*it).second;
						AddEdge(u, v);
					}
				}


			//Default constructor creates an empty graph
			Graph(): n(0), m(0) {};

			//Returns the number of vertices
			int GetNumVertices() const { return n; };
			//Returns the number of edges
			int GetNumEdges() const { return m; };

			//Given the edge's index, returns its endpoints as a pair
			std::pair<int, int> GetEdge(int e) const {
				if(e > (int)edges.size())
					throw "Error: edge does not exist";
				return edges[e];
			}

			//Given the endpoints, returns the index
			int GetEdgeIndex(int u, int v) const {
				if( u > n or
						v > n )
					throw "Error: vertex does not exist";

				if(edgeIndex[u][v] == -1)
					throw "Error: edge does not exist";

				return edgeIndex[u][v];
			}

			//Adds a new vertex to the graph
			void AddVertex() {
				for(int i = 0; i < n; i++) {
					adjMat[i].push_back(false);
					edgeIndex[i].push_back(-1);
				}
				n++;
				adjMat.push_back( std::vector<bool>(n, false) );
				edgeIndex.push_back( std::vector<int>(n, -1) );
				adjList.push_back( std::list<int>() );
			}

			//Adds a new edge to the graph
			void AddEdge(int u, int v) {
				if( u > n or
						v > n )
					throw "Error: vertex does not exist";

				if(adjMat[u][v]) return;

				adjMat[u][v] = adjMat[v][u] = true;
				adjList[u].push_back(v);
				adjList[v].push_back(u);

				edges.push_back(std::pair<int, int>(u, v));
				edgeIndex[u][v] = edgeIndex[v][u] = m++;
			}

			//Returns the adjacency list of a vertex
			const std::list<int> & AdjList(int v) const {
				if(v > n)
					throw "Error: vertex does not exist";

				return adjList[v];
			}

			//Returns the graph's adjacency matrix
			const std::vector< std::vector<bool> > & AdjMat() const {
				return adjMat;
			}

			void PrintGraph() {
				std::cout << "Number of vertices: " << n << std::endl;
				std::cout << "Number of edges: " << m << std::endl;
				for(int i = 0; i < m; ++i) {
					auto e = edges[i];
					std::cout << e.first << " " << e.second << std::endl;
				}
			}

		private:
			//Number of vertices
			int n;
			//Number of edges
			int m;

			//Adjacency matrix
			std::vector< std::vector<bool> > adjMat;

			//Adjacency lists
			std::vector< std::list<int> > adjList;

			//Array of edges
			std::vector< std::pair<int, int> > edges;

			//Indices of the edges
			std::vector< std::vector<int> > edgeIndex;

	};
} /* namespace mcpm */
