#ifndef __INDEXEDTETMESH_H__
#define __INDEXEDTETMESH_H__

#include <vector>
#include <array>
#include "Common/Common.h"

namespace Utilities
{
	class IndexedTetMesh
	{
	public:
		struct Edge
		{
			std::array<unsigned int, 2> m_vert;
		};

		struct Face
		{
			// edge indices
			std::array<unsigned int, 3> m_edges;
			// tet indices
			std::array<unsigned int, 2> m_tets;
		};

		struct Tet
		{
			std::array<unsigned int, 6> m_edges;
			std::array<unsigned int, 4> m_faces;
		};

	public:
		typedef std::vector<unsigned int> Tets;
		typedef std::vector<unsigned int> Faces;
		typedef std::vector<Tet> TetData;
		typedef std::vector<Face> FaceData;
		typedef std::vector<Edge> Edges;
		typedef std::vector<std::vector<unsigned int>> VerticesTets;
		typedef std::vector<std::vector<unsigned int>> VerticesFaces;
		typedef std::vector<std::vector<unsigned int>> VerticesEdges;

	protected:
		unsigned int m_numPoints;
		Tets m_tetIndices;
		Faces m_faceIndices;
		Edges m_edges;
		FaceData m_faces;
		TetData m_tets;
		VerticesTets m_verticesTets;
		VerticesFaces m_verticesFaces;
		VerticesEdges m_verticesEdges;

	public:
		IndexedTetMesh();
		~IndexedTetMesh();

		void release();
		void initMesh(const unsigned int nPoints, const unsigned int nEdges, const unsigned int nFaces, const unsigned int nTets);
		void addTet(const unsigned int * const indices);
		void addTet(const int * const indices);

		const Faces& getFaces() const {return m_faceIndices;}
		Faces& getFaces(){return m_faceIndices;}
		const Tets& getTets() const {return m_tetIndices;}
		Tets& getTets(){return m_tetIndices;}
		Edges& getEdges() {return m_edges;}
		const Edges& getEdges() const {return m_edges;}
		const FaceData& getFaceData() const {return m_faces;}
		const TetData& getTetData() const {return m_tets;}
		const VerticesTets& getVertexTets() const {return m_verticesTets;}
		const VerticesFaces& getVertexFaces() const {return m_verticesFaces;}
		const VerticesEdges& getVertexEdges() const {return m_verticesEdges;}


		unsigned int numVertices() const { return m_numPoints; }
		unsigned int numFaces() const { return (unsigned int)m_faceIndices.size() / 3; }
		unsigned int numTets() const { return (unsigned int)m_tetIndices.size() / 4; }
		unsigned int numEdges() const { return (unsigned int)m_edges.size(); }

		void buildNeighbors();
	};
}

#endif
