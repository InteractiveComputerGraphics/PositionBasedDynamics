#ifndef __INDEXEDTETMESH_H__
#define __INDEXEDTETMESH_H__

#include <vector>
#include "Common/Common.h"

namespace Utilities
{
	class IndexedTetMesh
	{
	public:
		struct Edge
		{
			unsigned int m_vert[2];
		};

		struct Face
		{
			// edge indices
			unsigned int m_edges[3];
			// tet indices
			unsigned int m_tets[2];
		};

		struct Tet
		{
			unsigned int m_edges[6];
			unsigned int m_faces[4];
		};

		// Stores the indices of each tet connected to a specific vertex
		struct VertexTets
		{
			VertexTets()
			{
				m_tIndices = 0;
				m_numTets = 0;
			}

			~VertexTets()
			{
				delete [] m_tIndices;
			}

			unsigned int m_numTets;
			unsigned int* m_tIndices;
		};

		// Stores the indices of each face connected to a specific vertex
		struct VertexFaces
		{
			VertexFaces()
			{
				m_fIndices = 0;
				m_numFaces = 0;
			}

			~VertexFaces()
			{
				delete [] m_fIndices;
			}

			unsigned int m_numFaces;
			unsigned int* m_fIndices;
		};

		// Stores the indices of each edge connected to a specific vertex
		struct VertexEdges
		{
			VertexEdges()
			{
				m_eIndices = 0;
				m_numEdges = 0;
			}

			~VertexEdges()
			{
				delete [] m_eIndices;
			}

			unsigned int m_numEdges;
			unsigned int* m_eIndices;
		};

	public:
		typedef std::vector<unsigned int> Tets;
		typedef std::vector<unsigned int> Faces;
		typedef std::vector<Tet> TetData;
		typedef std::vector<Face> FaceData;
		typedef std::vector<Edge> Edges;
		typedef std::vector<VertexTets> VerticesTets;
		typedef std::vector<VertexFaces> VerticesFaces;
		typedef std::vector<VertexEdges> VerticesEdges;

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
