#ifndef __INDEXEDFACEMESH_H__
#define __INDEXEDFACEMESH_H__

#include <vector>
#include "Common/Common.h"
#include <iterator>

namespace Utilities
{
	class IndexedFaceMesh
	{
	public:
		struct Edge
		{
			unsigned int m_face[2];
			unsigned int m_vert[2];
		};

		struct Face
		{
			unsigned int *m_edges;
		};

		// Stores the indices of each face connected to a specific vertex
		struct VertexFaces
		{
			VertexFaces()
			{
				m_fIndices = 0;
				m_numFaces = 0;
			}

			VertexFaces(VertexFaces const& other)
			{
				*this = other;
			}

			VertexFaces& operator=(VertexFaces const& other)
			{
				m_numFaces = other.m_numFaces;
				m_fIndices = new unsigned int[m_numFaces];
#if defined(_MSC_VER)
				std::copy(other.m_fIndices, other.m_fIndices + m_numFaces,
					stdext::unchecked_array_iterator<unsigned int*>(m_fIndices));
#else
				std::copy(other.m_fIndices, other.m_fIndices + m_numFaces, m_fIndices);
#endif		
				return *this;
			}

			~VertexFaces()
			{
				delete[] m_fIndices;
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

			VertexEdges(VertexEdges const& other)
			{
				*this = other;
			}

			VertexEdges& operator=(VertexEdges const& other)
			{
				m_numEdges = other.m_numEdges;
				m_eIndices = new unsigned int[m_numEdges];
#if defined(_MSC_VER)
				std::copy(other.m_eIndices, other.m_eIndices + m_numEdges,
					stdext::unchecked_array_iterator<unsigned int*>(m_eIndices));
#else
				std::copy(other.m_eIndices, other.m_eIndices + m_numEdges, m_eIndices);
#endif		
				return *this;
			}

			~VertexEdges()
			{
				delete[] m_eIndices;
			}

			unsigned int m_numEdges;
			unsigned int* m_eIndices;
		};

	public:
		typedef std::vector<unsigned int> Faces;
		typedef std::vector<Vector3r> FaceNormals;
		typedef std::vector<Vector3r> VertexNormals;
		typedef std::vector<Face> FaceData;
		typedef std::vector<Edge> Edges;
		typedef std::vector<VertexFaces> VerticesFaces;
		typedef std::vector<VertexEdges> VerticesEdges;
		typedef std::vector<unsigned int> UVIndices;
		typedef std::vector<Vector2r> UVs;

	protected:
		unsigned int m_numPoints;
		Faces m_indices;
		Edges m_edges;
		FaceData m_faces;
		bool m_closed;
		UVIndices m_uvIndices;
		UVs m_uvs;
		VerticesFaces m_verticesFaces;
		VerticesEdges m_verticesEdges;
		unsigned int m_verticesPerFace;
		FaceNormals m_normals;
		VertexNormals m_vertexNormals;

	public:
		IndexedFaceMesh(const unsigned int verticesPerFace = 3);
		IndexedFaceMesh(IndexedFaceMesh const& other);
		IndexedFaceMesh& operator=(IndexedFaceMesh const& other);
		~IndexedFaceMesh();

		void release();
		bool isClosed() const;
		void initMesh(const unsigned int nPoints, const unsigned int nEdges, const unsigned int nFaces);
		void addFace(const unsigned int * const indices);
		void addFace(const int * const indices);
		void addUV(const Real u, const Real v);
		void addUVIndex(const unsigned int index);

		const Faces& getFaces() const { return m_indices; }
		Faces& getFaces(){ return m_indices; }
		const FaceNormals& getFaceNormals() const { return m_normals; }
		FaceNormals& getFaceNormals(){ return m_normals; }
		const VertexNormals& getVertexNormals() const { return m_vertexNormals; }
		VertexNormals& getVertexNormals(){ return m_vertexNormals; }
		Edges& getEdges() { return m_edges; }
		const Edges& getEdges() const { return m_edges; }
		const FaceData& getFaceData() const { return m_faces; }
		const UVIndices& getUVIndices() const { return m_uvIndices; }
		const UVs& getUVs() const { return m_uvs; }
		const VerticesFaces& getVertexFaces() const { return m_verticesFaces; }
		const VerticesEdges& getVertexEdges() const { return m_verticesEdges; }


		unsigned int numVertices() const { return m_numPoints; }
		unsigned int numFaces() const { return (unsigned int)m_indices.size() / m_verticesPerFace; }
		unsigned int numEdges() const { return (unsigned int)m_edges.size(); }
		unsigned int numUVs() const { return (unsigned int)m_uvs.size(); }

		void copyUVs(const UVIndices& uvIndices, const UVs& uvs);

		void buildNeighbors();

		template<class PositionData>
		void updateNormals(const PositionData &pd, const unsigned int offset);

		template<class PositionData>
		void updateVertexNormals(const PositionData &pd);

		unsigned int getVerticesPerFace() const;
	};


	template<class PositionData>
	void IndexedFaceMesh::updateNormals(const PositionData &pd, const unsigned int offset)
	{
		m_normals.resize(numFaces());

		#pragma omp parallel default(shared)
		{
			#pragma omp for schedule(static)  
			for (int i = 0; i < (int) numFaces(); i++)
			{
				// Get first three points of face
				const Vector3r &a = pd.getPosition(m_indices[m_verticesPerFace*i] + offset);
				const Vector3r &b = pd.getPosition(m_indices[m_verticesPerFace*i + 1] + offset);
				const Vector3r &c = pd.getPosition(m_indices[m_verticesPerFace*i + 2] + offset);

				// Create normal
				Vector3r v1 = b - a;
				Vector3r v2 = c - a;

				m_normals[i] = v1.cross(v2);
				m_normals[i].normalize();
				// fix normals of degenerate triangles that can become zero vectors
				if (m_normals[i].squaredNorm() < 1e-6f)
					m_normals[i] = Vector3r::UnitX();
			}
		}
	}

	template<class PositionData>
	void IndexedFaceMesh::updateVertexNormals(const PositionData &pd)
	{
		m_vertexNormals.resize(numVertices());


		for (unsigned int i = 0; i < numVertices(); i++)
		{
			m_vertexNormals[i].setZero();
		}

		for (unsigned int i = 0u; i < numFaces(); i++)
		{
			const Vector3r &n = m_normals[i];
			m_vertexNormals[m_indices[m_verticesPerFace*i]] += n;
			m_vertexNormals[m_indices[m_verticesPerFace*i + 1]] += n;
			m_vertexNormals[m_indices[m_verticesPerFace*i + 2]] += n;
		}

		for (unsigned int i = 0; i < numVertices(); i++)
		{
			m_vertexNormals[i].normalize();
		}
	}

}

#endif
