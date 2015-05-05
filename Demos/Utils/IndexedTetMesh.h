#ifndef __INDEXEDTETMESH_H__
#define __INDEXEDTETMESH_H__

#include <vector>
#include <Eigen/Dense>

namespace PBD 
{
	template <class VertexData>
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
		VertexData m_vertexData;
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
		void initMemory(const unsigned int nPoints, const unsigned int nEdges, const unsigned int nFaces, const unsigned int nTets);
		void addTet(const unsigned int * const indices);
		void addTet(const int * const indices);
		void createVertex();
		void addVertex(const Eigen::Vector3f &vertex);
		const VertexData& getVertexData() const;
		VertexData& getVertexData();

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


		unsigned int numVertices() const {return m_vertexData.size();}
		unsigned int numFaces() const { return (unsigned int)m_faceIndices.size() / 3; }
		unsigned int numTets() const { return (unsigned int)m_tetIndices.size() / 4; }
		unsigned int numEdges() const { return (unsigned int)m_edges.size(); }

		void buildNeighbors();
	};

	template <class VertexData>
	IndexedTetMesh<VertexData>::IndexedTetMesh()
	{
	}

	template <class VertexData>
	IndexedTetMesh<VertexData>::~IndexedTetMesh()
	{
		release();
	}

	template <class VertexData>
	void IndexedTetMesh<VertexData>::createVertex()
	{
		m_vertexData.createVertex();
	}

	template <class VertexData>
	void IndexedTetMesh<VertexData>::addVertex(const Eigen::Vector3f& vertex)
	{
		m_vertexData.addVertex(vertex);
	}

	template <class VertexData>
	VertexData& IndexedTetMesh<VertexData>::getVertexData() 
	{
		return m_vertexData;
	}

	template <class VertexData>
	const VertexData& IndexedTetMesh<VertexData>::getVertexData() const 
	{
		return m_vertexData;
	}

	template <class VertexData>
	void IndexedTetMesh<VertexData>::initMemory(const unsigned int nPoints, const unsigned int nEdges, const unsigned int nFaces, const unsigned int nTets)
	{
		m_vertexData.reserve(nPoints);
		m_faceIndices.reserve(nFaces*3);
		m_tetIndices.reserve(nTets*4);
		m_edges.reserve(nEdges);
		m_faces.reserve(nFaces);
		m_tets.reserve(nTets);
		m_verticesTets.reserve(nPoints);
		m_verticesFaces.reserve(nPoints);
		m_verticesEdges.reserve(nPoints);
	}


	template <class VertexData>
	void IndexedTetMesh<VertexData>::release()
	{
		m_vertexData.release();
		m_faceIndices.clear();
		m_tetIndices.clear();
		m_edges.clear();
		m_tets.clear();
		m_faces.clear();
		m_verticesTets.clear();
		m_verticesFaces.clear();
		m_verticesEdges.clear();
	}

	/** Add a new tet. Indices must be an array of size 4.
	 */
	template <class VertexData>
	void IndexedTetMesh<VertexData>::addTet(const unsigned int * const indices)
	{
		for (unsigned int i=0u; i < 4; i++)
			m_tetIndices.push_back(indices[i]);
	}

	/** Add a new tet. Indices must be an array of size 4.
	 */
	template <class VertexData>
	void IndexedTetMesh<VertexData>::addTet(const int * const indices)
	{
		for (unsigned int i=0u; i < 4; i++)
			m_tetIndices.push_back((unsigned int) indices[i]);
	}

	template <class VertexData>
	void IndexedTetMesh<VertexData>::buildNeighbors()
	{
		typedef std::vector<unsigned int> VertexEdges;
		typedef std::vector<unsigned int> VertexFaces;
		typedef std::vector<unsigned int> VertexTets;

		VertexEdges* vEdges = new VertexEdges[numVertices()];
		VertexFaces* vFaces = new VertexFaces[numVertices()];
		VertexTets* vTets = new VertexTets[numVertices()];

		m_faces.clear();
		m_edges.clear();
		m_tets.resize(numTets());

		for(unsigned int i=0; i < numTets(); i++)
		{	
			// tet edge indices: {0,1, 0,2, 0,3, 1,2, 1,3, 2,3}
			const unsigned int edges[12] = {	m_tetIndices[4*i], m_tetIndices[4*i+1], 
												m_tetIndices[4*i], m_tetIndices[4*i+2], 
												m_tetIndices[4*i], m_tetIndices[4*i+3], 
												m_tetIndices[4*i+1], m_tetIndices[4*i+2], 
												m_tetIndices[4*i+1], m_tetIndices[4*i+3], 
												m_tetIndices[4*i+2], m_tetIndices[4*i+3]};
			
			// tet face indices: {0,1,2, 1,3,2, 3,0,2, 1,0,3} => clock wise
			/*const unsigned int faces[12] = {	m_tetIndices[4*i], m_tetIndices[4*i+1], m_tetIndices[4*i+2], 
												m_tetIndices[4*i+1], m_tetIndices[4*i+3], m_tetIndices[4*i+2], 
												m_tetIndices[4*i+3], m_tetIndices[4*i], m_tetIndices[4*i+2], 
												m_tetIndices[4*i+1], m_tetIndices[4*i], m_tetIndices[4*i+3]};*/

			// tet face indices: {1,0,2, 3,1,2, 0,3,2, 0,1,3} => counter clock wise
			const unsigned int faces[12] = {	m_tetIndices[4*i+1], m_tetIndices[4*i], m_tetIndices[4*i+2], 
												m_tetIndices[4*i+3], m_tetIndices[4*i+1], m_tetIndices[4*i+2], 
												m_tetIndices[4*i], m_tetIndices[4*i+3], m_tetIndices[4*i+2], 
												m_tetIndices[4*i], m_tetIndices[4*i+1], m_tetIndices[4*i+3]};

			for(unsigned int j=0u; j < 4; j++)
			{
				// add vertex-tet connection
				const unsigned int vIndex = m_tetIndices[4*i+j];
				vTets[vIndex].push_back(i);
			}

			for(unsigned int j=0u; j < 4; j++)
			{
				// add face information
				const unsigned int a = faces[j*3+0];
				const unsigned int b = faces[j*3+1];
				const unsigned int c = faces[j*3+2];
				unsigned int face = 0xffffffff;
				// find face
				for(unsigned int k=0; k < vFaces[a].size(); k++)
				{
					// Check if we already have this face in the list
					const unsigned int& faceIndex = vFaces[a][k];
					if(((m_faceIndices[3*faceIndex] == a) || (m_faceIndices[3*faceIndex] == b) || (m_faceIndices[3*faceIndex] == c)) &&
						((m_faceIndices[3*faceIndex+1] == a) || (m_faceIndices[3*faceIndex+1] == b) || (m_faceIndices[3*faceIndex+1] == c)) &&
						((m_faceIndices[3*faceIndex+2] == a) || (m_faceIndices[3*faceIndex+2] == b) || (m_faceIndices[3*faceIndex+2] == c)))
					{
						face = vFaces[a][k];
						break;
					}
				}
				if(face == 0xffffffff)
				{
					// create new
					Face f;
					m_faceIndices.push_back(a);
					m_faceIndices.push_back(b);
					m_faceIndices.push_back(c);
					face = (unsigned int) m_faceIndices.size()/3 - 1u;
					f.m_tets[0] = i;
					f.m_tets[1] = 0xffffffff;
					m_faces.push_back(f);

					// add vertex-face connection				
					vFaces[a].push_back(face);
					vFaces[b].push_back(face);
					vFaces[c].push_back(face);
				}
				else
				{
					Face &fd = m_faces[face];
					fd.m_tets[1] = i;
				}
				// append face
				m_tets[i].m_faces[j] = face;
			}

			for(unsigned int j=0u; j < 6; j++)
			{
				// add face information
				const unsigned int a = edges[j*2+0];
				const unsigned int b = edges[j*2+1];
				unsigned int edge = 0xffffffff;
				// find edge
				for(unsigned int k=0; k < vEdges[a].size(); k++)
				{
					// Check if we already have this edge in the list
					const Edge& e = m_edges[vEdges[a][k]];
					if(((e.m_vert[0] == a) || (e.m_vert[0] == b)) &&
						((e.m_vert[1] == a) || (e.m_vert[1] == b)))
					{
						edge = vEdges[a][k];
						break;
					}
				}
				if(edge == 0xffffffff)
				{
					// create new
					Edge e;
					e.m_vert[0] = a;
					e.m_vert[1] = b;
					m_edges.push_back(e);
					edge = (unsigned int) m_edges.size() - 1u;					

					// add vertex-edge connection				
					vEdges[a].push_back(edge);
					vEdges[b].push_back(edge);
				}
				// append edge
				m_tets[i].m_edges[j] = edge;
			}
		}

		m_verticesEdges.clear(); // to delete old pointers
		m_verticesEdges.resize(numVertices());
		m_verticesFaces.clear(); // to delete old pointers
		m_verticesFaces.resize(numVertices());
		m_verticesTets.clear(); // to delete old pointers
		m_verticesTets.resize(numVertices());
		for(unsigned int i=0; i < numVertices(); i++)
		{
			m_verticesEdges[i].m_numEdges = (unsigned int) vEdges[i].size();
			m_verticesEdges[i].m_eIndices = new unsigned int[m_verticesEdges[i].m_numEdges];
			memcpy(m_verticesEdges[i].m_eIndices, vEdges[i].data(), sizeof(unsigned int)*m_verticesEdges[i].m_numEdges);

			m_verticesFaces[i].m_numFaces = (unsigned int) vFaces[i].size();
			m_verticesFaces[i].m_fIndices = new unsigned int[m_verticesFaces[i].m_numFaces];
			memcpy(m_verticesFaces[i].m_fIndices, vFaces[i].data(), sizeof(unsigned int)*m_verticesFaces[i].m_numFaces);

			m_verticesTets[i].m_numTets = (unsigned int) vTets[i].size();
			m_verticesTets[i].m_tIndices = new unsigned int[m_verticesTets[i].m_numTets];
			memcpy(m_verticesTets[i].m_tIndices, vTets[i].data(), sizeof(unsigned int)*m_verticesTets[i].m_numTets);
		}

		delete [] vEdges;
		delete [] vFaces;
		delete [] vTets;

	}
}

#endif
