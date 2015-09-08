#ifndef __INDEXEDFACEMESH_H__
#define __INDEXEDFACEMESH_H__

#include <vector>
#include <Eigen/Dense>
#include <iterator>

namespace PBD 
{
	template <class VertexData>
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
#if defined(WIN32) || defined(_WIN32) || defined(WIN64)		
                std::copy(other.m_fIndices, other.m_fIndices + m_numFaces,
                    stdext::unchecked_array_iterator<unsigned int*>(m_fIndices));
#else
                std::copy(other.m_fIndices, other.m_fIndices + m_numFaces, m_fIndices);
#endif		
                return *this;
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

            VertexEdges(VertexEdges const& other)
            {
                *this = other;
            }

            VertexEdges& operator=(VertexEdges const& other)
            {
                m_numEdges = other.m_numEdges;
                m_eIndices = new unsigned int[m_numEdges];
#if defined(WIN32) || defined(_WIN32) || defined(WIN64)		
                std::copy(other.m_eIndices, other.m_eIndices + m_numEdges,
                    stdext::unchecked_array_iterator<unsigned int*>(m_eIndices));
#else
                std::copy(other.m_eIndices, other.m_eIndices + m_numEdges, m_eIndices);
#endif		
                return *this;
            }

			~VertexEdges()
			{
				delete [] m_eIndices;
			}

			unsigned int m_numEdges;
			unsigned int* m_eIndices;
		};

	public:
		typedef std::vector<unsigned int> Faces;
		typedef std::vector<Eigen::Vector3f> FaceNormals;
		typedef std::vector<Eigen::Vector3f> VertexNormals;
		typedef std::vector<Face> FaceData;
		typedef std::vector<Edge> Edges;
		typedef std::vector<VertexFaces> VerticesFaces;
		typedef std::vector<VertexEdges> VerticesEdges;
		typedef std::vector<unsigned int> UVIndices;
		typedef std::vector<Eigen::Vector2f> UVs;

	protected:
		VertexData m_vertexData;
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
		void initMemory(const unsigned int nPoints, const unsigned int nEdges, const unsigned int nFaces);
		void addEdge(const unsigned int * const indices);
		void addEdge(const int * const indices);
		void addFace(const unsigned int * const indices);
		void addFace(const int * const indices);
		void addUV(const float u, const float v);
		void addUVIndex(const unsigned int index);
		void createVertex();
		void addVertex(const Eigen::Vector3f &vertex);
		const VertexData& getVertexData() const;
		VertexData& getVertexData();

		const Faces& getFaces() const {return m_indices;}
		Faces& getFaces(){return m_indices;}
		const FaceNormals& getFaceNormals() const {return m_normals;}
		FaceNormals& getFaceNormals(){return m_normals;}
		const VertexNormals& getVertexNormals() const {return m_vertexNormals;}
		VertexNormals& getVertexNormals(){return m_vertexNormals;}
		Edges& getEdges() {return m_edges;}
		const Edges& getEdges() const {return m_edges;}
		const FaceData& getFaceData() const {return m_faces;}
		const UVIndices& getUVIndices() const { return m_uvIndices; }
		const UVs& getUVs() const { return m_uvs; }
		const VerticesFaces& getVertexFaces() const {return m_verticesFaces;}
		const VerticesEdges& getVertexEdges() const {return m_verticesEdges;}


		unsigned int numVertices() const {return m_vertexData.size();}
		unsigned int numFaces() const {return (unsigned int) m_indices.size() / m_verticesPerFace;}
		unsigned int numEdges() const {return (unsigned int) m_edges.size(); }
		unsigned int numUVs() const { return m_uvs.size(); }

		void copyUVs(const UVIndices& uvIndices, const UVs& uvs);

		void buildNeighbors();

		void updateNormals();
		template<class PositionData>
		void updateNormals(const PositionData &pd);

		void updateVertexNormals();
		template<class PositionData>
		void updateVertexNormals(const PositionData &pd);

		unsigned int getVerticesPerFace() const;
	};

    template <class VertexData> IndexedFaceMesh<VertexData>&
    IndexedFaceMesh<VertexData>::operator=(IndexedFaceMesh const& other)
    {
        m_vertexData       = other.m_vertexData;
        m_indices          = other.m_indices;
        m_edges            = other.m_edges;
        m_faces            = other.m_faces;
        m_closed           = other.m_closed;
		m_uvIndices = other.m_uvIndices;
		m_uvs = other.m_uvs;
        m_verticesPerFace  = other.m_verticesPerFace;
        m_normals          = other.m_normals;
        m_vertexNormals    = other.m_vertexNormals;

        for (size_t i(0u); i < m_faces.size(); ++i)
        {
            m_faces[i].m_edges = new unsigned int[m_verticesPerFace];
#if defined(WIN32) || defined(_WIN32) || defined(WIN64)	    
            std::copy(other.m_faces[i].m_edges, other.m_faces[i].m_edges + m_verticesPerFace,
                stdext::unchecked_array_iterator<unsigned int*>(m_faces[i].m_edges));
#else
            std::copy(other.m_faces[i].m_edges, other.m_faces[i].m_edges + m_verticesPerFace, m_faces[i].m_edges);
#endif	    
        }

        m_verticesEdges.resize(other.m_verticesEdges.size());
        for (size_t i(0u); i < m_verticesEdges.size(); ++i)
            m_verticesEdges[i] = other.m_verticesEdges[i];

        m_verticesFaces.resize(other.m_verticesFaces.size());
        for (size_t i(0u); i < m_verticesFaces.size(); ++i)
            m_verticesFaces[i] = other.m_verticesFaces[i];

        return *this;
    }

    template <class VertexData>
    IndexedFaceMesh<VertexData>::IndexedFaceMesh(IndexedFaceMesh const& other)
    {
        *this = other;
    }

	template <class VertexData>
	IndexedFaceMesh<VertexData>::IndexedFaceMesh(const unsigned int verticesPerFace)
	{
		m_verticesPerFace = verticesPerFace;
		m_closed=false;
	}

	template <class VertexData>
	IndexedFaceMesh<VertexData>::~IndexedFaceMesh()
	{
		release();
	}

	template <class VertexData>
	bool IndexedFaceMesh<VertexData>::isClosed() const
	{
		return m_closed;
	}

	template <class VertexData>
	void IndexedFaceMesh<VertexData>::createVertex()
	{
		m_vertexData.createVertex();
	}

	template <class VertexData>
	VertexData& IndexedFaceMesh<VertexData>::getVertexData() 
	{
		return m_vertexData;
	}

	template <class VertexData>
	const VertexData& IndexedFaceMesh<VertexData>::getVertexData() const 
	{
		return m_vertexData;
	}

	template <class VertexData>
	void IndexedFaceMesh<VertexData>::initMemory(const unsigned int nPoints, const unsigned int nEdges, const unsigned int nFaces)
	{
		m_vertexData.reserve(nPoints);
		m_indices.reserve(nFaces*m_verticesPerFace);
		m_edges.reserve(nEdges);
		m_faces.reserve(nFaces);
		m_uvIndices.reserve(nFaces);
		m_uvs.reserve(nPoints);
		m_verticesFaces.reserve(nPoints);
		m_verticesEdges.reserve(nPoints);
		m_normals.reserve(nFaces);
		m_vertexNormals.reserve(nPoints);
	}


	template <class VertexData>
	void IndexedFaceMesh<VertexData>::release()
	{
		m_vertexData.release();
		m_indices.clear();
		m_edges.clear();
		for(unsigned int i=0; i < m_faces.size(); i++)
			delete [] m_faces[i].m_edges;
		m_faces.clear();
		m_uvIndices.clear();
		m_uvs.clear();
		m_verticesFaces.clear();
		m_verticesEdges.clear();
		m_normals.clear();
		m_vertexNormals.clear();
	}

	/** Add a new vertex. 
	*/
	template <class VertexData>
	void IndexedFaceMesh<VertexData>::addVertex(const Eigen::Vector3f &vertex)
	{
		m_vertexData.addVertex(vertex);
	}

	/** Add a new face. Indices must be an array of size m_verticesPerFace.
	 */
	template <class VertexData>
	void IndexedFaceMesh<VertexData>::addFace(const unsigned int * const indices)
	{
		for (unsigned int i=0u; i < m_verticesPerFace; i++)
			m_indices.push_back(indices[i]);
	}

	/** Add a new face. Indices must be an array of size m_verticesPerFace.
	 */
	template <class VertexData>
	void IndexedFaceMesh<VertexData>::addFace(const int * const indices)
	{
		for (unsigned int i=0u; i < m_verticesPerFace; i++)
			m_indices.push_back((unsigned int) indices[i]);
	}

	/** Add a new edge. Indices must be an array of size m_verticesPerFace.
	 */
	template <class VertexData>
	void IndexedFaceMesh<VertexData>::addEdge(const unsigned int * const indices)
	{
		Edge& e = m_edges.create();
		e.m_vert[0] = indices[0];
		e.m_vert[1] = indices[1];
		e.m_face[0] = 0xffffffff;
		e.m_face[1] = 0xffffffff;
	}

	/** Add a new edge. Indices must be an array of size m_verticesPerFace.
	 */
	template <class VertexData>
	void IndexedFaceMesh<VertexData>::addEdge(const int * const indices)
	{
		Edge& e = m_edges.create();
		e.m_vert[0] = indices[0];
		e.m_vert[1] = indices[1];
		e.m_face[0] = 0xffffffff;
		e.m_face[1] = 0xffffffff;
	}

	template <class VertexData>
	void IndexedFaceMesh<VertexData>::addUV(const float u, const float v)
	{
		Eigen::Vector2f uv;
		uv[0] = u;
		uv[1] = v;
		m_uvs.push_back(uv);
	}

	template <class VertexData>
	void IndexedFaceMesh<VertexData>::addUVIndex(const unsigned int index)
	{
		m_uvIndices.push_back(index);
	}

	template <class VertexData>
	void IndexedFaceMesh<VertexData>::buildNeighbors()
	{
		typedef std::vector<unsigned int> PEdges;
		typedef std::vector<unsigned int> VertexFE;

		PEdges* pEdges = new PEdges[numVertices()];
		VertexFE* vFaces = new VertexFE[numVertices()];
		VertexFE* vEdges = new VertexFE[numVertices()];

		for(unsigned int i=0; i < m_faces.size(); i++)
			delete [] m_faces[i].m_edges;
		m_edges.clear();
		m_faces.resize(numFaces());

		unsigned int *v = new unsigned int[m_verticesPerFace];
		unsigned int *edges = new unsigned int[m_verticesPerFace*2];
		for(unsigned int i=0; i < numFaces(); i++)
		{		
			m_faces[i].m_edges = new unsigned int[m_verticesPerFace];
			for (unsigned int j=0u; j < m_verticesPerFace; j++)
				v[j] = m_indices[m_verticesPerFace*i+j];
			
			for (unsigned int j=0u; j < m_verticesPerFace-1u; j++)
			{
				edges[2*j] = v[j];
				edges[2*j+1] = v[j+1];
			}
			edges[2*(m_verticesPerFace-1)] = v[m_verticesPerFace-1];
			edges[2*(m_verticesPerFace-1)+1] = v[0];

			for(unsigned int j=0u; j < m_verticesPerFace; j++)
			{
				// add vertex-face connection
				const unsigned int vIndex = m_indices[m_verticesPerFace*i+j];
				bool found = false;
				for(unsigned int k=0; k < vFaces[vIndex].size(); k++)
				{
					if (vFaces[vIndex][k] == i)
					{
						found = true;
						break;
					}
				}
				if (!found)
				{
					vFaces[vIndex].push_back(i);
				}

				// add edge information
				const unsigned int a = edges[j*2+0];
				const unsigned int b = edges[j*2+1];
				unsigned int edge = 0xffffffff;
				// find edge
				for(unsigned int k=0; k < pEdges[a].size(); k++)
				{
					const Edge& e = m_edges[pEdges[a][k]];
					if(((e.m_vert[0] == a) || (e.m_vert[0] == b)) &&
						((e.m_vert[1] == a) || (e.m_vert[1] == b)))
					{
						edge = pEdges[a][k];
						break;
					}
				}
				if(edge == 0xffffffff)
				{
					// create new
					Edge e;
					e.m_vert[0] = a;
					e.m_vert[1] = b;
					e.m_face[0] = i;
					e.m_face[1] = 0xffffffff;
					m_edges.push_back(e);
					edge = (unsigned int) m_edges.size() - 1u;

					// add vertex-edge connection				
					vEdges[a].push_back(edge);
					vEdges[b].push_back(edge);
				}
				else
				{
					Edge& e = m_edges[edge];
					e.m_face[1] = i;
				}
				// append to points
				pEdges[a].push_back(edge);
				pEdges[b].push_back(edge);
				// append face
				m_faces[i].m_edges[j] = edge;
			}			
		}
		delete [] v;
		delete [] edges;

		// build vertex-face structure
		m_verticesFaces.clear(); // to delete old pointers
		m_verticesFaces.resize(numVertices());
		m_verticesEdges.clear(); // to delete old pointers
		m_verticesEdges.resize(numVertices());
		for(unsigned int i=0; i < numVertices(); i++)
		{
			m_verticesFaces[i].m_numFaces = (unsigned int) vFaces[i].size();
			m_verticesFaces[i].m_fIndices = new unsigned int[m_verticesFaces[i].m_numFaces];
			memcpy(m_verticesFaces[i].m_fIndices, vFaces[i].data(), sizeof(unsigned int)*m_verticesFaces[i].m_numFaces);

			m_verticesEdges[i].m_numEdges = (unsigned int) vEdges[i].size();
			m_verticesEdges[i].m_eIndices = new unsigned int[m_verticesEdges[i].m_numEdges];
			memcpy(m_verticesEdges[i].m_eIndices, vEdges[i].data(), sizeof(unsigned int)*m_verticesEdges[i].m_numEdges);
		}

		// check for boundary
		m_closed = true;
		for (unsigned int i = 0; i < (unsigned int)m_edges.size(); i++)
		{
			Edge& e = m_edges[i];
			if(e.m_face[1] == 0xffffffff)
			{
				m_closed = false;
				break;
			}
		}

		delete [] pEdges;
		delete [] vFaces;
		delete [] vEdges;
	}

	template <class VertexData>
	void IndexedFaceMesh<VertexData>::copyUVs(const UVIndices& uvIndices, const UVs& uvs)
	{
		m_uvs.clear();
		m_uvs.resize(uvs.size());

		for (unsigned int i = 0; i < uvs.size(); i++)
		{
			m_uvs[i] = uvs[i];
		}

		m_uvIndices.clear();
		m_uvIndices.resize(uvIndices.size());

		for (unsigned int i = 0; i < uvIndices.size(); i++)
		{
			m_uvIndices[i] = uvIndices[i];
		}
	}

	template <class VertexData>
	unsigned int IndexedFaceMesh<VertexData>::getVerticesPerFace() const
	{
		return m_verticesPerFace;
	}

	template <class VertexData>
	void IndexedFaceMesh<VertexData>::updateNormals()
	{
		updateNormals(getVertexData());
	}

	template <class VertexData>
	template<class PositionData>
	void IndexedFaceMesh<VertexData>::updateNormals(const PositionData &pd)
	{
		m_normals.resize(numFaces());
		for (unsigned int i=0u; i < numFaces(); i++)
		{
			// Get first three points of face
			const Eigen::Vector3f &a = pd.getPosition(m_indices[m_verticesPerFace*i]);
			const Eigen::Vector3f &b = pd.getPosition(m_indices[m_verticesPerFace*i+1]);
			const Eigen::Vector3f &c = pd.getPosition(m_indices[m_verticesPerFace*i+2]);

			// Create normal
			Eigen::Vector3f v1 = b - a;
			Eigen::Vector3f v2 = c - a;

			m_normals[i] = v1.cross(v2);
			m_normals[i].normalize();
		}
	}

	template <class VertexData>
	void IndexedFaceMesh<VertexData>::updateVertexNormals()
	{
		updateVertexNormals(getVertexData());
	}

	template <class VertexData>
	template<class PositionData>
	void IndexedFaceMesh<VertexData>::updateVertexNormals(const PositionData &pd)
	{
		m_vertexNormals.resize(numVertices());


		for (unsigned int i=0; i < numVertices(); i++)
		{
			m_vertexNormals[i].setZero();
		}	

		for (unsigned int i=0u; i < numFaces(); i++)
		{
			const Eigen::Vector3f &n = m_normals[i];
			m_vertexNormals[m_indices[m_verticesPerFace*i]] += n;
			m_vertexNormals[m_indices[m_verticesPerFace*i+1]] += n;
			m_vertexNormals[m_indices[m_verticesPerFace*i+2]] += n;
		}

		for (unsigned int i=0; i < numVertices(); i++)
		{
			m_vertexNormals[i].normalize();
		}
	}
}

#endif
