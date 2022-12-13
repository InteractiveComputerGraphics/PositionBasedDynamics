#include "IndexedFaceMesh.h"

using namespace Utilities;

IndexedFaceMesh& IndexedFaceMesh::operator=(IndexedFaceMesh const& other)
{
	m_numPoints		   = other.m_numPoints;
    m_indices          = other.m_indices;
    m_edges            = other.m_edges;
	m_facesEdges       = other.m_facesEdges;
    m_closed           = other.m_closed;
	m_uvIndices = other.m_uvIndices;
	m_uvs = other.m_uvs;
    m_normals          = other.m_normals;
    m_vertexNormals    = other.m_vertexNormals;

    for (size_t i(0u); i < m_facesEdges.size(); ++i)
    {
        m_facesEdges[i].resize(m_verticesPerFace);
#if defined(_MSC_VER)
        std::copy(other.m_facesEdges[i].data(), other.m_facesEdges[i].data() + m_verticesPerFace,
            stdext::unchecked_array_iterator<unsigned int*>(m_facesEdges[i].data()));
#else
        std::copy(other.m_facesEdges[i].data(), other.m_facesEdges[i].data() + m_verticesPerFace, m_facesEdges[i].data());
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

IndexedFaceMesh::IndexedFaceMesh(IndexedFaceMesh const& other)
{
    *this = other;
}

IndexedFaceMesh::IndexedFaceMesh()
{
	m_closed=false;
	m_flatShading = false;
	m_numPoints = 0;
}

IndexedFaceMesh::~IndexedFaceMesh()
{
	release();
}

bool IndexedFaceMesh::isClosed() const
{
	return m_closed;
}

void IndexedFaceMesh::initMesh(const unsigned int nPoints, const unsigned int nEdges, const unsigned int nFaces)
{
	m_numPoints = nPoints;
	m_indices.reserve(nFaces*m_verticesPerFace);
	m_edges.reserve(nEdges);
	m_facesEdges.reserve(nFaces);
	m_uvIndices.reserve(nFaces);
	m_uvs.reserve(nPoints);
	m_verticesFaces.reserve(nPoints);
	m_verticesEdges.reserve(nPoints);
	m_normals.reserve(nFaces);
	m_vertexNormals.reserve(nPoints);
}

void IndexedFaceMesh::release()
{
	m_indices.clear();
	m_edges.clear();
	m_facesEdges.clear();
	m_facesEdges.clear();
	m_uvIndices.clear();
	m_uvs.clear();
	m_verticesFaces.clear();
	m_verticesEdges.clear();
	m_normals.clear();
	m_vertexNormals.clear();
}

/** Add a new face. Indices must be an array of size m_verticesPerFace.
	*/
void IndexedFaceMesh::addFace(const unsigned int * const indices)
{
	for (unsigned int i=0u; i < m_verticesPerFace; i++)
		m_indices.push_back(indices[i]);
}

/** Add a new face. Indices must be an array of size m_verticesPerFace.
	*/
void IndexedFaceMesh::addFace(const int * const indices)
{
	for (unsigned int i=0u; i < m_verticesPerFace; i++)
		m_indices.push_back((unsigned int) indices[i]);
}

void IndexedFaceMesh::addUV(const Real u, const Real v)
{
	Vector2r uv;
	uv[0] = u;
	uv[1] = v;
	m_uvs.push_back(uv);
}
	
void IndexedFaceMesh::addUVIndex(const unsigned int index)
{
	m_uvIndices.push_back(index);
}
	
void IndexedFaceMesh::buildNeighbors()
{
	typedef std::vector<unsigned int> PEdges;

	PEdges* pEdges = new PEdges[numVertices()];

	// build vertex-face structure
	m_verticesFaces.clear(); // to delete old pointers
	m_verticesFaces.resize(numVertices());
	m_verticesEdges.clear(); // to delete old pointers
	m_verticesEdges.resize(numVertices());
	m_facesEdges.clear();
	m_facesEdges.resize(numFaces());

	m_edges.clear();

	unsigned int *v = new unsigned int[m_verticesPerFace];
	unsigned int *edges = new unsigned int[m_verticesPerFace*2];
	for(unsigned int i=0; i < numFaces(); i++)
	{		
		m_facesEdges[i].resize(m_verticesPerFace);
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
			for(unsigned int k=0; k < m_verticesFaces[vIndex].size(); k++)
			{
				if (m_verticesFaces[vIndex][k] == i)
				{
					found = true;
					break;
				}
			}
			if (!found)
			{
				m_verticesFaces[vIndex].push_back(i);
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
				m_verticesEdges[a].push_back(edge);
				m_verticesEdges[b].push_back(edge);
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
			m_facesEdges[i][j] = edge;
		}			
	}
	delete [] v;
	delete [] edges;

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
}
	
void IndexedFaceMesh::copyUVs(const UVIndices& uvIndices, const UVs& uvs)
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

unsigned int IndexedFaceMesh::getVerticesPerFace() const
{
	return m_verticesPerFace;
}
	
