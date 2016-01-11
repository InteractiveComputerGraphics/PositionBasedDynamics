#include "OBJLoader.h"
#include <vector>
#include <sstream>
#include <fstream>
#include <iostream>

using namespace PBD;
using namespace std;
 
struct MeshFaceIndices
{
    int posIndices[3];
    int texIndices[3];
    int normalIndices[3];
};

 
void OBJLoader::loadObj(const std::string &filename, VertexData &vertexData, IndexedFaceMesh &mesh, const Eigen::Vector3f &scale)
{
	std::cout << "Loading " << filename << std::endl;

	vector<Eigen::Vector3f> positions;
	vector<Eigen::Vector2f> texcoords;
	vector<Eigen::Vector3f> normals;
	vector<MeshFaceIndices> faces;
    
    ifstream filestream;
    filestream.open(filename.c_str());
	if (filestream.fail())
	{
		std::cerr << "Failed to open file: " << filename << "\n";
		return;
	}

    string line_stream; 
	bool vt = false;
	bool vn = false;

    while(getline(filestream, line_stream))
	{
        stringstream str_stream(line_stream);
        string type_str;
        str_stream >> type_str;

        if(type_str == "v")
		{
			Eigen::Vector3f pos;
			str_stream >> pos[0] >> pos[1] >> pos[2];
			for (unsigned int i = 0; i < 3; i++)
				pos[i] = pos[i] * scale[i];

			positions.push_back(pos);
        }
		else if(type_str == "vt")
		{
			Eigen::Vector2f tex;
			str_stream >> tex[0] >> tex[1];
			texcoords.push_back(tex);
			vt = true;
        }
		else if(type_str == "vn")
		{
			Eigen::Vector3f nor;
			str_stream >> nor[0] >> nor[1] >> nor[2];
			normals.push_back(nor);
			vn = true;
        }
		else if(type_str == "f")
		{
            MeshFaceIndices faceIndex;
            char interupt;
			if (vn && vt)
			{
				for(int i = 0; i < 3; ++i)
				{
					str_stream >> faceIndex.posIndices[i] >> interupt
						>> faceIndex.texIndices[i]  >> interupt
						>> faceIndex.normalIndices[i];
				}
			}
			else if (vn)
			{
				for(int i = 0; i < 3; ++i)
				{
					str_stream >> faceIndex.posIndices[i] >> interupt
						>> interupt  
						>> faceIndex.normalIndices[i];
				}
			}
			else if (vt)
			{
				for(int i = 0; i < 3; ++i)
				{
					str_stream >> faceIndex.posIndices[i] >> interupt						
						>> faceIndex.texIndices[i];
				}
			}
			else 
			{
				for (int i = 0; i < 3; ++i)
				{
					str_stream >> faceIndex.posIndices[i];
				}
			}
            faces.push_back(faceIndex);
        }
    }
    filestream.close();
	mesh.release();
	vertexData.release();
	const unsigned int nPoints = (unsigned int) positions.size();
	const unsigned int nFaces = (unsigned int) faces.size();
	const unsigned int nTexCoords = (unsigned int) texcoords.size();
	mesh.initMesh(nPoints, nFaces * 2, nFaces);
	vertexData.reserve(nPoints);
	for(unsigned int i=0; i < nPoints; i++)
	{
		vertexData.addVertex(positions[i]);
	}
	for(unsigned int i=0; i < nTexCoords; i++)
	{
		mesh.addUV(texcoords[i][0], texcoords[i][1]); 
	}
	for(unsigned int i=0; i < nFaces; i++)
	{
		// Reduce the indices by one
		int posIndices[3];
		int texIndices[3];
		for (int j=0; j < 3; j++)
		{
			posIndices[j] = faces[i].posIndices[j]-1;
            if (nTexCoords > 0)
            {
                texIndices[j] = faces[i].texIndices[j] - 1;
                mesh.addUVIndex(texIndices[j]);
            }
		}

		mesh.addFace(&posIndices[0]);
	}
	mesh.buildNeighbors();
}