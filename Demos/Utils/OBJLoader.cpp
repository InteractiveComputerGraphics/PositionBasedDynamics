#include "OBJLoader.h"
#include <vector>
#include <sstream>
#include <fstream>
#include <iostream>
#include "ObjectArray.h"
#include "StringTools.h"
#include "Logger.h"

using namespace PBD;
using namespace std;
 
void OBJLoader::loadObj(const std::string &filename, VertexData &vertexData, IndexedFaceMesh &mesh, const Vector3r &scale)
{
	LOG_INFO << "Loading " << filename;

	vector<Vector3r, Alloc_Vector3r> positions;
	vector<Vector2r, Alloc_Vector2r> texcoords;
	vector<Vector3r, Alloc_Vector3r> normals;
	ObjectArray<MeshFaceIndices> faces;
	
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

	std::vector<std::string> pos_buffer;
	std::vector<std::string> f_buffer;

	while(getline(filestream, line_stream))
	{
		stringstream str_stream(line_stream);
		string type_str;
		str_stream >> type_str;

		if(type_str == "v")
		{
			Vector3r pos;
			pos_buffer.clear();
			std::string parse_str = line_stream.substr(line_stream.find("v") + 1);
			StringTools::tokenize(parse_str, pos_buffer);
			for (unsigned int i = 0; i < 3; i++)
				pos[i] = stof(pos_buffer[i]) * scale[i];

			positions.push_back(pos);
		}
		else if(type_str == "vt")
		{
			Vector2r tex;
			pos_buffer.clear();
			std::string parse_str = line_stream.substr(line_stream.find("vt") + 2);
			StringTools::tokenize(parse_str, pos_buffer);
			for (unsigned int i = 0; i < 2; i++)
				tex[i] = stof(pos_buffer[i]);

			texcoords.push_back(tex);
			vt = true;
		}
		else if(type_str == "vn")
		{
			Vector3r nor;
			pos_buffer.clear();
			std::string parse_str = line_stream.substr(line_stream.find("vn") + 2);
			StringTools::tokenize(parse_str, pos_buffer);
			for (unsigned int i = 0; i < 3; i++)
				nor[i] = stof(pos_buffer[i]);

			normals.push_back(nor);
			vn = true;
		}
		else if(type_str == "f")
		{
			MeshFaceIndices faceIndex;
			if (vn && vt)
			{
				f_buffer.clear();
				std::string parse_str = line_stream.substr(line_stream.find("f") + 1);
				StringTools::tokenize(parse_str, f_buffer);
				for(int i = 0; i < 3; ++i)
				{
					pos_buffer.clear();
					StringTools::tokenize(f_buffer[i], pos_buffer, "/");
					faceIndex.posIndices[i] = stoi(pos_buffer[0]);
					faceIndex.texIndices[i] = stoi(pos_buffer[1]);
					faceIndex.normalIndices[i] = stoi(pos_buffer[2]);
				}
			}
			else if (vn)
			{
				f_buffer.clear();
				std::string parse_str = line_stream.substr(line_stream.find("f") + 1);
				StringTools::tokenize(parse_str, f_buffer);
				for(int i = 0; i < 3; ++i)
				{
					pos_buffer.clear();
					StringTools::tokenize(f_buffer[i], pos_buffer, "/");
					faceIndex.posIndices[i] = stoi(pos_buffer[0]);
					faceIndex.normalIndices[i] = stoi(pos_buffer[1]);
				}
			}
			else if (vt)
			{
				f_buffer.clear();
				std::string parse_str = line_stream.substr(line_stream.find("f") + 1);
				StringTools::tokenize(parse_str, f_buffer);
				for(int i = 0; i < 3; ++i)
				{
					pos_buffer.clear();
					StringTools::tokenize(f_buffer[i], pos_buffer, "/");
					faceIndex.posIndices[i] = stoi(pos_buffer[0]);
					faceIndex.texIndices[i] = stoi(pos_buffer[1]);
				}
			}
			else 
			{
				f_buffer.clear();
				std::string parse_str = line_stream.substr(line_stream.find("f") + 1);
				StringTools::tokenize(parse_str, f_buffer);
				for (int i = 0; i < 3; ++i)
				{
					faceIndex.posIndices[i] = stoi(f_buffer[i]);
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

	mesh.updateNormals(vertexData, 0);
	mesh.updateVertexNormals(vertexData);

	LOG_INFO << "Number of triangles: " << nFaces;
	LOG_INFO << "Number of vertices: " << nPoints;
}