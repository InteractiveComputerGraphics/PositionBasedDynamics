#include "TetGenLoader.h"
#include <sstream>
#include <fstream>
#include <iostream>
#include "Logger.h"

using namespace Utilities;
using namespace std;

// Call this function to load a model from a *.tet file
void TetGenLoader::loadTetFile(const std::string &filename, std::vector<Vector3r>& vertices, std::vector<unsigned int>& tets)
{
	LOG_INFO << "Loading " << filename;

	// variables
	size_t i, num_materials, num_vertices, num_tetras, num_triangles;
	Real value;
	string line, label;
	stringstream sStream;
	// try to open the file
	ifstream fin(filename.c_str());
	if(!fin)
	{
		LOG_ERR << "'" + filename + "' file not found.";
		return;
	}

	// load tet version 1.2
	getline(fin, line);
	sStream << line;
	sStream >> label; // tet
	sStream >> label; // version
	sStream >> value;
	sStream.clear();
	// load number of materials
	getline(fin, line); // num_materials x
	sStream << line;
	sStream >> label;
	sStream >> num_materials;
	sStream.clear();
	// load number of vertices
	getline(fin, line); // num_vertices x
	sStream << line;
	sStream >> label;
	sStream >> num_vertices;
	sStream.clear();
	// reverse the order of the vertices
	vertices.resize(num_vertices);
	//// read number of tetraeders
	getline(fin, line); // num_tetras x
	sStream << line;
	sStream >> label;
	sStream >> num_tetras;
	sStream.clear();
	tets.resize(4*num_tetras);

	// read number of triangles
	getline(fin, line); // num_triangles x
	sStream << line;
	sStream >> label;
	sStream >> num_triangles;
	sStream.clear();

	// skip materials
	getline(fin, line);
	for(i = 0; i < num_materials; ++i)
		getline(fin, line);
	// skip the VERTICES label
	getline(fin, line);

	// read the vertices
	for(i = 0; i < num_vertices; ++i)
	{
		Real x, y, z;
		getline(fin, line);
		sStream << line;
		sStream >> x >> y >> z;
		getline(sStream, line);
		sStream.clear();

		vertices[i] = Vector3r(x, y, z);
	}

	// skip TETRAS label
	getline(fin, line);
	// read tets
	for(i = 0; i < num_tetras; ++i)
	{
		unsigned int tet[4];
		unsigned int m;
		getline(fin, line);
		sStream << line;
		sStream >> tet[0] >> tet[1] >> tet[2] >> tet[3] >> m;

		//for (unsigned int j = 0; j < 4; j++)
		//	tet[j]--;

		getline(sStream, line);
		sStream.clear();

		for (int j = 0; j < 4; j++)
			tets[4*i +j] = tet[j];
	}
	// close file
	fin.close();

	LOG_INFO << "Number of tets: " << num_tetras;
	LOG_INFO << "Number of vertices: " << num_vertices;
}



void TetGenLoader::loadTetgenModel(const std::string &nodeFilename, const std::string &eleFilename, std::vector<Vector3r>& vertices, std::vector<unsigned int>& tets)
{
	LOG_INFO << "Loading " << nodeFilename;
	LOG_INFO << "Loading " << eleFilename;

	// variables
	size_t i, num_vertices, num_tetras;
	string nodeLine, eleLine, label;
	stringstream sStream;
	// try to open the file
	ifstream finNode(nodeFilename.c_str());
	ifstream finEle(eleFilename.c_str());
	if(!finNode)
	{
		LOG_ERR << "'" + nodeFilename + "' file not found.";
		return;
	}
	if(!finEle)
	{
		LOG_ERR << "'" + eleFilename + "' file not found.";
		return;
	}

	// get num vertices
	getline(finNode, nodeLine);
	sStream << nodeLine;
	sStream >> num_vertices;
	sStream >> label; // 3
	sStream >> label; // 0
	sStream >> label; // 0
	sStream.clear();

	// get num tetras
	getline(finEle, eleLine);
	sStream << eleLine;
	sStream >> num_tetras;
	sStream >> label; // 4
	sStream >> label; // 0
	sStream >> label; // 0
	sStream.clear();

	vertices.resize(num_vertices);
	tets.resize(4u*num_tetras);

	// read vertices
	for(i = 0; i < num_vertices; ++i)
	{
		unsigned nodeInd;
		Real x, y, z;
		getline(finNode, nodeLine);
		sStream << nodeLine;
		sStream >> nodeInd >> x >> y >> z;
		getline(sStream, nodeLine);
		sStream.clear();
		
		vertices[i] = Vector3r(x, y, z);
	}

	// read tetrahedra
	for(i = 0; i < num_tetras; ++i)
	{
		unsigned eleInd;
		//unsigned int tet[4];
		getline(finEle, eleLine);
		sStream << eleLine	;
		sStream >> eleInd >> tets[4*i+0] >> tets[4*i+1] >> tets[4*i+2] >> tets[4*i+3];

		getline(sStream, eleLine);
		sStream.clear();
	}
	// close file
	finNode.close();
	finEle.close();

	LOG_INFO << "Number of tets: " << num_tetras;
	LOG_INFO << "Number of vertices: " << num_vertices;
}

void TetGenLoader::loadMSHModel(const std::string &mshFilename, std::vector<Vector3r>& vertices, std::vector<unsigned int>& tets)
{
	LOG_INFO << "Loading " << mshFilename;

    // variables
    size_t i, num_vertices, num_tetras;
    string line, label;
    stringstream sStream;
    // try to open the file
    ifstream mshStream(mshFilename.c_str());
    if(!mshStream)
    {
        LOG_ERR << "'" << mshFilename << "' file not found.";
        return;
    }

    // get num vertices
    getline(mshStream, line);
    getline(mshStream, line);
    sStream << line;
    sStream >> num_vertices;
    sStream.clear();

    vertices.resize(num_vertices);

    // read vertices
    for(i = 0; i < num_vertices; ++i)
    {
        unsigned nodeInd;
		Real x, y, z;
        getline(mshStream, line);
        sStream << line;
        sStream >> nodeInd >> x >> y >> z;
        getline(sStream, line);
        sStream.clear();

		vertices[i] = Vector3r(x, y, z);
    }

    // get num tetras
    getline(mshStream, line);
    getline(mshStream, line);
    getline(mshStream, line);
    sStream << line;
    sStream >> num_tetras;
    sStream.clear();

    tets.resize(4u*num_tetras);

    // read tetrahedra
    for(i = 0; i < num_tetras; ++i)
    {
        unsigned eleInd;
        //unsigned int tet[4];
        getline(mshStream, line);
        sStream << line	;
        sStream >> eleInd >> tets[4*i+0] >> tets[4*i+1] >> tets[4*i+2] >> tets[4*i+3];

        --tets[4*i+0];
        --tets[4*i+1];
        --tets[4*i+2];
        --tets[4*i+3];

        getline(sStream, line);
        sStream.clear();
    }
    // close file
    mshStream.close();

	LOG_INFO << "Number of tets: " << num_tetras;
	LOG_INFO << "Number of vertices: " << num_vertices;
}
