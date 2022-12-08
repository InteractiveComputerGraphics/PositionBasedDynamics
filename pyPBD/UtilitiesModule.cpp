#include "common.h"

#include <pybind11/pybind11.h>
#include <Utils/IndexedFaceMesh.h>
#include <Utils/OBJLoader.h>
#include <Utils/PLYLoader.h>
#include <Utils/TetGenLoader.h>
#include <Utils/Timing.h>
#include <Utils/Logger.h>

namespace py = pybind11;

template <typename... Args>
using overload_cast_ = pybind11::detail::overload_cast_impl<Args...>;

void UtilitiesModule(py::module m_sub) 
{
    using Faces = Utilities::IndexedFaceMesh::Faces;
    using FaceNormals = Utilities::IndexedFaceMesh::FaceNormals;
    using VertexNormals = Utilities::IndexedFaceMesh::VertexNormals;
    using UVIndices = Utilities::IndexedFaceMesh::UVIndices;
    using UVs = Utilities::IndexedFaceMesh::UVs;

    py::class_<Utilities::IndexedFaceMesh>(m_sub, "IndexedFaceMesh")
        .def(py::init<>())
        .def("release", &Utilities::IndexedFaceMesh::release)
        .def("isClosed", &Utilities::IndexedFaceMesh::isClosed)
        .def("initMesh", &Utilities::IndexedFaceMesh::initMesh)
        .def("addFace", [](Utilities::IndexedFaceMesh& mesh, std::vector<int> data) {
            mesh.addFace(data.data());
        })
        .def("addFace", [](Utilities::IndexedFaceMesh& mesh, std::vector<unsigned int> data) {
            mesh.addFace(data.data());
        })
        .def("addUV", &Utilities::IndexedFaceMesh::addUV)
        .def("addUVIndex", &Utilities::IndexedFaceMesh::addUVIndex)

        .def("getFaces", [](Utilities::IndexedFaceMesh& mesh) -> py::memoryview {
            const std::vector<unsigned int>& faces = mesh.getFaces();
            unsigned int* base_ptr = const_cast<unsigned int*>(&faces[0]);
            return py::memoryview::from_buffer(base_ptr, { (int)faces.size() }, { sizeof(unsigned int) }, true);
        })
        .def("getFaceNormals", [](Utilities::IndexedFaceMesh& mesh) -> py::memoryview {
            const auto& n = mesh.getFaceNormals();
            Real* base_ptr = const_cast<Real*>(&n[0][0]);
            return py::memoryview::from_buffer(base_ptr, { (int) n.size(), 3 }, { sizeof(Real) * 3, sizeof(Real) }, true);
        })
        .def("getVertexNormals", [](Utilities::IndexedFaceMesh& mesh) -> py::memoryview {
            const auto& n = mesh.getVertexNormals();
            Real* base_ptr = const_cast<Real*>(&n[0][0]);
            return py::memoryview::from_buffer(base_ptr, { (int) n.size(), 3 }, { sizeof(Real) * 3, sizeof(Real) }, true);
        })
        .def("getEdges", (const Utilities::IndexedFaceMesh::Edges & (Utilities::IndexedFaceMesh::*)()const)(&Utilities::IndexedFaceMesh::getEdges))
        // .def("getEdges", (Edges & (Utilities::IndexedFaceMesh::*)())(&Utilities::IndexedFaceMesh::getEdges)) // TODO: wont work by reference
        .def("getFacesEdges", (const Utilities::IndexedFaceMesh::FacesEdges & (Utilities::IndexedFaceMesh::*)()const)(&Utilities::IndexedFaceMesh::getFacesEdges))
        .def("getUVIndices", (const UVIndices & (Utilities::IndexedFaceMesh::*)()const)(&Utilities::IndexedFaceMesh::getUVIndices))
        .def("getUVs", (const UVs & (Utilities::IndexedFaceMesh::*)()const)(&Utilities::IndexedFaceMesh::getUVs))
        .def("getVertexFaces", (const Utilities::IndexedFaceMesh::VerticesFaces & (Utilities::IndexedFaceMesh::*)()const)(&Utilities::IndexedFaceMesh::getVertexFaces))
        .def("getVertexEdges", (const Utilities::IndexedFaceMesh::VerticesEdges & (Utilities::IndexedFaceMesh::*)()const)(&Utilities::IndexedFaceMesh::getVertexEdges))
        .def("numVertices", &Utilities::IndexedFaceMesh::numVertices)
        .def("numFaces", &Utilities::IndexedFaceMesh::numFaces)
        .def("numEdges", &Utilities::IndexedFaceMesh::numEdges)
        .def("numUVs", &Utilities::IndexedFaceMesh::numUVs)
        .def("copyUVs", &Utilities::IndexedFaceMesh::copyUVs)
        .def("getVerticesPerFace", &Utilities::IndexedFaceMesh::getVerticesPerFace)
        .def("buildNeighbors", &Utilities::IndexedFaceMesh::buildNeighbors)
        .def("updateNormalsVertexData", &Utilities::IndexedFaceMesh::updateNormals<PBD::VertexData>)
        .def("updateVertexNormalsVertexData", &Utilities::IndexedFaceMesh::updateVertexNormals<PBD::VertexData>)
        .def("updateNormalsParticleData", &Utilities::IndexedFaceMesh::updateNormals<PBD::ParticleData>)
        .def("updateVertexNormalsParticleData", &Utilities::IndexedFaceMesh::updateVertexNormals<PBD::ParticleData>);

    py::class_<Utilities::IndexedFaceMesh::Edge>(m_sub, "IndexedFaceMeshEdge")
        .def(py::init<>())
        .def_readwrite("m_face", &Utilities::IndexedFaceMesh::Edge::m_face)
        .def_readwrite("m_vert", &Utilities::IndexedFaceMesh::Edge::m_vert);

    py::class_<Utilities::IndexedTetMesh>(m_sub, "IndexedTetMesh")
        .def(py::init<>())
        .def("release", &Utilities::IndexedTetMesh::release)
        .def("initMesh", &Utilities::IndexedTetMesh::initMesh)
        .def("addTet", overload_cast_<const unsigned int* const>()(&Utilities::IndexedTetMesh::addTet))
        .def("addTet", overload_cast_<const int* const>()(&Utilities::IndexedTetMesh::addTet))

        .def("getFaces", [](Utilities::IndexedTetMesh& mesh) -> py::memoryview {
            const std::vector<unsigned int>& faces = mesh.getFaces();
            unsigned int* base_ptr = const_cast<unsigned int*>(&faces[0]);
            return py::memoryview::from_buffer(base_ptr, { (int)faces.size() }, { sizeof(unsigned int) }, true);
         })
        .def("getTets", [](Utilities::IndexedTetMesh& mesh) -> py::memoryview {
            const std::vector<unsigned int>& tets = mesh.getTets();
            unsigned int* base_ptr = const_cast<unsigned int*>(&tets[0]);
            return py::memoryview::from_buffer(base_ptr, { (int)tets.size() }, { sizeof(unsigned int) }, true);
        })
        .def("getEdges", (const Utilities::IndexedTetMesh::Edges& (Utilities::IndexedTetMesh::*)()const)(&Utilities::IndexedTetMesh::getEdges))
        .def("getFaceData", (const Utilities::IndexedTetMesh::FaceData& (Utilities::IndexedTetMesh::*)()const)(&Utilities::IndexedTetMesh::getFaceData))
        .def("getTetData", (const Utilities::IndexedTetMesh::TetData& (Utilities::IndexedTetMesh::*)()const)(&Utilities::IndexedTetMesh::getTetData))
        .def("getVertexTets", (const Utilities::IndexedTetMesh::VerticesTets& (Utilities::IndexedTetMesh::*)()const)(&Utilities::IndexedTetMesh::getVertexTets))
        .def("getVertexFaces", (const Utilities::IndexedTetMesh::VerticesFaces & (Utilities::IndexedTetMesh::*)()const)(&Utilities::IndexedTetMesh::getVertexFaces))
        .def("getVertexEdges", (const Utilities::IndexedTetMesh::VerticesEdges& (Utilities::IndexedTetMesh::*)()const)(&Utilities::IndexedTetMesh::getVertexEdges))
        .def("numVertices", &Utilities::IndexedTetMesh::numVertices)
        .def("numFaces", &Utilities::IndexedTetMesh::numFaces)
        .def("numTets", &Utilities::IndexedTetMesh::numTets)
        .def("numEdges", &Utilities::IndexedTetMesh::numEdges)
        .def("buildNeighbors", &Utilities::IndexedTetMesh::buildNeighbors);

    py::class_<Utilities::IndexedTetMesh::Edge>(m_sub, "IndexedTetMeshEdge")
        .def(py::init<>())
        .def_readwrite("m_vert", &Utilities::IndexedTetMesh::Edge::m_vert);

    py::class_<Utilities::IndexedTetMesh::Face>(m_sub, "IndexedTetMeshFace")
        .def(py::init<>())
        .def_readwrite("m_tets", &Utilities::IndexedTetMesh::Face::m_tets)
        .def_readwrite("m_edges", &Utilities::IndexedTetMesh::Face::m_edges);

    py::class_<Utilities::IndexedTetMesh::Tet>(m_sub, "IndexedTetMeshTet")
        .def(py::init<>())
        .def_readwrite("m_faces", &Utilities::IndexedTetMesh::Tet::m_faces)
        .def_readwrite("m_edges", &Utilities::IndexedTetMesh::Tet::m_edges);

    py::class_<Utilities::MeshFaceIndices>(m_sub, "MeshFaceIndices")
        .def(py::init<>())
        .def_readwrite("posIndices", &Utilities::MeshFaceIndices::posIndices)
        .def_readwrite("texIndices", &Utilities::MeshFaceIndices::texIndices)
        .def_readwrite("normalIndices", &Utilities::MeshFaceIndices::normalIndices);

    py::class_<Utilities::OBJLoader>(m_sub, "OBJLoader")
        .def(py::init<>())
        .def("loadObj", [](
            const std::string& filename,
            const Utilities::OBJLoader::Vec3f& scale)
            {
                std::vector<Utilities::OBJLoader::Vec3f> x;
                std::vector<Utilities::OBJLoader::Vec3f> normals;
                std::vector<Utilities::OBJLoader::Vec2f> texCoords;
                std::vector<Utilities::MeshFaceIndices> faces;
                Utilities::OBJLoader::loadObj(filename, &x, &faces, &normals, &texCoords, scale);
                return py::make_tuple(x, normals, texCoords, faces);
            })
        .def("loadObjToMesh", [](const std::string& filename, const Vector3r& scale)
            {
                PBD::VertexData vd;
                Utilities::IndexedFaceMesh mesh;
                std::vector<Utilities::OBJLoader::Vec3f> x;
                std::vector<Utilities::OBJLoader::Vec3f> normals;
                std::vector<Utilities::OBJLoader::Vec2f> texCoords;
                std::vector<Utilities::MeshFaceIndices> faces;
                Utilities::OBJLoader::Vec3f s = { (float)scale[0], (float)scale[1], (float)scale[2] };
                Utilities::OBJLoader::loadObj(filename, &x, &faces, &normals, &texCoords, s);

                mesh.release();
                const unsigned int nPoints = (unsigned int)x.size();
                const unsigned int nFaces = (unsigned int)faces.size();
                const unsigned int nTexCoords = (unsigned int)texCoords.size();
                mesh.initMesh(nPoints, nFaces * 2, nFaces);
                vd.reserve(nPoints);
                for (unsigned int i = 0; i < nPoints; i++)
                {
                    vd.addVertex(Vector3r(x[i][0], x[i][1], x[i][2]));
                }
                for (unsigned int i = 0; i < nTexCoords; i++)
                {
                    mesh.addUV(texCoords[i][0], texCoords[i][1]);
                }
                for (unsigned int i = 0; i < nFaces; i++)
                {
                    int posIndices[3];
                    int texIndices[3];
                    for (int j = 0; j < 3; j++)
                    {
                        posIndices[j] = faces[i].posIndices[j];
                        if (nTexCoords > 0)
                        {
                            texIndices[j] = faces[i].texIndices[j];
                            mesh.addUVIndex(texIndices[j]);
                        }
                    }

                    mesh.addFace(&posIndices[0]);
                }
                mesh.buildNeighbors();

                mesh.updateNormals(vd, 0);
                mesh.updateVertexNormals(vd);
                return py::make_tuple(vd, mesh);
            });

    py::class_<Utilities::PLYLoader>(m_sub, "PLYLoader")
        .def(py::init<>())
        .def_static("loadPly", &Utilities::PLYLoader::loadPly);

    py::class_<Utilities::TetGenLoader>(m_sub, "TetGenLoader")
        .def(py::init<>())
        .def("loadTetFile", [](
            const std::string& filename)
            {
                std::vector<Vector3r> x;
                std::vector<unsigned int> tets;
                Utilities::TetGenLoader::loadTetFile(filename, x, tets);
                return py::make_tuple(x, tets);
            })
        .def("loadTetgenModel", [](
            const std::string& nodeFilename, const std::string& eleFilename)
            {
                std::vector<Vector3r> x;
                std::vector<unsigned int> tets;
                Utilities::TetGenLoader::loadTetgenModel(nodeFilename, eleFilename, x, tets);
                return py::make_tuple(x, tets);
            });

    // ---------------------------------------
    // Logger
    // ---------------------------------------
    py::enum_<Utilities::LogLevel>(m_sub, "LogLevel")
        .value("DEBUG", Utilities::LogLevel::DEBUG)
        .value("INFO", Utilities::LogLevel::INFO)
        .value("WARN", Utilities::LogLevel::WARN)
        .value("ERR", Utilities::LogLevel::ERR);

    py::class_<Utilities::ConsoleSink>(m_sub, "ConsoleSink")
        .def(py::init<>([](const Utilities::LogLevel minLevel) {
        return Utilities::ConsoleSink(minLevel);
            }))
        .def("write", &Utilities::ConsoleSink::write);

    // TODO: check if it is okay to use shared pointer and implement the actual logger functions
    py::class_<Utilities::FileSink, std::shared_ptr<Utilities::FileSink>>(m_sub, "FileSink")
        .def(py::init<>([](const Utilities::LogLevel minLevel, const std::string& fileName) {
        return std::make_shared<Utilities::FileSink>(minLevel, fileName);
            }))
        .def("write", &Utilities::FileSink::write);

    py::class_<Utilities::Logger>(m_sub, "Logger")
        .def(py::init<>())
        .def("addConsoleSink", [](const Utilities::LogLevel &level)
            {
                Utilities::logger.addSink(std::unique_ptr<Utilities::ConsoleSink>(new Utilities::ConsoleSink(level)));
            })
        .def("addFileSink", [](const Utilities::LogLevel& level, const std::string &fileName)
            {
                Utilities::logger.addSink(std::unique_ptr<Utilities::FileSink>(new Utilities::FileSink(level, fileName)));
            })
        .def("write", &Utilities::Logger::write);

    // ---------------------------------------
    // Timing
    // ---------------------------------------
    // TODO: Timing and find a way for everything to be actually printed
    py::class_<Utilities::TimingHelper>(m_sub, "TimingHelper")
        .def(py::init<>())
        .def_readwrite("start", &Utilities::TimingHelper::start)
        .def_readwrite("name", &Utilities::TimingHelper::name);

    py::class_<Utilities::AverageTime>(m_sub, "AverageTime")
        .def(py::init<>())
        .def_readwrite("totalTime", &Utilities::AverageTime::totalTime)
        .def_readwrite("counter", &Utilities::AverageTime::counter)
        .def_readwrite("name", &Utilities::AverageTime::name);

    py::class_<Utilities::IDFactory>(m_sub, "IDFactory")
        .def(py::init<>())
        .def_static("getId", &Utilities::IDFactory::getId);

    py::class_<Utilities::Timing>(m_sub, "Timing")
        .def(py::init<>())
        .def_readwrite_static("m_dontPrintTimes", &Utilities::Timing::m_dontPrintTimes)
        .def_readwrite_static("m_startCounter", &Utilities::Timing::m_startCounter)
        .def_readwrite_static("m_stopCounter", &Utilities::Timing::m_stopCounter)
        .def_readwrite_static("m_timingStack", &Utilities::Timing::m_timingStack)
        .def_readwrite_static("m_averageTimes", &Utilities::Timing::m_averageTimes)
        .def_static("reset", &Utilities::Timing::reset)
        .def_static("startTiming", &Utilities::Timing::startTiming)
        .def_static("stopTimingPrint", []()
            {
                static int timing_timerId = -1;
                Utilities::Timing::stopTiming(true);
            })
        .def_static("stopTimingAvgPrint", []()
            {
                static int timing_timerId = -1; 
                Utilities::Timing::stopTiming(true, timing_timerId); 
            })
        .def_static("stopTimingAvg", []()
            {
                static int timing_timerId = -1;
                Utilities::Timing::stopTiming(false, timing_timerId);
            })
        .def_static("printAverageTimes", &Utilities::Timing::printAverageTimes)
        .def_static("printTimeSums", &Utilities::Timing::printTimeSums);

}
