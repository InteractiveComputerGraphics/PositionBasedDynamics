Implementation of "Position Based Elastic Rods" paper
(http://www.nobuyuki-umetani.com/PositionBasedElasticRod/2014_sca_PositionBasedElasticRod.html)
for Position Based Dynamics library (http://github.com/janbender/PositionBasedDynamics):

Przemyslaw Korzeniowski
Department of Surgery and Cancer
Imperial College London

http://github.com/korzen/PositionBasedDynamics-ElasticRod
p.korzeniowski [at] imperial.ac.uk
korzenio [at] gmail.com

Bending and twisting constraints ported to Eigen from https://github.com/serpheroth/rod 

So far tested only on Win8.1 using Visual Studio 2013 and 2015 (both x86 and x64)

IMPORTANT: OpenMP is not supported in this release!
By default, the CMake generated ElasticRodDemo project has OpenMP turned on. 
Deactivate it to run the demo, otherwise to rod will explode.

Release notes:
v0.1 (initial release)
-Added ghost points to simulation model
-Added edge and ghost point constraints
-Added bending and twisting constraints
-Added simple viscous dampening to time-stepping
-Simple demo

ToDo
-Fix CMake project file to turn off OpenMP by default
-Add control over edge and ghost rest lengths
-Optimize matrix ops performance
-Gravity fix for ghost points
-Bilateral constraints interleaving
-Fix OpenMP
-Frame attachments
-Test build on Linux/MacOS

