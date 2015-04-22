# RVO2_3D_Float_Java
A Jave implementation of the RVO2_3D library

This is a Java implementation of the RVO2_3D library. The original RVO2 library is a C++ implementation of 
Reciprocal Collision Avoidance for Real-Time Multi-Agent Simulation. More information can be found in this link:
http://gamma.cs.unc.edu/RVO2/

The original code uses float type for real numbers, and this library adheres to it. For a double type based implementation,
please refer to another library named RVO2_3D_Double_Java.

This library uses Double3D and MutableDouble3D for 3D position and velocity, which are from Mason (http://cs.gmu.edu/~eclab/projects/mason/).
However, I made some change to these to classes. So, to use this code, you first need to download the Mason code and add import
it into eclipse as a project. Then you need to substitude the two .java files in Mason (i.e. MASON/mason/sim/util/Double3D.java and
MASON/mason/sim/util/MutableDouble3D.java) with the files provide in the "RVO2_3D_Float_Java/depend" folder.
Finally, You will need you need to refer the RVO2_3D_Float_Java to the imported project. 
