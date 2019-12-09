AMD Debugger API (ROCdbgapi)
============================

The AMD Debugger API is a library that provides all the support necessary for a
debugger and other tools to perform low level control of the execution and
inspection of execution state of AMD's commercially available GPU architectures.

The following AMD GPU architectures are supported:

- GFX9
  - gfx900 (Vega 10)
  - gfx906 (Vega 20)
  - gfx908 (MI100)

For more information about ROCm and ROCdbgapi, please refer to the Release Notes
file available at:

- https://github.com/RadeonOpenCompute/ROCm

Build the AMD Debugger API Library
----------------------------------

The ROCdbgapi library can be built on Ubuntu 16.04, Ubuntu 18.04, and Centos
7.6.

Building the ROCdbgapi library has the following prerequisites:

1. A C++14 compiler such as GCC 5 or Clang 3.4.

2. AMD Code Object Manager Library (ROCcomgr) which can be installed as part of
   the ROCm release by the ``comgr`` package.

3. ROCm CMake modules which can be installed as part of the ROCm release by the
   ``rocm-cmake`` package.

An example command-line to build and install the ROCdbgapi library on Linux is:

````shell
cd rocdbgapi
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=../install ..
make
````

You may substitute a path of your own choosing for ``CMAKE_INSTALL_PREFIX``.

The built ROCdbgapi library will be placed in:

- ``build/include/amd-dbgapi.h``
- ``build/librocm-dbgapi.so*``

To install the ROCdbgapi library:

````shell
make install
````

The installed ROCdbgapi library will be placed in:

- ``../install/include/amd-dbgapi.h``
- ``../install/lib/librocm-dbgapi.so*``
- ``../install/share/amd-dbgapi/LICENSE.txt``
- ``../install/share/amd-dbgapi/README.md``

To use the ROCdbgapi library, the ROCcomgr library must be installed. This can
be installed as part of the ROCm release by the ``comgr`` package:

- ``libamd_comgr.so.1``

Build the AMD Debugger API Documentation
----------------------------------------

Generating the AMD Debugger API documentation has the following prerequisites:

1. For Ubuntu 16.04 and Ubuntu 18.04 the following adds the needed packages:

   ````shell
   apt install doxygen graphviz texlive-full
   ````

2. For Centos 7.6 the following adds the needed packages:

   ````shell
   yum install -y doxygen graphviz texlive
   ````

An example command-line to generate the HTML and PDF documentation after running
the above ``cmake`` is:

````shell
make doc
````

The generated documentation is put in:

- ``doc/html/index.html``
- ``doc/latex/refman.pdf``
