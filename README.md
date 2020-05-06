AMD Debugger API (ROCdbgapi)
============================

The AMD Debugger API is a library that provides all the support necessary for a
debugger and other tools to perform low level control of the execution and
inspection of execution state of AMD's commercially available GPU architectures.

The following AMD GPU architectures are supported:

- GFX9
  - gfx900 (Vega 10)
  - gfx906 (Vega 7nm also referred to as Vega 20)
  - gfx908 (Arcturus)

For more information about ROCm and ROCdbgapi, please refer to the Release Notes
file available at:

- https://github.com/RadeonOpenCompute/ROCm

Build the AMD Debugger API Library
----------------------------------

The ROCdbgapi library can be built on Ubuntu 16.04, Ubuntu 18.04, Centos 8.1,
RHEL 8.1, and SLES 15 Service Pack 1.

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

   NOTE: The ``doxygen`` 1.8.13 that is installed by Ubuntu 18.04 has a bug that
   prevents the PDF from being created. Both ``doxygen`` 1.8.11 and 1.8.17 can
   be built from source to avoid the issue.

2. For CentOS 8.1 and RHEL 8.1 the following adds the needed packages:

   ````shell
   yum install -y doxygen graphviz texlive texlive-xtab texlive-multirow \
     texlive-sectsty texlive-tocloft
   ````

3. For SLES 15 Service Pack 15 the following adds the needed packages:

   ````shell
   zypper in doxygen graphviz texlive-scheme-medium texlive-hanging \
     texlive-stackengine texlive-tocloft texlive-etoc texlive-tabu
   ````

An example command-line to generate the HTML and PDF documentation after running
the above ``cmake`` is:

````shell
make doc
````

The generated ROCdbgapi library documentation is put in:

- ``doc/html/index.html``
- ``doc/latex/refman.pdf``

If the ROCdbgapi library PDF documentation has been generated, ``make install``
will place it in:

- ``../install/share/doc/amd-dbgapi/amd-dbgapi.pdf``
