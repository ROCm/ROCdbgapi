# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

import os

from rocm_docs import ROCmDocs

os.system("sudo apt update")
os.system("sudo apt install cmake")
os.system("cmake -P ../cmake/build-amd-dbgapi-h.cmake")
os.system("cp ../README.md index.md")

docs_core = ROCmDocs("ROCdbgapi Documentation")
docs_core.run_doxygen()
docs_core.enable_api_reference()
docs_core.setup()

for sphinx_var in ROCmDocs.SPHINX_VARS:
    globals()[sphinx_var] = getattr(docs_core, sphinx_var)
