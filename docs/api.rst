rc_agent C++ API reference
==========================

The complete API reference is generated **at build time** by Doxygen from
a separate checkout of the ``feat/rc-api-v2`` source branch. It includes
the class index, namespaces, implementation/file browser, and Graphviz diagrams.

`Browse the full Doxygen HTML API reference
<https://ardev1161.github.io/rc_agent/doxygen/index.html>`_.

The documentation build additionally generates Doxygen XML and configures
Breathe for curated Sphinx reference pages. The initial source contains
repeated or malformed C++ declarations that make an indiscriminate
``doxygenindex`` expansion unsafe under strict Sphinx validation.
Instead of hiding the warning or weakening CI, the standalone Doxygen
reference is published in full; curated Breathe references can be added
only when each one passes CI.

This API is **not** evidence that all RC API v2 protobuf schemas have been
wired into the running application. See :doc:`protocol` and
:doc:`validation` for the actual migration status.
