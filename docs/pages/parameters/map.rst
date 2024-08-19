Map
###
.. highlight:: YAML
.. rstcheck: ignore-directives=doxygenstruct

The settings in the config's ``map`` section control the data structure used to store the map. We currently recommend using either the ``hashed_chunked_wavelet_octree`` or ``hashed_wavelet_octree`` data structure. The ``hashed_chunked_wavelet_octree`` data structure provides the best performance in terms of computational speed and RAM usage. In case you're interested in modifying wavemap's code, the ``hashed_wavelet_octree`` data structure is simpler while still offering good performance. For learning purposes, we also briefly discuss wavemap's simplest wavelet-based data structure: the ``wavelet_octree``.

Wavelet octree
**************
Selected by setting ``map/type`` to ``"wavelet_octree"``.

The ``wavelet_octree`` is the simplest of the wavelet-based data structures and stores the wavelet coefficients in a standard octree. This data structure can be useful in case you need the entire map to be contained in a single tree and don't mind sacrificing performance.

.. doxygenstruct:: wavemap::WaveletOctreeConfig
    :project: wavemap_cpp
    :members:

Hashed wavelet octree
*********************
Selected by setting ``map/type`` to ``"hashed_wavelet_octree"``.

The ``hashed_wavelet_octree`` combines the strengths of wavelet octrees with block-hashing. At the top level, the map is split into blocks which are accessed through a hash table. Each block in turn contains a small wavelet octree.

.. doxygenstruct:: wavemap::HashedWaveletOctreeConfig
    :project: wavemap_cpp
    :members:

Hashed chunked wavelet octree
*****************************
Selected by setting ``map/type`` to ``"hashed_chunked_wavelet_octree"``.

The ``hashed_chunked_wavelet_octree`` is similar to the ``hashed_wavelet_octree``, but instead of storing all octree nodes separately it groups them into chunks.

.. doxygenstruct:: wavemap::HashedChunkedWaveletOctreeConfig
    :project: wavemap_cpp
    :members:
