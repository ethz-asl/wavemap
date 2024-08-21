import os
from sys import argv
from dataclasses import asdict
from sphinxawesome_theme import ThemeOptions
from sphinxawesome_theme.postprocess import Icons
import lxml.etree

# Project information
project = 'wavemap'
author = 'Victor Reijgwart'
copyright = 'Victor Reijgwart, ASL ETHZ.'  # pylint: disable=redefined-builtin

# The full version, including alpha/beta/rc tags
release = lxml.etree.parse('../interfaces/ros1/wavemap/package.xml').find(
    'version').text
# The short X.Y version
version = release

# General configuration
extensions = [
    'sphinx.ext.mathjax', "sphinx.ext.extlinks", 'sphinx.ext.githubpages',
    'sphinx.ext.autodoc', 'sphinx_design', 'sphinx_sitemap', 'breathe',
    'exhale'
]
templates_path = ['_templates']
source_suffix = ['.rst', '.md']
master_doc = 'index'
language = 'en'
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

# The name of the Pygments (syntax highlighting) style to use
pygments_style = 'sas'

# Exclude latex helper files when building other document types
if 'latexpdf' not in argv:
    exclude_patterns = ['**latex_*']

# Options for HTML output

# The theme to use
html_title = project
html_theme = "sphinxawesome_theme"
html_last_updated_fmt = ""
html_use_index = False  # Don't create index
html_domain_indices = False  # Don't need module indices
html_copy_source = False  # Don't need sources
html_logo = 'logo.png'
html_favicon = 'favicon.ico'
html_permalinks_icon = Icons.permalinks_icon
html_baseurl = 'https://ethz-asl.github.io/wavemap/'
html_extra_path = ["robots.txt", "_redirects", "google1182636d9b15b461.html"]
html_context = {
    "mode": "production",
}
html_static_path = ["_static"]
html_css_files = ["custom.css"]
html_js_files = []

# Theme specific options
theme_options = ThemeOptions(
    show_prev_next=True,
    awesome_external_links=True,
    main_nav_links={
        "Docs": "index",
        "Code": "https://github.com/ethz-asl/wavemap",
        "Releases": "https://github.com/ethz-asl/wavemap/releases"
    },
)
html_theme_options = asdict(theme_options)

# Sitemap specific options
sitemap_url_scheme = "{link}"

# Options for LaTeX output
latex_elements = {}
latex_documents = [
    ('latex_index', 'wavemap.tex', 'Wavemap Documentation', author, 'manual'),
]
man_pages = []

# Options for Texinfo output

# Grouping the document tree into Texinfo files. List of tuples
# (source start file, target name, title, author,
#  dir menu entry, description, category)
texinfo_documents = [
    ('latex_index', 'wavemap', 'Wavemap Documentation', author, 'wavemap',
     'Fast, efficient and accurate multi-resolution, '
     'multi-sensor 3D occupancy mapping.', 'Miscellaneous'),
]

# Extension configuration

# Setup the breathe extension
breathe_projects = {
    "wavemap_cpp": "./_doxygen_cpp/xml",
    "wavemap_ros1": "./_doxygen_ros1/xml"
}
breathe_default_project = "wavemap_cpp"

# Setup the exhale extension
exhale_args = {
    "verboseBuild": False,
    # These arguments are required
    "containmentFolder": "./cpp_api",
    # Tell exhale we'll build our TOC tree and pin the file names
    "rootFileName": "EXCLUDE",
    "classHierarchyFilename": 'class_view_hierarchy.rst',
    "fileHierarchyFilename": 'file_view_hierarchy.rst',
    "unabridgedApiFilename": 'unabridged_api.rst',
    # Must be the same as STRIP_FROM_PATH in the Doxyfile
    "doxygenStripFromPath": "..",
    # Heavily encouraged optional argument (see docs)
    # "rootFileTitle": "API",
    "fullApiSubSectionTitle": "C++ API",
    # Suggested optional arguments
    "createTreeView": False,
    # TIP: if using the sphinx-bootstrap-theme, you need
    # "treeViewIsBootstrap": True,
    "exhaleExecutesDoxygen": False,
    "exhaleUseDoxyfile": True,
    "pageLevelConfigMeta": ":github_url: https://github.com/ethz-asl/wavemap"
}

# Tell sphinx what the primary language being documented is
primary_domain = 'cpp'

# Tell sphinx what the pygments highlight language should be
highlight_language = 'cpp'

# Provide a short syntax to link to files in the repository
sha = os.environ.get('GITHUB_SHA')  # Attempt to read it from CI env variables
if sha is None:
    import git

    repo = git.Repo(os.path.dirname(__file__), search_parent_directories=True)
    sha = repo.head.object.hexsha

extlinks = {
    "gh_file": (f"https://github.com/ethz-asl/wavemap/tree/{sha}/%s", "%s"),
}

# Configure the link checker (invoked with `make linkcheck`)
linkcheck_allowed_redirects = {
    # All HTTP redirections from the source URI to the canonical URI will be treated as "working".
    'https://github.com/ethz-asl/wavemap/tree/.*':
    'https://github.com/ethz-asl/wavemap/blob.*',
    'https://github.com/ethz-asl/wavemap/assets/.*': 'https://.*'
}
