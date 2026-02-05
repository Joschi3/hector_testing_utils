# Configuration file for the Sphinx documentation builder.
# This file is used by rosdoc2 when generating documentation.

project = "hector_testing_utils"
copyright = "2026, Aljoscha Schmidt"
author = "Aljoscha Schmidt"

# Extensions
extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.viewcode",
    "myst_parser",
    "breathe",
]

# MyST parser settings (for Markdown support)
myst_enable_extensions = [
    "colon_fence",
    "deflist",
    "fieldlist",
]

# Source file suffixes
source_suffix = {
    ".rst": "restructuredtext",
    ".md": "markdown",
}

# The master toctree document
master_doc = "index"

# HTML theme
html_theme = "sphinx_rtd_theme"

# Breathe configuration (for Doxygen integration)
breathe_default_project = "hector_testing_utils"

# Exclude patterns
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]
