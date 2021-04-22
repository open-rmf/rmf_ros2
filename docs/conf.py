# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
# import os
# import sys
# sys.path.insert(0, os.path.abspath('.'))


# -- Project information -----------------------------------------------------

project = 'rmf_ros2'
copyright = '2021, Open Source Robotics Corporation'
author = 'Open Source Robotics Corporation'

# The full version, including alpha/beta/rc tags
release = '1.0.0'


# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    'breathe',
    'exhale',
]

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = []


# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
try:
    import sphinx_rtd_theme
except ImportError:
    html_theme = 'alabaster'
    # This is required for the alabaster theme
    # refs: http://alabaster.readthedocs.io/en/latest/installation.html#sidebars
    html_sidebars = {
        '**': [
            'relations.html',  # needs 'show_related': True theme option
            'searchbox.html',
            ]
        }
    sys.stderr.write('sphinx_rtd_theme missing. Use pip to install it.\n')
else:
    html_theme = "sphinx_rtd_theme"
    html_theme_path = [sphinx_rtd_theme.get_html_theme_path()]
    html_theme_options = {
        'canonical_url': '',
        'analytics_id': '',
        'logo_only': False,
        'display_version': True,
        'prev_next_buttons_location': 'None',
        # Toc options
        'collapse_navigation': False,
        'sticky_navigation': True,
        'navigation_depth': 4,
    }


# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']


# -- Options for breathe and exhale ------------------------------------------

import os
import os.path
import subprocess

read_the_docs_build = os.environ.get('READTHEDOCS', None) == 'True'
if read_the_docs_build:
    if not os.path.isdir('_build/doxygen'):
        os.makedirs('_build/doxygen')
    subprocess.call('doxygen', shell=True)

breathe_projects = {
    'rmf_ros2': '_build/doxygen/xml'
}
breathe_default_project = 'rmf_ros2'

exhale_args = {
    'containmentFolder': './api',
    'rootFileName': 'rmf_ros2_root.rst',
    'rootFileTitle': 'rmf_ros2 API',
    'doxygenStripFromPath': '../..',
    'createTreeView': True,
    'exhaleExecutesDoxygen': False,
    'exhaleDoxygenStdin': 'INPUT = ../rmf_traffic_ros2/include ../rmf_task_ros2/include ../rmf_fleet_adapter/include ../rmf_fleet_adapter_python/include',
}

primary_domain = 'cpp'
highlight_language = 'cpp'
