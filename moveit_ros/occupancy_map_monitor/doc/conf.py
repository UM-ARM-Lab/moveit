project = 'moveit.occupancy_map_monitor'
copyright = '2021, MoveIt maintainer team'
author = 'MoveIt maintainer team'

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.autosummary',
    'sphinx.ext.intersphinx',
    "sphinx_rtd_theme",
]

# NOTE: Important variables that make auto-generation work,
#  see https://www.sphinx-doc.org/en/master/usage/extensions/autodoc.html
autosummary_generate = True
autosummary_imported_members = True
autoclass_content = "both"  # Add __init__ doc (ie. params) to class summaries

# Customization, not as important
html_show_sourcelink = True  # Remove 'view source code' from top of page (for html, not python)
autodoc_inherit_docstrings = True  # If no docstring, inherit from base class
set_type_checking_flag = True  # Enable 'expensive' imports for sphinx_autodoc_typehints
add_module_names = False

templates_path = ['_templates']

source_suffix = '.rst'

master_doc = 'index'

exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store', '_templates']

html_theme = "sphinx_rtd_theme"

html_static_path = ['_static']

htmlhelp_basename = 'moveit_occupancy_map_monitordoc'

intersphinx_mapping = {'https://docs.python.org/3': None}
