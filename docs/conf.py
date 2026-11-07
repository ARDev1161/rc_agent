"""Sphinx documentation for rc_agent; source code checked out separately."""
from pathlib import Path

DOCS = Path(__file__).resolve().parent
project = "rc_agent"
author = "rc_agent contributors"
extensions = ["myst_parser", "sphinxcontrib.mermaid", "breathe"]
source_suffix = {".md": "markdown", ".rst": "restructuredtext"}
root_doc = "index"
myst_fence_as_directive = ["mermaid"]
myst_heading_anchors = 3
mermaid_config = {"securityLevel": "strict"}
mermaid_fullscreen = False
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]
breathe_projects = {"rc_agent": str(DOCS / "_build" / "xml")}
breathe_default_project = "rc_agent"
revision_file = DOCS / "_source_sha.txt"
source_sha = revision_file.read_text(encoding="utf-8").strip() if revision_file.exists() else "unbuilt"
html_theme = "furo"
html_title = f"rc_agent documentation · source {source_sha[:12]}"
html_baseurl = "https://ardev1161.github.io/rc_agent/"
html_static_path = ["_static"]
html_css_files = ["layout.css"]
html_theme_options = {
    "source_repository": "https://github.com/ARDev1161/rc_agent/",
    "source_branch": "documentation",
    "source_directory": "docs/",
}
