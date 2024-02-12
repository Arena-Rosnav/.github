# Arena Rosnav Documentation

## Installation

To start using the docs locally, just install the plugins from [requirements.txt](requirements.txt) or run

```sh
pip install mkdocs mkdocs-video jinja2
```

To start the Docs, run

```sh
mkdocs serve
```

## Style Guide

- The proper name of the project is Arena-Rosnav and should not be stylized.
- Stylize other proper nouns and trademarks as _emphasized_ text.
- Stylize paths, file names, and parameters as simple `quote blocks`.
- Reference locations relative to their parent package when possible. Otherwise specify paths relative to the workspace root.
- Specify directories with a trailing slash.
- `insert/<camel_case_parameters>/anywhere` when needed.