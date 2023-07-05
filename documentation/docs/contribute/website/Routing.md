---
outline: deep
---

# Routing

## File-Based Routing

VitePress uses file-based routing, which means the generated HTML pages are mapped from the directory structure of the source Markdown files. For example, given the following directory structure:

```
.
├─ guide
│  ├─ getting-started.md
│  └─ index.md
├─ index.md
└─ prologue.md
```

The generated HTML pages will be:

```
index.md                  -->  /index.html (accessible as /)
prologue.md               -->  /prologue.html
guide/index.md            -->  /guide/index.html (accessible as /guide/)
guide/getting-started.md  -->  /guide/getting-started.html
```

The resulting HTML can be hosted on any web server that can serve static files.

## Root and Source Directory

There are two important concepts in the file structure of the project: the **project root** and the **source directory**.
