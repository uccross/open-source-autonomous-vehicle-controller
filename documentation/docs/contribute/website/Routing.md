---
outline: deep
---

# Routing

## File-Based Routing

VitePress uses file-based routing, which means the generated HTML pages are mapped from the directory structure of the source Markdown files. For example, given the following directory structure:

```
.
├─ guide
│  ├─ introduction.md
│  └─ index.md
├─ index.md
└─ about
```

The generated HTML pages will be:

```
index.md                  -->  /index.html (accessible as /)
about                     -->  /about
guide/index.md            -->  /guide/index.html (accessible as /guide/)
guide/getting-started.md  -->  /guide/getting-started.html
```

The resulting HTML can be hosted on any web server that can serve static files.

## Root and Source Directory

There are two important concepts in the file structure of the project: the **project root** and the **source directory**.

### Project Root

Project root is where VitePress will try to look for the `.vitepress` special directory. The `.vitepress` directory is a reserved location for VitePress' config file, dev server cache, build output, and optional theme customization code.

When you run `vitepress dev` or `vitepress build` from the command line, VitePress will use the current working directory as project root. To specify a sub-directory as root, you will need to pass the relative path to the command. For example, our VitePress project is located in `./docs`, you should run `vitepress dev docs`:

```
.
├─ docs                    # project root
│  ├─ .vitepress           # config dir
│  ├─ getting-started.md
│  └─ index.md
└─ ...
```

```sh
vitepress dev docs
```

This is going to result in the following source-to-HTML mapping:

```
docs/index.md            -->  /index.html (accessible as /)
docs/getting-started.md  -->  /getting-started.html
```

### Source Directory

Source directory is where your Markdown source files live. By default, it is the same as the project root. However, you can configure it via the `srcDir` config option.

The `srcDir` option is resolved relative to project root. For example, with `srcDir: 'src'`, your file structure will look like this:

```
.                          # project root
├─ .vitepress              # config dir
└─ src                     # source dir
   ├─ getting-started.md
   └─ index.md
```

The resulting source-to-HTML mapping:

```
src/index.md            -->  /index.html (accessible as /)
src/getting-started.md  -->  /getting-started.html
```

## Linking Between Pages

You can use both absolute and relative paths when linking between pages. Note that although both `.md` and `.html` extensions will work, the best practice is to omit file extensions so that VitePress can generate the final URLs based on your config.

```md
<!-- Do -->
[Getting Started](./getting-started)
[Getting Started](../guide/getting-started)

<!-- Don't -->
[Getting Started](./getting-started.md)
[Getting Started](./getting-started.html)
```

- Learn more about routing in vitepress [here](https://vitepress.dev/guide/routing)

- Website pages are written in Markdown format which are then converted to HTML.

- Learn about Makdown language [here.](https://www.markdownguide.org/)
