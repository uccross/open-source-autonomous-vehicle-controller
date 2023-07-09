# Getting Started

The Documentation website for the OSAVC controller is being created using the **Vitepress framework.**

## What is VitePress?

VitePress is a [Static Site Generator](https://en.wikipedia.org/wiki/Static_site_generator) (SSG) designed for building fast, content-centric websites. In a nutshell, VitePress takes your source content written in [Markdown](https://en.wikipedia.org/wiki/Markdown), applies a theme to it, and generates static HTML pages that can be easily deployed anywhere.

Learn more about Vitepress [Here](https://vitepress.dev/guide/what-is-vitepress).

## Installation

### Prerequisites

- [Node.js](https://nodejs.org/) version 16 or higher.
- Terminal for accessing VitePress via its command line interface (CLI).
- Text Editor with [Markdown](https://en.wikipedia.org/wiki/Markdown) syntax support.
  - [VSCode](https://code.visualstudio.com/) is recommended, along with the [official Vue extension](https://marketplace.visualstudio.com/items?itemName=Vue.volar).

## Cloning the Repository in your PC

To clone the Repo just execute the following command on Gitbash or your terminal prompt. We are cloning the repository in Desktop you can clone it into directory you want. Navigate to your desired directory

```sh
cd Desktop
```

```sh
git clone https://github.com/uccross/open-source-autonomous-vehicle-controller.git
```

Now navigate to the documentation folder

```sh
cd documentation
```

This is the project folder for our documentation Website.

## File Structure

The file structure will looks like this:

```
.
├─ docs
│  ├─ .vitepress
│  │  └─ config.js
│  ├─ about
│  ├─ assets
│  └─ blog
│  └─ contribute
│  └─ guide
│  └─ index.md
└─ package.json
```

The `docs` directory is the **project root** of the Website. The `.vitepress` directory is a reserved location for VitePress' config file, dev server cache, build output, and optional theme customization code.

### The Config File

The config file (`.vitepress/config.ts`) allows you to customize various aspects of your Website, with the most basic options being the title and description of the site:

```js
// .vitepress/config.ts
export default {
  // site-level options
  title: 'OSAVC',
  description: 'Open Source Autonomous vehical Controller.',

  themeConfig: {
    // theme-level options
  }
}
```

### Source Files

Markdown files outside the `.vitepress` directory are considered **source files**.

VitePress uses **file-based routing**: each `.md` file is compiled into a corresponding `.html` file with the same path. For example, `index.md` will be compiled into `index.html`, and can be visited at the root path `/` of the resulting VitePress site.

VitePress also provides the ability to generate clean URLs, rewrite paths, and dynamically generate pages. These will be covered in the [Routing Guide](../website/Routing.md).

## Up and Running

The "scripts" property of your `package.json` file supports a number of built-in scripts and their preset life cycle events as well as arbitrary scripts.

```json
{
  ...
  "scripts": {
    "docs:dev": "vitepress dev docs",
    "docs:build": "vitepress build docs",
    "docs:preview": "vitepress preview docs"
  },
  ...
}
```

The `docs:dev` script will start a local dev server with instant hot updates. Run it with the following command:

::: code-group

```sh [npm]
$ npm run docs:dev
```

```sh [pnpm]
$ pnpm run docs:dev
```

```sh [yarn]
$ yarn docs:dev
```

:::

The dev server should be running at `http://localhost:5173`. Visit the URL in your browser to see your new site in action!
