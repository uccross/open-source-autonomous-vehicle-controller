import { _ as _export_sfc, o as openBlock, c as createElementBlock, V as createStaticVNode } from "./chunks/framework.ce59e187.js";
const __pageData = JSON.parse('{"title":"Getting Started","description":"","frontmatter":{},"headers":[],"relativePath":"contribute/website/getting-started.md"}');
const _sfc_main = { name: "contribute/website/getting-started.md" };
const _hoisted_1 = /* @__PURE__ */ createStaticVNode('<h1 id="getting-started" tabindex="-1">Getting Started <a class="header-anchor" href="#getting-started" aria-label="Permalink to &quot;Getting Started&quot;">​</a></h1><p>The Documentation website for the OSAVC controller is being created using the <strong>Vitepress framework.</strong></p><h2 id="what-is-vitepress" tabindex="-1">What is VitePress? <a class="header-anchor" href="#what-is-vitepress" aria-label="Permalink to &quot;What is VitePress?&quot;">​</a></h2><p>VitePress is a <a href="https://en.wikipedia.org/wiki/Static_site_generator" target="_blank" rel="noreferrer">Static Site Generator</a> (SSG) designed for building fast, content-centric websites. In a nutshell, VitePress takes your source content written in <a href="https://en.wikipedia.org/wiki/Markdown" target="_blank" rel="noreferrer">Markdown</a>, applies a theme to it, and generates static HTML pages that can be easily deployed anywhere.</p><p>Learn more about Vitepress <a href="https://vitepress.dev/guide/what-is-vitepress" target="_blank" rel="noreferrer">Here</a>.</p><h2 id="installation" tabindex="-1">Installation <a class="header-anchor" href="#installation" aria-label="Permalink to &quot;Installation&quot;">​</a></h2><h3 id="prerequisites" tabindex="-1">Prerequisites <a class="header-anchor" href="#prerequisites" aria-label="Permalink to &quot;Prerequisites&quot;">​</a></h3><ul><li><a href="https://nodejs.org/" target="_blank" rel="noreferrer">Node.js</a> version 16 or higher.</li><li>Terminal for accessing VitePress via its command line interface (CLI).</li><li>Text Editor with <a href="https://en.wikipedia.org/wiki/Markdown" target="_blank" rel="noreferrer">Markdown</a> syntax support. <ul><li><a href="https://code.visualstudio.com/" target="_blank" rel="noreferrer">VSCode</a> is recommended, along with the <a href="https://marketplace.visualstudio.com/items?itemName=Vue.volar" target="_blank" rel="noreferrer">official Vue extension</a>.</li></ul></li></ul><h2 id="cloning-the-repository-in-your-pc" tabindex="-1">Cloning the Repository in your PC <a class="header-anchor" href="#cloning-the-repository-in-your-pc" aria-label="Permalink to &quot;Cloning the Repository in your PC&quot;">​</a></h2><p>To clone the Repo just execute the following command on Gitbash or your terminal prompt. We are cloning the repository in Desktop you can clone it into directory you want. Navigate to your desired directory</p><div class="language-sh"><button title="Copy Code" class="copy"></button><span class="lang">sh</span><pre class="shiki material-theme-palenight"><code><span class="line"><span style="color:#82AAFF;">cd</span><span style="color:#A6ACCD;"> </span><span style="color:#C3E88D;">Desktop</span></span></code></pre></div><div class="language-sh"><button title="Copy Code" class="copy"></button><span class="lang">sh</span><pre class="shiki material-theme-palenight"><code><span class="line"><span style="color:#FFCB6B;">git</span><span style="color:#A6ACCD;"> </span><span style="color:#C3E88D;">clone</span><span style="color:#A6ACCD;"> </span><span style="color:#C3E88D;">https://github.com/uccross/open-source-autonomous-vehicle-controller.git</span></span></code></pre></div><p>Now navigate to the documentation folder</p><div class="language-sh"><button title="Copy Code" class="copy"></button><span class="lang">sh</span><pre class="shiki material-theme-palenight"><code><span class="line"><span style="color:#82AAFF;">cd</span><span style="color:#A6ACCD;"> </span><span style="color:#C3E88D;">documentation</span></span></code></pre></div><p>This is the project folder for our documentation Website.</p><h2 id="file-structure" tabindex="-1">File Structure <a class="header-anchor" href="#file-structure" aria-label="Permalink to &quot;File Structure&quot;">​</a></h2><p>The file structure will looks like this:</p><div class="language-"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki material-theme-palenight"><code><span class="line"><span style="color:#A6ACCD;">.</span></span>\n<span class="line"><span style="color:#A6ACCD;">├─ docs</span></span>\n<span class="line"><span style="color:#A6ACCD;">│  ├─ .vitepress</span></span>\n<span class="line"><span style="color:#A6ACCD;">│  │  └─ config.js</span></span>\n<span class="line"><span style="color:#A6ACCD;">│  ├─ about</span></span>\n<span class="line"><span style="color:#A6ACCD;">│  ├─ assets</span></span>\n<span class="line"><span style="color:#A6ACCD;">│  └─ blog</span></span>\n<span class="line"><span style="color:#A6ACCD;">│  └─ contribute</span></span>\n<span class="line"><span style="color:#A6ACCD;">│  └─ guide</span></span>\n<span class="line"><span style="color:#A6ACCD;">│  └─ index.md</span></span>\n<span class="line"><span style="color:#A6ACCD;">└─ package.json</span></span></code></pre></div><p>The <code>docs</code> directory is the <strong>project root</strong> of the Website. The <code>.vitepress</code> directory is a reserved location for VitePress&#39; config file, dev server cache, build output, and optional theme customization code.</p><h3 id="the-config-file" tabindex="-1">The Config File <a class="header-anchor" href="#the-config-file" aria-label="Permalink to &quot;The Config File&quot;">​</a></h3><p>The config file (<code>.vitepress/config.ts</code>) allows you to customize various aspects of your Website, with the most basic options being the title and description of the site:</p><div class="language-js"><button title="Copy Code" class="copy"></button><span class="lang">js</span><pre class="shiki material-theme-palenight"><code><span class="line"><span style="color:#676E95;font-style:italic;">// .vitepress/config.ts</span></span>\n<span class="line"><span style="color:#89DDFF;font-style:italic;">export</span><span style="color:#A6ACCD;"> </span><span style="color:#89DDFF;font-style:italic;">default</span><span style="color:#A6ACCD;"> </span><span style="color:#89DDFF;">{</span></span>\n<span class="line"><span style="color:#89DDFF;">  </span><span style="color:#676E95;font-style:italic;">// site-level options</span></span>\n<span class="line"><span style="color:#A6ACCD;">  </span><span style="color:#F07178;">title</span><span style="color:#89DDFF;">:</span><span style="color:#A6ACCD;"> </span><span style="color:#89DDFF;">&#39;</span><span style="color:#C3E88D;">OSAVC</span><span style="color:#89DDFF;">&#39;</span><span style="color:#89DDFF;">,</span></span>\n<span class="line"><span style="color:#A6ACCD;">  </span><span style="color:#F07178;">description</span><span style="color:#89DDFF;">:</span><span style="color:#A6ACCD;"> </span><span style="color:#89DDFF;">&#39;</span><span style="color:#C3E88D;">Open Source Autonomous vehical Controller.</span><span style="color:#89DDFF;">&#39;</span><span style="color:#89DDFF;">,</span></span>\n<span class="line"></span>\n<span class="line"><span style="color:#A6ACCD;">  </span><span style="color:#F07178;">themeConfig</span><span style="color:#89DDFF;">:</span><span style="color:#A6ACCD;"> </span><span style="color:#89DDFF;">{</span></span>\n<span class="line"><span style="color:#89DDFF;">    </span><span style="color:#676E95;font-style:italic;">// theme-level options</span></span>\n<span class="line"><span style="color:#A6ACCD;">  </span><span style="color:#89DDFF;">}</span></span>\n<span class="line"><span style="color:#89DDFF;">}</span></span></code></pre></div><h3 id="source-files" tabindex="-1">Source Files <a class="header-anchor" href="#source-files" aria-label="Permalink to &quot;Source Files&quot;">​</a></h3><p>Markdown files outside the <code>.vitepress</code> directory are considered <strong>source files</strong>.</p><p>VitePress uses <strong>file-based routing</strong>: each <code>.md</code> file is compiled into a corresponding <code>.html</code> file with the same path. For example, <code>index.md</code> will be compiled into <code>index.html</code>, and can be visited at the root path <code>/</code> of the resulting VitePress site.</p><p>VitePress also provides the ability to generate clean URLs, rewrite paths, and dynamically generate pages. These will be covered in the <a href="./../website/Routing">Routing Guide</a>.</p><h2 id="up-and-running" tabindex="-1">Up and Running <a class="header-anchor" href="#up-and-running" aria-label="Permalink to &quot;Up and Running&quot;">​</a></h2><p>The &quot;scripts&quot; property of your <code>package.json</code> file supports a number of built-in scripts and their preset life cycle events as well as arbitrary scripts.</p><div class="language-json"><button title="Copy Code" class="copy"></button><span class="lang">json</span><pre class="shiki material-theme-palenight"><code><span class="line"><span style="color:#89DDFF;">{</span></span>\n<span class="line"><span style="color:#A6ACCD;">  ...</span></span>\n<span class="line"><span style="color:#A6ACCD;">  </span><span style="color:#89DDFF;">&quot;</span><span style="color:#C792EA;">scripts</span><span style="color:#89DDFF;">&quot;</span><span style="color:#89DDFF;">:</span><span style="color:#A6ACCD;"> </span><span style="color:#89DDFF;">{</span></span>\n<span class="line"><span style="color:#A6ACCD;">    </span><span style="color:#89DDFF;">&quot;</span><span style="color:#FFCB6B;">docs:dev</span><span style="color:#89DDFF;">&quot;</span><span style="color:#89DDFF;">:</span><span style="color:#A6ACCD;"> </span><span style="color:#89DDFF;">&quot;</span><span style="color:#C3E88D;">vitepress dev docs</span><span style="color:#89DDFF;">&quot;</span><span style="color:#89DDFF;">,</span></span>\n<span class="line"><span style="color:#A6ACCD;">    </span><span style="color:#89DDFF;">&quot;</span><span style="color:#FFCB6B;">docs:build</span><span style="color:#89DDFF;">&quot;</span><span style="color:#89DDFF;">:</span><span style="color:#A6ACCD;"> </span><span style="color:#89DDFF;">&quot;</span><span style="color:#C3E88D;">vitepress build docs</span><span style="color:#89DDFF;">&quot;</span><span style="color:#89DDFF;">,</span></span>\n<span class="line"><span style="color:#A6ACCD;">    </span><span style="color:#89DDFF;">&quot;</span><span style="color:#FFCB6B;">docs:preview</span><span style="color:#89DDFF;">&quot;</span><span style="color:#89DDFF;">:</span><span style="color:#A6ACCD;"> </span><span style="color:#89DDFF;">&quot;</span><span style="color:#C3E88D;">vitepress preview docs</span><span style="color:#89DDFF;">&quot;</span></span>\n<span class="line"><span style="color:#A6ACCD;">  </span><span style="color:#89DDFF;">},</span></span>\n<span class="line"><span style="color:#A6ACCD;">  ...</span></span>\n<span class="line"><span style="color:#89DDFF;">}</span></span></code></pre></div><p>The <code>docs:dev</code> script will start a local dev server with instant hot updates. Run it with the following command:</p><div class="vp-code-group"><div class="tabs"><input type="radio" name="group-X9Ej8" id="tab-gF1NYkE" checked="checked"><label for="tab-gF1NYkE">npm</label><input type="radio" name="group-X9Ej8" id="tab-BALzW1A"><label for="tab-BALzW1A">pnpm</label><input type="radio" name="group-X9Ej8" id="tab-GMi0FeJ"><label for="tab-GMi0FeJ">yarn</label></div><div class="blocks"><div class="language-sh active"><button title="Copy Code" class="copy"></button><span class="lang">sh</span><pre class="shiki material-theme-palenight"><code><span class="line"><span style="color:#FFCB6B;">$</span><span style="color:#A6ACCD;"> </span><span style="color:#C3E88D;">npm</span><span style="color:#A6ACCD;"> </span><span style="color:#C3E88D;">run</span><span style="color:#A6ACCD;"> </span><span style="color:#C3E88D;">docs:dev</span></span></code></pre></div><div class="language-sh"><button title="Copy Code" class="copy"></button><span class="lang">sh</span><pre class="shiki material-theme-palenight"><code><span class="line"><span style="color:#FFCB6B;">$</span><span style="color:#A6ACCD;"> </span><span style="color:#C3E88D;">pnpm</span><span style="color:#A6ACCD;"> </span><span style="color:#C3E88D;">run</span><span style="color:#A6ACCD;"> </span><span style="color:#C3E88D;">docs:dev</span></span></code></pre></div><div class="language-sh"><button title="Copy Code" class="copy"></button><span class="lang">sh</span><pre class="shiki material-theme-palenight"><code><span class="line"><span style="color:#FFCB6B;">$</span><span style="color:#A6ACCD;"> </span><span style="color:#C3E88D;">yarn</span><span style="color:#A6ACCD;"> </span><span style="color:#C3E88D;">docs:dev</span></span></code></pre></div></div></div><p>The dev server should be running at <code>http://localhost:5173</code>. Visit the URL in your browser to see your new site in action!</p>', 32);
const _hoisted_33 = [
  _hoisted_1
];
function _sfc_render(_ctx, _cache, $props, $setup, $data, $options) {
  return openBlock(), createElementBlock("div", null, _hoisted_33);
}
const gettingStarted = /* @__PURE__ */ _export_sfc(_sfc_main, [["render", _sfc_render]]);
export {
  __pageData,
  gettingStarted as default
};
