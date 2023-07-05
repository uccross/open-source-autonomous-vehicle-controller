import { _ as _export_sfc, o as openBlock, c as createElementBlock, V as createStaticVNode } from "./chunks/framework.dd7eec84.js";
const __pageData = JSON.parse('{"title":"Asset Handling","description":"","frontmatter":{},"headers":[],"relativePath":"contribute/website/assetHandeling.md","filePath":"contribute/website/assetHandeling.md"}');
const _sfc_main = { name: "contribute/website/assetHandeling.md" };
const _hoisted_1 = /* @__PURE__ */ createStaticVNode('<h1 id="asset-handling" tabindex="-1">Asset Handling <a class="header-anchor" href="#asset-handling" aria-label="Permalink to &quot;Asset Handling&quot;">​</a></h1><h2 id="referencing-static-assets" tabindex="-1">Referencing Static Assets <a class="header-anchor" href="#referencing-static-assets" aria-label="Permalink to &quot;Referencing Static Assets&quot;">​</a></h2><p>All Markdown files are compiled into Vue components and processed by <a href="https://vitejs.dev/guide/assets.html" target="_blank" rel="noreferrer">Vite</a>. You can, <strong>and should</strong>, reference any assets using relative URLs:</p><div class="language-md"><button title="Copy Code" class="copy"></button><span class="lang">md</span><pre class="shiki material-theme-palenight"><code><span class="line"><span style="color:#89DDFF;">![</span><span style="color:#C3E88D;">An image</span><span style="color:#89DDFF;">](</span><span style="color:#A6ACCD;text-decoration:underline;">./image.png</span><span style="color:#89DDFF;">)</span></span></code></pre></div><p>You can reference static assets in your markdown files, your <code>*.vue</code> components in the theme, styles and plain <code>.css</code> files either using absolute public paths (based on project root) or relative paths (based on your file system). The latter is similar to the behavior you are used to if you have used Vite, Vue CLI, or webpack&#39;s <code>file-loader</code>.</p><p>Common image, media, and font filetypes are detected and included as assets automatically.</p><p>All referenced assets, including those using absolute paths, will be copied to the output directory with a hashed file name in the production build. Never-referenced assets will not be copied. Image assets smaller than 4kb will be base64 inlined - this can be configured via the <code>vite</code> config option.</p><p>All <strong>static</strong> path references, including absolute paths, should be based on your working directory structure.</p>', 8);
const _hoisted_9 = [
  _hoisted_1
];
function _sfc_render(_ctx, _cache, $props, $setup, $data, $options) {
  return openBlock(), createElementBlock("div", null, _hoisted_9);
}
const assetHandeling = /* @__PURE__ */ _export_sfc(_sfc_main, [["render", _sfc_render]]);
export {
  __pageData,
  assetHandeling as default
};
