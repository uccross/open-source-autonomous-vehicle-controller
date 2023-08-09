import { _ as _export_sfc, o as openBlock, c as createElementBlock, V as createStaticVNode } from "./chunks/framework.4f38d4a2.js";
const _imports_0 = "/SRA-Board-Documentation/assets/MPLABX1.d0d35cca.png";
const _imports_1 = "/SRA-Board-Documentation/assets/MPLABX2.4af1e6a9.png";
const _imports_2 = "/SRA-Board-Documentation/assets/MPLABX3.33de2aa1.png";
const __pageData = JSON.parse('{"title":"Setting Up MPLAB X","description":"","frontmatter":{"title":"Setting Up MPLAB X","date":"2023-06-28T00:00:00.000Z","category":"Tutorial","tags":["MPLAB X","Software"]},"headers":[],"relativePath":"blog/posts/post2.md"}');
const _sfc_main = { name: "blog/posts/post2.md" };
const _hoisted_1 = /* @__PURE__ */ createStaticVNode('<p>Problem with MPLAB X ?</p><hr><h2 id="no-such-file-or-directory-for-include-files" tabindex="-1">No Such file or directory for include files <a class="header-anchor" href="#no-such-file-or-directory-for-include-files" aria-label="Permalink to &quot;No Such file or directory for include files&quot;">​</a></h2><ul><li>File -&gt; Project Properties -&gt; xc32-gcc</li></ul><p><img src="' + _imports_0 + '" alt="Image"></p><ul><li>Choose Preprocessing and messages in the option categories</li><li>Click the three dots next to include directories</li></ul><p><img src="' + _imports_1 + '" alt="Image"></p><ul><li><p>Enter the relative folder path and click OK to save</p></li><li><p>A heap is required, but has not been specified.</p><ul><li>File -&gt; Project Properties -&gt; xc32-ld</li><li>Enter 0 bytes as the heap size</li></ul></li></ul><p><img src="' + _imports_2 + '" alt="Image"></p><ul><li>Click OK to save</li></ul>', 10);
const _hoisted_11 = [
  _hoisted_1
];
function _sfc_render(_ctx, _cache, $props, $setup, $data, $options) {
  return openBlock(), createElementBlock("div", null, _hoisted_11);
}
const post2 = /* @__PURE__ */ _export_sfc(_sfc_main, [["render", _sfc_render]]);
export {
  __pageData,
  post2 as default
};
