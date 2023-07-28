import { _ as _export_sfc, o as openBlock, c as createElementBlock, V as createStaticVNode } from "./chunks/framework.ce59e187.js";
const _imports_0 = "/open-source-autonomous-vehicle-controller/assets/image1.e60da5a1.png";
const _imports_1 = "/open-source-autonomous-vehicle-controller/assets/image2.868e9e1e.png";
const _imports_2 = "/open-source-autonomous-vehicle-controller/assets/image3.d1fb97af.png";
const _imports_3 = "/open-source-autonomous-vehicle-controller/assets/image4.eaf8458a.png";
const _imports_4 = "/open-source-autonomous-vehicle-controller/assets/image5.ba7c09b6.png";
const _imports_5 = "/open-source-autonomous-vehicle-controller/assets/image6.f333867e.png";
const _imports_6 = "/open-source-autonomous-vehicle-controller/assets/image7.b3d0a01c.png";
const _imports_7 = "/open-source-autonomous-vehicle-controller/assets/image8.da7c76d4.png";
const _imports_8 = "/open-source-autonomous-vehicle-controller/assets/image9.4fcb8125.png";
const _imports_9 = "/open-source-autonomous-vehicle-controller/assets/image10.2f07772a.png";
const __pageData = JSON.parse('{"title":"Creating RC_ESC.X","description":"","frontmatter":{},"headers":[],"relativePath":"guide/CreatingRC_ESC_X.md"}');
const _sfc_main = { name: "guide/CreatingRC_ESC_X.md" };
const _hoisted_1 = /* @__PURE__ */ createStaticVNode('<h1 id="creating-rc-esc-x" tabindex="-1">Creating RC_ESC.X <a class="header-anchor" href="#creating-rc-esc-x" aria-label="Permalink to &quot;Creating RC_ESC.X&quot;">​</a></h1><ul><li>File -&gt; New Project to create a new MIcrochip Embedded Standalone Project</li></ul><p><img src="' + _imports_0 + '" alt="New Project"></p><ul><li>Click Next and make sure to select PIC32MX795F512L as the Device and the PICkit3 as the Tool</li></ul><p><img src="' + _imports_1 + '" alt="PiCkit3"></p><ul><li>Click Next and select XC32 v4.21 as the compiler</li></ul><p><img src="' + _imports_2 + '" alt="XC32 v4.21"></p><ul><li>Click Next and set the Project Name to what you want to name the library and the Project Location as open-source-autonomous-vehicle-controller/lib</li></ul><p><img src="' + _imports_3 + '" alt="Project Name"></p><ul><li>Click Finish and create new xc32_header.h files under the Header Files folder by right clicking the folder -&gt; clicking New -&gt; xc32_header.h</li></ul><p><img src="' + _imports_4 + '" alt="xc32_header"></p><ul><li>Create new xc32_newfile.c files under the Source Files folder by right clicking the folder -&gt; clicking New -&gt; xc32_newfile.</li></ul><p><img src="' + _imports_5 + '" alt="xc32_header"></p><ul><li>It is good practice to include a test harness for your device defined within an #ifdef directive in your new .c file.</li></ul><p><img src="' + _imports_6 + '" alt="c file"></p><ul><li>Navigate to File -&gt; Project Properties -&gt; XC32 (Global Options) -&gt; xc32-gcc. Choose Preprocessing and messages as the option categories.</li><li>Click the three dots next to Preprocessor macros and enter the macro that your #ifdef directive checks in your test harness. In this case, the macro will be RCESC_TESTING</li></ul><p><img src="' + _imports_7 + '" alt="RCESC_TESTING"></p><ul><li>Click OK to save</li><li>Click the three dots next to Include directories to enter the relative paths of all the libraries that you will use in your new project. In this case, Board.X, Serial.X, and System_timer.X</li></ul><p><img src="' + _imports_8 + '" alt="Image"></p><ul><li>Click OK to save.</li><li>Add the header files from the libraries that you will be using by right clicking on Header Files -&gt; Add Existing Item -&gt; select the .h file to add. In this case, Board.h, SerialM32.h, System_timer.h</li></ul><p><img src="' + _imports_9 + '" alt="Add existing item"></p><ul><li>Under Source Files, repeat the same process for the .c files that correspond to the .h files you’ve included. In this case, Board.c, SerialM32.c, System_timer.c</li></ul>', 22);
const _hoisted_23 = [
  _hoisted_1
];
function _sfc_render(_ctx, _cache, $props, $setup, $data, $options) {
  return openBlock(), createElementBlock("div", null, _hoisted_23);
}
const CreatingRC_ESC_X = /* @__PURE__ */ _export_sfc(_sfc_main, [["render", _sfc_render]]);
export {
  __pageData,
  CreatingRC_ESC_X as default
};
