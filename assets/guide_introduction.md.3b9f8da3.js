import { _ as _export_sfc, o as openBlock, c as createElementBlock, V as createStaticVNode } from "./chunks/framework.ce59e187.js";
const __pageData = JSON.parse('{"title":"Introduction to the OSAVC","description":"","frontmatter":{},"headers":[],"relativePath":"guide/introduction.md"}');
const _sfc_main = { name: "guide/introduction.md" };
const _hoisted_1 = /* @__PURE__ */ createStaticVNode('<h1 id="introduction-to-the-osavc" tabindex="-1">Introduction to the OSAVC <a class="header-anchor" href="#introduction-to-the-osavc" aria-label="Permalink to &quot;Introduction to the OSAVC&quot;">​</a></h1><p>The Open Source Real-time Controller for Resource-constrained Autonomous Vehicles and Systems (OSAVC) is a project aimed at designing and implementing an open-source real-time hardware controller specifically tailored for resource-constrained autonomous vehicles and systems.</p><h2 id="motivation" tabindex="-1">Motivation <a class="header-anchor" href="#motivation" aria-label="Permalink to &quot;Motivation&quot;">​</a></h2><p>The field of autonomous systems is rapidly growing across various domains, from scientific and industrial to military applications. As technology advances, smaller yet more powerful microprocessors and microcontrollers are becoming available. OSAVC addresses the need for a dedicated, modular real-time controller that can be integrated into distributed control architectures for autonomous vehicles and systems.</p><h2 id="problem-statement" tabindex="-1">Problem Statement <a class="header-anchor" href="#problem-statement" aria-label="Permalink to &quot;Problem Statement&quot;">​</a></h2><p>The lack of a vehicle-agnostic, real-time controller for autonomous vehicles and systems is a significant challenge. OSAVC aims to address this by providing a flexible, open-source solution that supports real-time computation, adaptability to different vehicle configurations, and accessibility to a broad community of users and contributors.</p><h2 id="architecture-overview" tabindex="-1">Architecture Overview <a class="header-anchor" href="#architecture-overview" aria-label="Permalink to &quot;Architecture Overview&quot;">​</a></h2><p>OSAVC consists of several key components:</p><ul><li><strong>Real-time Autopilot:</strong> Provides real-time control and computation capabilities, allowing integration with different vehicle types and algorithms.</li><li><strong>Single Board Computer (SBC):</strong> Enables non-real-time tasks and access to advanced computational packages, extending the system&#39;s capabilities beyond real-time functions.</li><li><strong>Tensor Processing Unit (TPU):</strong> Facilitates onboard machine learning capabilities, such as object detection and image classification, without overburdening the SBC.</li></ul><h2 id="features" tabindex="-1">Features <a class="header-anchor" href="#features" aria-label="Permalink to &quot;Features&quot;">​</a></h2><ul><li>Modular and adaptable design for various vehicle types.</li><li>Real-time computation for precise control and estimation algorithms.</li><li>Integration with SBC for non-real-time tasks and advanced computation.</li><li>Onboard machine learning capabilities for object detection and classification.</li><li>Open-source hardware and firmware, promoting collaboration and innovation.</li></ul><h2 id="getting-started" tabindex="-1">Getting Started <a class="header-anchor" href="#getting-started" aria-label="Permalink to &quot;Getting Started&quot;">​</a></h2><p>For detailed instructions on setting up and using the OSAVC controller, please refer to the <a href="./FirstProject">Building your first projects section</a>.</p><h2 id="contributions" tabindex="-1">Contributions <a class="header-anchor" href="#contributions" aria-label="Permalink to &quot;Contributions&quot;">​</a></h2><p>Contributions to the OSAVC project are welcome! If you&#39;re interested in contributing, please review the <a href="./../contribute/code/getting-started">Contribution Guidelines</a> for more information.</p>', 15);
const _hoisted_16 = [
  _hoisted_1
];
function _sfc_render(_ctx, _cache, $props, $setup, $data, $options) {
  return openBlock(), createElementBlock("div", null, _hoisted_16);
}
const introduction = /* @__PURE__ */ _export_sfc(_sfc_main, [["render", _sfc_render]]);
export {
  __pageData,
  introduction as default
};