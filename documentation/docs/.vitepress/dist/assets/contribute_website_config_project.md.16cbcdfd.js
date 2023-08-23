import { _ as _export_sfc, o as openBlock, c as createElementBlock, V as createStaticVNode } from "./chunks/framework.ce59e187.js";
const _imports_0 = "/open-source-autonomous-vehicle-controller/assets/projectPosts.f4d4483d.png";
const _imports_1 = "/open-source-autonomous-vehicle-controller/assets/projectdataDotJsonFile.465a0558.png";
const _imports_2 = "/open-source-autonomous-vehicle-controller/assets/projectPage.c02699a6.png";
const __pageData = JSON.parse('{"title":"Adding a Project on the website.","description":"","frontmatter":{},"headers":[],"relativePath":"contribute/website/config/project.md"}');
const _sfc_main = { name: "contribute/website/config/project.md" };
const _hoisted_1 = /* @__PURE__ */ createStaticVNode('<h1 id="adding-a-project-on-the-website" tabindex="-1">Adding a Project on the website. <a class="header-anchor" href="#adding-a-project-on-the-website" aria-label="Permalink to &quot;Adding a Project on the website.&quot;">​</a></h1><ul><li><p>Navigate inside the OSAVC git repo which we have cloned</p></li><li><p>documentation -&gt; docs -&gt; projects -&gt; posts</p></li></ul><div class="language-"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki material-theme-palenight"><code><span class="line"><span style="color:#A6ACCD;">.</span></span>\n<span class="line"><span style="color:#A6ACCD;">├─ docs              &lt;---</span></span>\n<span class="line"><span style="color:#A6ACCD;">│  ├─ .vitepress</span></span>\n<span class="line"><span style="color:#A6ACCD;">│  │  └─ config.js</span></span>\n<span class="line"><span style="color:#A6ACCD;">│  ├─ about</span></span>\n<span class="line"><span style="color:#A6ACCD;">│  ├─ assets</span></span>\n<span class="line"><span style="color:#A6ACCD;">│  ├─ blog</span></span>\n<span class="line"><span style="color:#A6ACCD;">│  └─ contribute</span></span>\n<span class="line"><span style="color:#A6ACCD;">│  └─ guide</span></span>\n<span class="line"><span style="color:#A6ACCD;">│  └─ projects       &lt;---</span></span>\n<span class="line"><span style="color:#A6ACCD;">│  |  └─ posts       &lt;---</span></span>\n<span class="line"><span style="color:#A6ACCD;">│  └─ index.md</span></span>\n<span class="line"><span style="color:#A6ACCD;">└─ package.json</span></span></code></pre></div><p>Inside the folder create your Markdown file in format <strong>name.md</strong>.</p><p>example : <strong>Vehicle.md</strong></p><p>if you are using a IDE like VsCode you can see the file structure as shown below.</p><p><img src="' + _imports_0 + '" alt="Projects Posts Navigation"></p><p>Inside the <strong>vehicle.md</strong> you can write the details of your project using Markdown language.</p><div class="language-md"><button title="Copy Code" class="copy"></button><span class="lang">md</span><pre class="shiki material-theme-palenight"><code><span class="line"><span style="color:#89DDFF;"># </span><span style="color:#FFCB6B;">Name of your Project</span></span>\n<span class="line"></span>\n<span class="line"><span style="color:#FF9CAC;font-style:italic;">&gt;</span><span style="color:#89DDFF;font-style:italic;"> Short description of your project.</span></span>\n<span class="line"></span>\n<span class="line"><span style="color:#89DDFF;">## </span><span style="color:#FFCB6B;">Detailed Explanation of the project</span></span></code></pre></div><p>Now we have successfully completed creating the md file for our Project.</p><p>Now for listing it on our website we need to add the data of our project to the <strong>data.json</strong> file which you can find inside the projects folder.</p><p><img src="' + _imports_1 + '" alt="data.json File"></p><p>The <strong>data.json</strong> file will contain information like</p><div class="language-json"><button title="Copy Code" class="copy"></button><span class="lang">json</span><pre class="shiki material-theme-palenight"><code><span class="line"><span style="color:#89DDFF;">[</span></span>\n<span class="line"><span style="color:#A6ACCD;">  </span><span style="color:#89DDFF;">{</span></span>\n<span class="line"><span style="color:#A6ACCD;">    </span><span style="color:#89DDFF;">&quot;</span><span style="color:#C792EA;">author</span><span style="color:#89DDFF;">&quot;</span><span style="color:#89DDFF;">:</span><span style="color:#A6ACCD;"> </span><span style="color:#89DDFF;">&quot;</span><span style="color:#C3E88D;">Author Name</span><span style="color:#89DDFF;">&quot;</span><span style="color:#89DDFF;">,</span></span>\n<span class="line"><span style="color:#A6ACCD;">    </span><span style="color:#89DDFF;">&quot;</span><span style="color:#C792EA;">image</span><span style="color:#89DDFF;">&quot;</span><span style="color:#89DDFF;">:</span><span style="color:#A6ACCD;"> </span><span style="color:#89DDFF;">&quot;</span><span style="color:#C3E88D;">Link of the image you want to add on the ArticleCard</span><span style="color:#89DDFF;">&quot;</span><span style="color:#89DDFF;">,</span><span style="color:#A6ACCD;"> </span><span style="color:#676E95;font-style:italic;">// this image has to be hosted online in png format</span></span>\n<span class="line"><span style="color:#A6ACCD;">    </span><span style="color:#676E95;font-style:italic;">// eg: https://i.ibb.co/3Tjw3gv/OSAVC-BOARD.png</span></span>\n<span class="line"><span style="color:#A6ACCD;">    </span><span style="color:#676E95;font-style:italic;">// This Link has the OSAVC Image hosted online</span></span>\n<span class="line"><span style="color:#A6ACCD;">    </span><span style="color:#89DDFF;">&quot;</span><span style="color:#C792EA;">title</span><span style="color:#89DDFF;">&quot;</span><span style="color:#89DDFF;">:</span><span style="color:#A6ACCD;"> </span><span style="color:#89DDFF;">&quot;</span><span style="color:#C3E88D;">Title of your Project</span><span style="color:#89DDFF;">&quot;</span><span style="color:#89DDFF;">,</span></span>\n<span class="line"><span style="color:#A6ACCD;">    </span><span style="color:#89DDFF;">&quot;</span><span style="color:#C792EA;">path</span><span style="color:#89DDFF;">&quot;</span><span style="color:#89DDFF;">:</span><span style="color:#A6ACCD;"> </span><span style="color:#89DDFF;">&quot;</span><span style="color:#C3E88D;">./posts/post1</span><span style="color:#89DDFF;">&quot;</span><span style="color:#89DDFF;">,</span><span style="color:#A6ACCD;">  </span><span style="color:#676E95;font-style:italic;">// relative path to your md file</span></span>\n<span class="line"><span style="color:#A6ACCD;">    </span><span style="color:#89DDFF;">&quot;</span><span style="color:#C792EA;">excerpt</span><span style="color:#89DDFF;">&quot;</span><span style="color:#89DDFF;">:</span><span style="color:#A6ACCD;"> </span><span style="color:#89DDFF;">&quot;</span><span style="color:#C3E88D;">Short description to your project which you want to show to the user on the home page of Projects</span><span style="color:#89DDFF;">&quot;</span></span>\n<span class="line"><span style="color:#A6ACCD;">  </span><span style="color:#89DDFF;">},</span></span>\n<span class="line"><span style="color:#89DDFF;">]</span></span></code></pre></div><p>You have to add different section for your project in the above given format.</p><p>This will add your Project as a ArticleCard on the Projects home page.</p><p>You can always view the project on the website by hosting it locally by running the <code>docs:dev</code> script which will start a local dev server with instant hot updates.</p><p>Run the following command on your terminal.</p><div class="vp-code-group"><div class="tabs"><input type="radio" name="group-l3Eqo" id="tab-iKoKP6P" checked="checked"><label for="tab-iKoKP6P">npm</label><input type="radio" name="group-l3Eqo" id="tab-88AvYRY"><label for="tab-88AvYRY">pnpm</label><input type="radio" name="group-l3Eqo" id="tab-9ov_smv"><label for="tab-9ov_smv">yarn</label></div><div class="blocks"><div class="language-sh active"><button title="Copy Code" class="copy"></button><span class="lang">sh</span><pre class="shiki material-theme-palenight"><code><span class="line"><span style="color:#FFCB6B;">npm</span><span style="color:#A6ACCD;"> </span><span style="color:#C3E88D;">run</span><span style="color:#A6ACCD;"> </span><span style="color:#C3E88D;">docs:dev</span></span></code></pre></div><div class="language-sh"><button title="Copy Code" class="copy"></button><span class="lang">sh</span><pre class="shiki material-theme-palenight"><code><span class="line"><span style="color:#FFCB6B;">pnpm</span><span style="color:#A6ACCD;"> </span><span style="color:#C3E88D;">run</span><span style="color:#A6ACCD;"> </span><span style="color:#C3E88D;">docs:dev</span></span></code></pre></div><div class="language-sh"><button title="Copy Code" class="copy"></button><span class="lang">sh</span><pre class="shiki material-theme-palenight"><code><span class="line"><span style="color:#FFCB6B;">yarn</span><span style="color:#A6ACCD;"> </span><span style="color:#C3E88D;">docs:dev</span></span></code></pre></div></div></div><p>The dev server should be running at <code>http://localhost:5173</code>. Visit the URL in your browser to see your changes in action!</p><p>The Projects section should now contain your project as shown below.</p><p><img src="' + _imports_2 + '" alt="Trouble Shooting Section Pages"></p>', 22);
const _hoisted_23 = [
  _hoisted_1
];
function _sfc_render(_ctx, _cache, $props, $setup, $data, $options) {
  return openBlock(), createElementBlock("div", null, _hoisted_23);
}
const project = /* @__PURE__ */ _export_sfc(_sfc_main, [["render", _sfc_render]]);
export {
  __pageData,
  project as default
};
