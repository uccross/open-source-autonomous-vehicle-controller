import { _ as _export_sfc, o as openBlock, c as createElementBlock, V as createStaticVNode } from "./chunks/framework.dd7eec84.js";
const _imports_0 = "/open-source-autonomous-vehicle-controller/assets/Fork.7c4d6941.png";
const _imports_1 = "/open-source-autonomous-vehicle-controller/assets/YourFork.29038bf6.png";
const _imports_2 = "/open-source-autonomous-vehicle-controller/assets/Clone.95a09b5c.png";
const __pageData = JSON.parse('{"title":"Getting Started","description":"","frontmatter":{},"headers":[],"relativePath":"contribute/code/getting-started.md","filePath":"contribute/code/getting-started.md"}');
const _sfc_main = { name: "contribute/code/getting-started.md" };
const _hoisted_1 = /* @__PURE__ */ createStaticVNode('<h1 id="getting-started" tabindex="-1">Getting Started <a class="header-anchor" href="#getting-started" aria-label="Permalink to &quot;Getting Started&quot;">​</a></h1><h2 id="sign-into-github" tabindex="-1">Sign into GitHub <a class="header-anchor" href="#sign-into-github" aria-label="Permalink to &quot;Sign into GitHub&quot;">​</a></h2><p>Sign into your GitHub account, or <a href="https://github.com/join?ref=dataschool.io" target="_blank" rel="noreferrer">create a free GitHub account</a> if you don&#39;t have one.</p><h2 id="fork-the-project-repository" tabindex="-1">Fork the project repository <a class="header-anchor" href="#fork-the-project-repository" aria-label="Permalink to &quot;Fork the project repository&quot;">​</a></h2><p>Go to the <a href="https://github.com/uccross/open-source-autonomous-vehicle-controller" target="_blank" rel="noreferrer">OSAVC project&#39;s repository</a> on GitHub, and then &quot;fork&quot; it by clicking the Fork button in the upper right corner: <img src="' + _imports_0 + '" alt="Fork OSAVC Repo"></p><p>This creates a copy of the project repository in your GitHub account. In the upper left corner, you will see that you are now looking at a repository in your account:</p><p><img src="' + _imports_1 + '" alt="Your Fork"></p><h2 id="clone-your-fork" tabindex="-1">Clone your fork <a class="header-anchor" href="#clone-your-fork" aria-label="Permalink to &quot;Clone your fork&quot;">​</a></h2><p>While still in your repository, click the green Clone or download button and then copy the HTTPS URL:</p><p><img src="' + _imports_2 + '" alt="Clone your fork"></p><p>Using Git on your local machine, clone your fork using the URL you just copied:</p><div class="language-bash"><button title="Copy Code" class="copy"></button><span class="lang">bash</span><pre class="shiki material-theme-palenight"><code><span class="line"><span style="color:#FFCB6B;">git</span><span style="color:#A6ACCD;"> </span><span style="color:#C3E88D;">clone</span><span style="color:#A6ACCD;"> </span><span style="color:#C3E88D;">URL_OF_FORK.</span></span></code></pre></div><p>For example, I used</p><div class="language-bash"><button title="Copy Code" class="copy"></button><span class="lang">bash</span><pre class="shiki material-theme-palenight"><code><span class="line"><span style="color:#FFCB6B;">git</span><span style="color:#A6ACCD;"> </span><span style="color:#C3E88D;">clone</span><span style="color:#A6ACCD;"> </span><span style="color:#C3E88D;">https://github.com/Aniruddha1261/open-source-autonomous-vehicle-controller.git</span></span></code></pre></div><p>Cloning copies the repository files (and commit history) from GitHub to your local machine. The repository will be downloaded into a subdirectory of your working directory, and the subdirectory will have the same name as the repository.</p><p>(If you run into problems during this step, read the <a href="https://docs.github.com/en/get-started/quickstart/set-up-git?ref=dataschool.io" target="_blank" rel="noreferrer">Set up Git</a> page from GitHub&#39;s documentation.)</p><h2 id="navigate-to-your-local-repository" tabindex="-1">Navigate to your local repository <a class="header-anchor" href="#navigate-to-your-local-repository" aria-label="Permalink to &quot;Navigate to your local repository&quot;">​</a></h2><p>Since the clone was downloaded into a subdirectory of your working directory, you can navigate to it using:</p><div class="language-bash"><button title="Copy Code" class="copy"></button><span class="lang">bash</span><pre class="shiki material-theme-palenight"><code><span class="line"><span style="color:#82AAFF;">cd</span><span style="color:#A6ACCD;"> </span><span style="color:#C3E88D;">NAME_OF_REPOSITORY.</span></span></code></pre></div><p>For example, I used</p><div class="language-bash"><button title="Copy Code" class="copy"></button><span class="lang">bash</span><pre class="shiki material-theme-palenight"><code><span class="line"><span style="color:#82AAFF;">cd</span><span style="color:#A6ACCD;"> </span><span style="color:#C3E88D;">open-source-autonomous-vehicle-controller</span></span></code></pre></div><h2 id="check-that-your-fork-is-the-origin-remote" tabindex="-1">Check that your fork is the &quot;origin&quot; remote <a class="header-anchor" href="#check-that-your-fork-is-the-origin-remote" aria-label="Permalink to &quot;Check that your fork is the &quot;origin&quot; remote&quot;">​</a></h2><p>You are going to be synchronizing your local repository with both the project repository (on GitHub) and your fork (also on GitHub). The URLs that point to these repositories are called &quot;remotes&quot;. More specifically, the project repository is called the &quot;upstream&quot; remote, and your fork is called the &quot;origin&quot; remote.</p><p>When you cloned your fork, that should have automatically set your fork as the &quot;origin&quot; remote. Use <strong>git remote -v</strong> to show your current remotes. You should see the URL of your fork (which you copied in step 3) next to the word &quot;origin&quot;.</p><p>If you don&#39;t see an &quot;origin&quot; remote, you can add it using:</p><div class="language-bash"><button title="Copy Code" class="copy"></button><span class="lang">bash</span><pre class="shiki material-theme-palenight"><code><span class="line"><span style="color:#FFCB6B;">git</span><span style="color:#A6ACCD;"> </span><span style="color:#C3E88D;">remote</span><span style="color:#A6ACCD;"> </span><span style="color:#C3E88D;">add</span><span style="color:#A6ACCD;"> </span><span style="color:#C3E88D;">origin</span><span style="color:#A6ACCD;"> </span><span style="color:#C3E88D;">URL_OF_FORK.</span></span></code></pre></div><p>(If you run into problems during this step, read the <a href="https://docs.github.com/en/get-started/getting-started-with-git/managing-remote-repositories?ref=dataschool.io" target="_blank" rel="noreferrer">Managing remote repositories</a> page from GitHub&#39;s documentation.)</p><h2 id="create-a-new-branch" tabindex="-1">Create a new branch <a class="header-anchor" href="#create-a-new-branch" aria-label="Permalink to &quot;Create a new branch&quot;">​</a></h2><p>Rather than making changes to the project&#39;s &quot;master&quot; branch, it&#39;s a good practice to instead create your own branch. This creates an environment for your work that is isolated from the master branch.</p><p>Use</p><div class="language-bash"><button title="Copy Code" class="copy"></button><span class="lang">bash</span><pre class="shiki material-theme-palenight"><code><span class="line"><span style="color:#FFCB6B;">git</span><span style="color:#A6ACCD;"> </span><span style="color:#C3E88D;">checkout</span><span style="color:#A6ACCD;"> </span><span style="color:#C3E88D;">-b</span><span style="color:#A6ACCD;"> </span><span style="color:#C3E88D;">BRANCH_NAME</span></span></code></pre></div><p>to create a new branch and then immediately switch to it. The name of the branch should briefly describe what you are working on, and should not contain any spaces.</p><p>For example, I used</p><div class="language-bash"><button title="Copy Code" class="copy"></button><span class="lang">bash</span><pre class="shiki material-theme-palenight"><code><span class="line"><span style="color:#FFCB6B;">git</span><span style="color:#A6ACCD;"> </span><span style="color:#C3E88D;">checkout</span><span style="color:#A6ACCD;"> </span><span style="color:#C3E88D;">-b</span><span style="color:#A6ACCD;"> </span><span style="color:#C3E88D;">doc</span></span></code></pre></div><p>because I was making some small fixes to the documentation.</p><p>Use git branch to show your local branches. You should see your new branch as well as &quot;master&quot;, and your new branch should have an asterisk next to it to indicate that it&#39;s &quot;checked out&quot; (meaning that you&#39;re working in it).</p>', 36);
const _hoisted_37 = [
  _hoisted_1
];
function _sfc_render(_ctx, _cache, $props, $setup, $data, $options) {
  return openBlock(), createElementBlock("div", null, _hoisted_37);
}
const gettingStarted = /* @__PURE__ */ _export_sfc(_sfc_main, [["render", _sfc_render]]);
export {
  __pageData,
  gettingStarted as default
};