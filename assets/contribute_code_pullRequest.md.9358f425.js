import { _ as _export_sfc, o as openBlock, c as createElementBlock, V as createStaticVNode } from "./chunks/framework.4f38d4a2.js";
const __pageData = JSON.parse('{"title":"Review the pull request","description":"","frontmatter":{},"headers":[],"relativePath":"contribute/code/pullRequest.md"}');
const _sfc_main = { name: "contribute/code/pullRequest.md" };
const _hoisted_1 = /* @__PURE__ */ createStaticVNode('<h1 id="review-the-pull-request" tabindex="-1">Review the pull request <a class="header-anchor" href="#review-the-pull-request" aria-label="Permalink to &quot;Review the pull request&quot;">​</a></h1><p>You have now created a pull request, which is stored in the project&#39;s repository (not in your fork of the repository). It&#39;s a good idea to read through what you wrote, as well as clicking on the <strong>Commits</strong> tab and the <strong>Files changed</strong> tab to review the contents of your pull request</p><p>If you realize that you left out some important details, you can click the 3 dots in the upper right corner to edit your pull request description.</p><h2 id="add-more-commits-to-your-pull-request" tabindex="-1">Add more commits to your pull request <a class="header-anchor" href="#add-more-commits-to-your-pull-request" aria-label="Permalink to &quot;Add more commits to your pull request&quot;">​</a></h2><p>You can continue to add more commits to your pull request even after opening it! For example, the project maintainers may ask you to make some changes, or you may just think of a change that you forgot to include:</p><p>Start by returning to your local repository, and use <strong>git branch</strong> to see which branch is currently checked out. If you are currently in the master branch (rather than the branch you created), then use <strong>git checkout BRANCH_NAME</strong> to switch. For example, I used git checkout doc.</p><p>Then, you should repeat given above:</p><ul><li>make changes, commit them, and push them to your fork.</li></ul><p>Finally, return to your open pull request on GitHub and refresh the page. You will see that your new commits have automatically been added to the pull request:</p><h2 id="discuss-the-pull-request" tabindex="-1">Discuss the pull request <a class="header-anchor" href="#discuss-the-pull-request" aria-label="Permalink to &quot;Discuss the pull request&quot;">​</a></h2><p>If there are questions or discussion about your pull request from the project maintainers, you can add to the conversation using the comment box at the bottom of the pull request:</p><p>If there are inline comments about specific changes you made, you can respond to those as well:</p><p>Click the Resolve conversation button once you have addressed any specific requests.</p><h2 id="delete-your-branch-from-your-fork" tabindex="-1">Delete your branch from your fork <a class="header-anchor" href="#delete-your-branch-from-your-fork" aria-label="Permalink to &quot;Delete your branch from your fork&quot;">​</a></h2><p>If the project maintainers accept your pull request (congratulations!), they will merge your proposed changes into the project&#39;s master branch and close your pull request:</p><p>You will be given the option to delete your branch from your fork, since it&#39;s no longer of any use:</p><h2 id="delete-your-branch-from-your-local-repository" tabindex="-1">Delete your branch from your local repository <a class="header-anchor" href="#delete-your-branch-from-your-local-repository" aria-label="Permalink to &quot;Delete your branch from your local repository&quot;">​</a></h2><p>You should also delete the branch you created from your local repository, so that you don&#39;t accidentally start working in it the next time you want to make a contribution to this project.</p><p>First, switch to the master branch: git checkout master.</p><p>Then, delete the branch you created: git branch -D BRANCH_NAME. For example, I used git branch -D doc</p><h2 id="synchronize-your-fork-with-the-project-repository" tabindex="-1">Synchronize your fork with the project repository <a class="header-anchor" href="#synchronize-your-fork-with-the-project-repository" aria-label="Permalink to &quot;Synchronize your fork with the project repository&quot;">​</a></h2><p>At this point, your fork is out of sync with the project repository&#39;s master branch.</p><p>To get it back in sync, you should first use Git to pull the latest changes from &quot;upstream&quot; (the project repository) into your local repository: <strong>git pull upstream master</strong>.</p><p>Then, push those changes from your local repository to the &quot;origin&quot; (your fork): <strong>git push origin master</strong>.</p><p>If you return to your fork on GitHub, you will see that the master branch is &quot;even&quot; with the project repository&#39;s master branch</p>', 25);
const _hoisted_26 = [
  _hoisted_1
];
function _sfc_render(_ctx, _cache, $props, $setup, $data, $options) {
  return openBlock(), createElementBlock("div", null, _hoisted_26);
}
const pullRequest = /* @__PURE__ */ _export_sfc(_sfc_main, [["render", _sfc_render]]);
export {
  __pageData,
  pullRequest as default
};
