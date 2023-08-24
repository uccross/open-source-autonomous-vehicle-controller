import { A as useRoute, l as computed, d as defineComponent, u as useData, o as openBlock, c as createElementBlock, b as unref, n as normalizeClass, G as createCommentVNode, C as createBaseVNode, t as toDisplayString, _ as _export_sfc, y as withBase, J as createVNode, E as withCtx, a as createTextVNode, F as Fragment, R as renderList, S as pushScopeId, U as popScopeId, D as createBlock, h as ref, Q as inBrowser, H as resolveComponent, a5 as useUpdateHead, a6 as RouterSymbol, a7 as initData, a8 as dataSymbol, a9 as Content, aa as ClientOnly, ab as siteDataRef, ac as createSSRApp, ad as createRouter, ae as pathToFile, M as __vitePreload, q as onMounted, k as watchEffect, af as usePrefetch, ag as useCopyCode, ah as useCodeGroups, ai as h } from "./chunks/framework.ce59e187.js";
import { t as theme$1 } from "./chunks/theme.4c21da17.js";
const style$1 = "";
const data$1 = JSON.parse('[{"title":"Setting Up Hardware","author":"Aaron Hunter","url":"/blog/posts/post1","excerpt":"<blockquote>\\n<p>Getting Trouble Setting up the Hardware ?</p>\\n</blockquote>\\n","tags":["Hardware","Connections"],"category":"Tutorial","date":{"raw":"2023-06-28","time":1687953600000,"formatted":"June 28, 2023","since":"about 2 months ago"}},{"title":"Setting Up MPLAB X","author":"Aaron Hunter","url":"/blog/posts/post2","excerpt":"<p>Problem with MPLAB X ?</p>\\n","tags":["MPLAB X","Software"],"category":"Tutorial","date":{"raw":"2023-06-28","time":1687953600000,"formatted":"June 28, 2023","since":"about 2 months ago"}}]');
function usePosts() {
  const route = useRoute();
  const path = route.path;
  function findCurrentIndex() {
    const result = data$1.findIndex((p) => p.url === route.path);
    if (result === -1)
      console.error(`blog post missing: ${route.path}`);
    return result;
  }
  const post = computed(() => data$1[findCurrentIndex()]);
  const nextPost = computed(() => data$1[findCurrentIndex() - 1]);
  const prevPost = computed(() => data$1[findCurrentIndex() + 1]);
  return { posts: data$1, post, nextPost, prevPost, path };
}
const _sfc_main$i = /* @__PURE__ */ defineComponent({
  __name: "VPBPostCategory",
  props: {
    category: {}
  },
  setup(__props) {
    const { theme: theme2 } = useData();
    return (_ctx, _cache) => {
      var _a;
      return openBlock(), createElementBlock("div", null, [
        ((_a = unref(theme2).blog) == null ? void 0 : _a.categoryIcons) && unref(theme2).blog.categoryIcons[_ctx.category.toLowerCase()] ? (openBlock(), createElementBlock("div", {
          key: 0,
          class: normalizeClass([unref(theme2).blog.categoryIcons[_ctx.category.toLowerCase()], "mr-2"])
        }, null, 2)) : createCommentVNode("", true),
        createBaseVNode("span", null, toDisplayString(_ctx.category), 1)
      ]);
    };
  }
});
const VPBPostCategory_vue_vue_type_style_index_0_scoped_5711fefc_lang = "";
const VPBPostCategory = /* @__PURE__ */ _export_sfc(_sfc_main$i, [["__scopeId", "data-v-5711fefc"]]);
const data = JSON.parse('[{"name":"Aaron Hunter","avatar":"https://avatars.githubusercontent.com/u/7852021?v=4","gravatar":null,"twitter":null,"url":"/blog/authors/Aaron","excerpt":""},{"name":"Carlos Espinosa","avatar":"https://avatars.githubusercontent.com/u/34257319?v=4","gravatar":null,"twitter":"@getanyword","url":"/blog/authors/Carlos","excerpt":""}]');
function useAuthors() {
  const route = useRoute();
  const path = route.path;
  function findByName(name) {
    return data.find((p) => p.name === name);
  }
  function findCurrentIndex() {
    const result = data.findIndex((p) => p.url === route.path);
    if (result === -1)
      console.error(`author page missing: ${route.path}`);
    return result;
  }
  const author = computed(() => data[findCurrentIndex()]);
  const nextAuthor = computed(() => data[findCurrentIndex() - 1]);
  const prevAuthor = computed(() => data[findCurrentIndex() + 1]);
  return { authors: data, author, nextAuthor, prevAuthor, findByName, path };
}
const _hoisted_1$d = {
  key: 0,
  class: "flex items-center space-x-4"
};
const _hoisted_2$b = ["src", "alt"];
const _hoisted_3$a = ["src", "alt"];
const _hoisted_4$8 = ["href"];
const _hoisted_5$7 = { class: "font-medium dark:text-white" };
const _hoisted_6$6 = { key: 1 };
const _sfc_main$h = /* @__PURE__ */ defineComponent({
  __name: "VPBHomeAuthor",
  props: {
    name: {}
  },
  setup(__props) {
    const props = __props;
    useData();
    const { findByName } = useAuthors();
    const author = computed(() => {
      return findByName(props.name);
    });
    return (_ctx, _cache) => {
      return author.value ? (openBlock(), createElementBlock("div", _hoisted_1$d, [
        author.value.avatar ? (openBlock(), createElementBlock("img", {
          key: 0,
          class: "h-7 w-7 rounded-full",
          src: author.value.avatar,
          alt: author.value.name
        }, null, 8, _hoisted_2$b)) : author.value.gravatar ? (openBlock(), createElementBlock("img", {
          key: 1,
          class: "h-7 w-7 rounded-full",
          src: `https://gravatar.com/avatar/${author.value.gravatar}`,
          alt: author.value.name
        }, null, 8, _hoisted_3$a)) : createCommentVNode("", true),
        createBaseVNode("a", {
          href: unref(withBase)(author.value.url),
          class: "inline-flex items-center font-medium hover:text-[color:var(--vp-c-brand-dark)]"
        }, [
          createBaseVNode("span", _hoisted_5$7, toDisplayString(author.value.name), 1)
        ], 8, _hoisted_4$8)
      ])) : (openBlock(), createElementBlock("div", _hoisted_6$6));
    };
  }
});
const _hoisted_1$c = { class: "rounded-lg border border-[color:var(--vp-c-brand-light)] p-6 shadow-md dark:border-[color:var(--vp-c-brand-dark)]" };
const _hoisted_2$a = { class: "mb-5 flex items-center justify-between text-gray-500" };
const _hoisted_3$9 = { class: "bg-primary-100 inline-flex items-center rounded text-sm font-medium text-[color:var(--vp-c-brand-light)] dark:text-[color:var(--vp-c-brand-dark)]" };
const _hoisted_4$7 = { class: "text-sm" };
const _hoisted_5$6 = { class: "mb-2 text-2xl font-bold tracking-tight text-[color:var(--vp-c-brand-light)] dark:text-[color:var(--vp-c-brand-dark)]" };
const _hoisted_6$5 = ["href"];
const _hoisted_7$5 = ["innerHTML"];
const _hoisted_8$5 = { class: "flex items-center justify-between" };
const _hoisted_9$5 = ["href"];
const _hoisted_10$4 = /* @__PURE__ */ createBaseVNode("div", { class: "i-[bx/right-arrow-alt] ml-2" }, null, -1);
const _sfc_main$g = /* @__PURE__ */ defineComponent({
  __name: "VPBHomePost",
  props: {
    post: {}
  },
  setup(__props) {
    return (_ctx, _cache) => {
      return openBlock(), createElementBlock("article", _hoisted_1$c, [
        createBaseVNode("div", _hoisted_2$a, [
          createBaseVNode("span", _hoisted_3$9, [
            createVNode(VPBPostCategory, {
              category: _ctx.post.category
            }, {
              default: withCtx(() => [
                createBaseVNode("span", _hoisted_4$7, toDisplayString(_ctx.post.date.since), 1)
              ]),
              _: 1
            }, 8, ["category"])
          ])
        ]),
        createBaseVNode("h2", _hoisted_5$6, [
          createBaseVNode("a", {
            href: _ctx.post.url
          }, toDisplayString(_ctx.post.title), 9, _hoisted_6$5)
        ]),
        createBaseVNode("div", {
          class: "mb-5 font-light",
          innerHTML: _ctx.post.excerpt
        }, null, 8, _hoisted_7$5),
        createBaseVNode("div", _hoisted_8$5, [
          createVNode(_sfc_main$h, {
            name: _ctx.post.author
          }, null, 8, ["name"]),
          createBaseVNode("a", {
            href: _ctx.post.url,
            class: "inline-flex items-center font-medium hover:text-[color:var(--vp-c-brand-dark)]"
          }, [
            createTextVNode(" Read more "),
            _hoisted_10$4
          ], 8, _hoisted_9$5)
        ])
      ]);
    };
  }
});
const _hoisted_1$b = { class: "mx-auto max-w-screen-xl lg:px-6 lg:py-16" };
const _hoisted_2$9 = { class: "mx-auto mb-8 max-w-screen-sm text-center lg:mb-16" };
const _hoisted_3$8 = { class: "mb-4 text-3xl font-extrabold tracking-tight text-[color:var(--vp-c-brand-light)] dark:text-[color:var(--vp-c-brand-dark)] lg:text-4xl" };
const _hoisted_4$6 = { class: "font-light text-[color:var(--vp-c-text-light-1)] dark:text-[color:var(--vp-c-text-dark-1)] sm:text-xl" };
const _hoisted_5$5 = { class: "grid gap-6 p-2 lg:grid-cols-2" };
const _sfc_main$f = /* @__PURE__ */ defineComponent({
  __name: "VPBHome",
  setup(__props) {
    const { posts } = usePosts();
    const { theme: theme2 } = useData();
    return (_ctx, _cache) => {
      var _a, _b;
      return openBlock(), createElementBlock("div", _hoisted_1$b, [
        createBaseVNode("div", _hoisted_2$9, [
          createBaseVNode("h2", _hoisted_3$8, toDisplayString((_a = unref(theme2).blog) == null ? void 0 : _a.title), 1),
          createBaseVNode("p", _hoisted_4$6, toDisplayString((_b = unref(theme2).blog) == null ? void 0 : _b.description), 1)
        ]),
        createBaseVNode("div", _hoisted_5$5, [
          (openBlock(true), createElementBlock(Fragment, null, renderList(unref(posts), (post) => {
            return openBlock(), createElementBlock("div", {
              key: post.url
            }, [
              createVNode(_sfc_main$g, { post }, null, 8, ["post"])
            ]);
          }), 128))
        ])
      ]);
    };
  }
});
const _hoisted_1$a = /* @__PURE__ */ createBaseVNode("dt", { class: "sr-only" }, "Published on", -1);
const _hoisted_2$8 = { class: "text-base font-medium leading-6 text-gray-500 dark:text-gray-300" };
const _hoisted_3$7 = ["datetime"];
const _sfc_main$e = /* @__PURE__ */ defineComponent({
  __name: "VPBPostDate",
  setup(__props) {
    const { post } = usePosts();
    function getDateTime() {
      return new Date(post.value.date.time).toISOString();
    }
    return (_ctx, _cache) => {
      return openBlock(), createElementBlock("dl", null, [
        _hoisted_1$a,
        createBaseVNode("dd", _hoisted_2$8, [
          createBaseVNode("time", {
            datetime: getDateTime()
          }, toDisplayString(unref(post).date.formatted), 9, _hoisted_3$7)
        ])
      ]);
    };
  }
});
const _withScopeId$1 = (n) => (pushScopeId("data-v-f2d4f8ad"), n = n(), popScopeId(), n);
const _hoisted_1$9 = /* @__PURE__ */ _withScopeId$1(() => /* @__PURE__ */ createBaseVNode("dt", { class: "sr-only" }, "Authors", -1));
const _hoisted_2$7 = { class: "flex justify-center space-x-8 sm:space-x-12 xl:block xl:space-x-0 xl:space-y-8" };
const _hoisted_3$6 = {
  key: 0,
  class: "flex items-center space-x-2"
};
const _hoisted_4$5 = ["src"];
const _hoisted_5$4 = ["src"];
const _hoisted_6$4 = { class: "whitespace-nowrap text-sm font-medium leading-5" };
const _hoisted_7$4 = /* @__PURE__ */ _withScopeId$1(() => /* @__PURE__ */ createBaseVNode("dt", { class: "sr-only" }, "Name", -1));
const _hoisted_8$4 = { class: "text-gray-900 dark:text-white" };
const _hoisted_9$4 = ["href"];
const _hoisted_10$3 = {
  key: 0,
  class: "sr-only"
};
const _hoisted_11$2 = { key: 1 };
const _hoisted_12$2 = ["href"];
const _sfc_main$d = /* @__PURE__ */ defineComponent({
  __name: "VPBPostAuthor",
  props: {
    insideDoc: { type: Boolean }
  },
  setup(__props) {
    const { findByName } = useAuthors();
    const { post } = usePosts();
    const author = computed(() => {
      return findByName(post.value.author);
    });
    return (_ctx, _cache) => {
      return openBlock(), createElementBlock("dl", {
        class: normalizeClass(["pb-10 pt-6 xl:border-b xl:border-gray-200 xl:pt-11 dark:xl:border-slate-200/5", { "xs:show xl:hidden": _ctx.insideDoc }])
      }, [
        _hoisted_1$9,
        createBaseVNode("dd", null, [
          createBaseVNode("ul", _hoisted_2$7, [
            author.value ? (openBlock(), createElementBlock("li", _hoisted_3$6, [
              author.value.gravatar ? (openBlock(), createElementBlock("img", {
                key: 0,
                src: `https://gravatar.com/avatar/${author.value.gravatar}`,
                alt: "author image",
                class: "h-10 w-10 rounded-full"
              }, null, 8, _hoisted_4$5)) : author.value.avatar ? (openBlock(), createElementBlock("img", {
                key: 1,
                src: author.value.avatar,
                alt: "author image",
                class: "h-10 w-10 rounded-full"
              }, null, 8, _hoisted_5$4)) : createCommentVNode("", true),
              createBaseVNode("dl", _hoisted_6$4, [
                _hoisted_7$4,
                createBaseVNode("dd", _hoisted_8$4, [
                  createBaseVNode("a", {
                    href: unref(withBase)(author.value.url),
                    class: "text-lg text-gray-900 hover:text-[color:var(--vp-c-brand-light)] dark:text-white dark:hover:text-[color:var(--vp-c-brand-dark)]"
                  }, toDisplayString(author.value.name), 9, _hoisted_9$4)
                ]),
                author.value.twitter ? (openBlock(), createElementBlock("dt", _hoisted_10$3, "Twitter")) : createCommentVNode("", true),
                author.value.twitter ? (openBlock(), createElementBlock("dd", _hoisted_11$2, [
                  createBaseVNode("a", {
                    href: `https://twitter.com/${author.value.twitter}`,
                    target: "_blank",
                    rel: "noopener noreferrer"
                  }, "@" + toDisplayString(author.value.twitter), 9, _hoisted_12$2)
                ])) : createCommentVNode("", true)
              ])
            ])) : createCommentVNode("", true)
          ])
        ])
      ], 2);
    };
  }
});
const VPBPostAuthor_vue_vue_type_style_index_0_scoped_f2d4f8ad_lang = "";
const VPBPostAuthor = /* @__PURE__ */ _export_sfc(_sfc_main$d, [["__scopeId", "data-v-f2d4f8ad"]]);
const _hoisted_1$8 = { class: "bg-primary-100 inline-flex items-center rounded text-sm font-medium" };
const _sfc_main$c = /* @__PURE__ */ defineComponent({
  __name: "VPBPostDetails",
  props: {
    insideDoc: { type: Boolean }
  },
  setup(__props) {
    const { post } = usePosts();
    return (_ctx, _cache) => {
      return openBlock(), createElementBlock(Fragment, null, [
        createBaseVNode("div", {
          class: normalizeClass(["flex justify-center space-x-8 sm:space-x-12 xl:block xl:space-x-0 xl:space-y-8", { "xs:show xl:hidden": _ctx.insideDoc }])
        }, [
          createBaseVNode("span", _hoisted_1$8, [
            createVNode(VPBPostCategory, {
              category: unref(post).category
            }, null, 8, ["category"])
          ])
        ], 2),
        createVNode(VPBPostAuthor, { "inside-doc": "" })
      ], 64);
    };
  }
});
const _hoisted_1$7 = { class: "space-y-1 pt-6 text-center xl:pb-10" };
const _hoisted_2$6 = { class: "md:leading-14 text-3xl font-extrabold leading-9 tracking-tight text-[color:var(--vp-c-brand-dark)] dark:text-[color:var(--vp-c-brand-light)] sm:text-4xl sm:leading-10 md:text-5xl" };
const _sfc_main$b = /* @__PURE__ */ defineComponent({
  __name: "VPBLayoutPostTop",
  setup(__props) {
    const { post } = usePosts();
    return (_ctx, _cache) => {
      return openBlock(), createElementBlock(Fragment, null, [
        createBaseVNode("header", _hoisted_1$7, [
          createVNode(_sfc_main$e),
          createBaseVNode("h1", _hoisted_2$6, toDisplayString(unref(post).title), 1)
        ]),
        createVNode(_sfc_main$c, { "inside-doc": "" })
      ], 64);
    };
  }
});
const _withScopeId = (n) => (pushScopeId("data-v-bcdba372"), n = n(), popScopeId(), n);
const _hoisted_1$6 = {
  key: 0,
  class: "py-3"
};
const _hoisted_2$5 = /* @__PURE__ */ _withScopeId(() => /* @__PURE__ */ createBaseVNode("h2", { class: "text-xs uppercase tracking-wide text-gray-500 dark:text-white" }, " Next Article ", -1));
const _hoisted_3$5 = { class: "link" };
const _hoisted_4$4 = ["href"];
const _hoisted_5$3 = {
  key: 1,
  class: "py-3"
};
const _hoisted_6$3 = /* @__PURE__ */ _withScopeId(() => /* @__PURE__ */ createBaseVNode("h2", { class: "text-xs uppercase tracking-wide text-gray-500 dark:text-white" }, " Previous Article ", -1));
const _hoisted_7$3 = { class: "link" };
const _hoisted_8$3 = ["href"];
const _hoisted_9$3 = { class: "pt-3" };
const _hoisted_10$2 = ["href"];
const _sfc_main$a = /* @__PURE__ */ defineComponent({
  __name: "VPBPostLinks",
  props: {
    insideDoc: { type: Boolean }
  },
  setup(__props) {
    var _a;
    const { site } = useData();
    const { nextPost, prevPost } = usePosts();
    const theme2 = site.value.themeConfig;
    const path = withBase(((_a = theme2.blog) == null ? void 0 : _a.path) ?? "/blog/");
    return (_ctx, _cache) => {
      return openBlock(), createElementBlock("footer", {
        class: normalizeClass(["mb-24 divide-y divide-gray-200 text-sm font-medium leading-5 dark:divide-slate-200/5", { "xs:show lg:hidden": _ctx.insideDoc }])
      }, [
        unref(nextPost) ? (openBlock(), createElementBlock("div", _hoisted_1$6, [
          _hoisted_2$5,
          createBaseVNode("div", _hoisted_3$5, [
            createBaseVNode("a", {
              href: `${unref(nextPost).url}`
            }, toDisplayString(unref(nextPost).title), 9, _hoisted_4$4)
          ])
        ])) : createCommentVNode("", true),
        unref(prevPost) ? (openBlock(), createElementBlock("div", _hoisted_5$3, [
          _hoisted_6$3,
          createBaseVNode("div", _hoisted_7$3, [
            createBaseVNode("a", {
              href: `${unref(prevPost).url}`
            }, toDisplayString(unref(prevPost).title), 9, _hoisted_8$3)
          ])
        ])) : createCommentVNode("", true),
        createBaseVNode("div", _hoisted_9$3, [
          createBaseVNode("a", {
            class: "link",
            href: unref(withBase)(unref(path))
          }, "← Back to the blog", 8, _hoisted_10$2)
        ])
      ], 2);
    };
  }
});
const VPBPostLinks_vue_vue_type_style_index_0_scoped_bcdba372_lang = "";
const VPBPostLinks = /* @__PURE__ */ _export_sfc(_sfc_main$a, [["__scopeId", "data-v-bcdba372"]]);
const _sfc_main$9 = /* @__PURE__ */ defineComponent({
  __name: "VPBLayoutPostBottom",
  setup(__props) {
    return (_ctx, _cache) => {
      return openBlock(), createBlock(VPBPostLinks, { "inside-doc": "" });
    };
  }
});
const _sfc_main$8 = /* @__PURE__ */ defineComponent({
  __name: "VPBTagIcon",
  props: {
    tag: {}
  },
  setup(__props) {
    const { theme: theme2 } = useData();
    return (_ctx, _cache) => {
      var _a;
      return ((_a = unref(theme2).blog) == null ? void 0 : _a.tagIcons) && unref(theme2).blog.tagIcons[_ctx.tag.toLowerCase()] ? (openBlock(), createElementBlock("div", {
        key: 0,
        class: normalizeClass([unref(theme2).blog.tagIcons[_ctx.tag.toLowerCase()], "mr-2"])
      }, null, 2)) : createCommentVNode("", true);
    };
  }
});
const _hoisted_1$5 = { class: "bg-primary-100 inline-flex items-center rounded text-sm font-medium" };
const _hoisted_2$4 = { class: "bg-primary-100 inline-flex rounded text-sm font-medium" };
const _hoisted_3$4 = { class: "flex flex-wrap gap-2 py-5" };
const _hoisted_4$3 = ["href"];
const _sfc_main$7 = /* @__PURE__ */ defineComponent({
  __name: "VPBLayoutPostAsideTop",
  setup(__props) {
    var _a;
    const { site } = useData();
    const { post } = usePosts();
    const theme2 = site.value.themeConfig;
    const path = withBase(((_a = theme2.blog) == null ? void 0 : _a.tagsPath) ?? "/blog/tags");
    return (_ctx, _cache) => {
      return openBlock(), createElementBlock(Fragment, null, [
        createBaseVNode("span", _hoisted_1$5, [
          createVNode(VPBPostCategory, {
            category: unref(post).category
          }, null, 8, ["category"])
        ]),
        createBaseVNode("span", _hoisted_2$4, [
          createBaseVNode("div", _hoisted_3$4, [
            (openBlock(true), createElementBlock(Fragment, null, renderList(unref(post).tags, (tagName) => {
              return openBlock(), createElementBlock("a", {
                key: tagName,
                class: "rounded-sm bg-gray-100 px-2 py-1 text-xs font-semibold text-gray-600",
                href: `${unref(path)}?init=${tagName}`
              }, [
                createVNode(_sfc_main$8, { tag: tagName }, null, 8, ["tag"]),
                createTextVNode(" " + toDisplayString(tagName), 1)
              ], 8, _hoisted_4$3);
            }), 128))
          ])
        ]),
        createVNode(VPBPostAuthor)
      ], 64);
    };
  }
});
const _sfc_main$6 = /* @__PURE__ */ defineComponent({
  __name: "VPBLayoutPostAsideBottom",
  setup(__props) {
    return (_ctx, _cache) => {
      return openBlock(), createBlock(VPBPostLinks);
    };
  }
});
const _hoisted_1$4 = { class: "mb-24 divide-y divide-gray-200 text-sm font-medium leading-5 dark:divide-slate-200/5" };
const _hoisted_2$3 = { class: "pt-3" };
const _hoisted_3$3 = ["href"];
const _sfc_main$5 = /* @__PURE__ */ defineComponent({
  __name: "VPBLayoutAuthorAsideBottom",
  setup(__props) {
    var _a;
    const { site } = useData();
    const theme2 = site.value.themeConfig;
    const path = withBase(((_a = theme2.blog) == null ? void 0 : _a.path) ?? "/blog/");
    return (_ctx, _cache) => {
      return openBlock(), createElementBlock("footer", _hoisted_1$4, [
        createBaseVNode("div", _hoisted_2$3, [
          createBaseVNode("a", {
            class: "link",
            href: unref(withBase)(unref(path))
          }, "← Back to the blog", 8, _hoisted_3$3)
        ])
      ]);
    };
  }
});
const VPBLayoutAuthorAsideBottom_vue_vue_type_style_index_0_scoped_7003e0d2_lang = "";
const VPBLayoutAuthorAsideBottom = /* @__PURE__ */ _export_sfc(_sfc_main$5, [["__scopeId", "data-v-7003e0d2"]]);
const _hoisted_1$3 = { class: "mb-1 flex items-center justify-between text-gray-500" };
const _hoisted_2$2 = ["src"];
const _hoisted_3$2 = ["src"];
const _hoisted_4$2 = { class: "ml-4 text-4xl text-[color:var(--vp-c-brand-light)] dark:text-[color:var(--vp-c-brand-dark)]" };
const _hoisted_5$2 = { class: "mt-4 flex items-center justify-between text-gray-500" };
const _hoisted_6$2 = ["href"];
const _hoisted_7$2 = /* @__PURE__ */ createBaseVNode("div", { class: "i-[bx/arrow-back] mr-2" }, null, -1);
const _hoisted_8$2 = /* @__PURE__ */ createBaseVNode("span", null, "Previous Author", -1);
const _hoisted_9$2 = [
  _hoisted_7$2,
  _hoisted_8$2
];
const _hoisted_10$1 = { key: 1 };
const _hoisted_11$1 = ["href"];
const _hoisted_12$1 = /* @__PURE__ */ createBaseVNode("span", null, "Next Author", -1);
const _hoisted_13$1 = /* @__PURE__ */ createBaseVNode("div", { class: "i-[bx/right-arrow-alt] ml-2" }, null, -1);
const _hoisted_14 = [
  _hoisted_12$1,
  _hoisted_13$1
];
const _sfc_main$4 = /* @__PURE__ */ defineComponent({
  __name: "VPBLayoutAuthorTop",
  setup(__props) {
    const { author, prevAuthor, nextAuthor } = useAuthors();
    return (_ctx, _cache) => {
      return openBlock(), createElementBlock("div", null, [
        createBaseVNode("div", _hoisted_1$3, [
          unref(author).gravatar ? (openBlock(), createElementBlock("img", {
            key: 0,
            src: `https://gravatar.com/avatar/${unref(author).gravatar}`,
            alt: "author image",
            class: "h-20 w-20 rounded-full"
          }, null, 8, _hoisted_2$2)) : unref(author).avatar ? (openBlock(), createElementBlock("img", {
            key: 1,
            src: unref(author).avatar,
            alt: "author image",
            class: "h-20 w-20 rounded-full"
          }, null, 8, _hoisted_3$2)) : createCommentVNode("", true),
          createBaseVNode("span", _hoisted_4$2, toDisplayString(unref(author).name), 1)
        ]),
        createBaseVNode("div", _hoisted_5$2, [
          unref(prevAuthor) ? (openBlock(), createElementBlock("a", {
            key: 0,
            href: unref(withBase)(unref(prevAuthor).url),
            class: "inline-flex items-center font-medium hover:text-[color:var(--vp-c-brand-dark)] dark:text-white"
          }, _hoisted_9$2, 8, _hoisted_6$2)) : createCommentVNode("", true),
          !unref(prevAuthor) ? (openBlock(), createElementBlock("div", _hoisted_10$1)) : createCommentVNode("", true),
          unref(nextAuthor) ? (openBlock(), createElementBlock("a", {
            key: 2,
            href: unref(withBase)(unref(nextAuthor).url),
            class: "inline-flex items-center font-medium hover:text-[color:var(--vp-c-brand-dark)] dark:text-white"
          }, _hoisted_14, 8, _hoisted_11$1)) : createCommentVNode("", true)
        ])
      ]);
    };
  }
});
const _sfc_main$3 = /* @__PURE__ */ defineComponent({
  __name: "VPBLayout",
  setup(__props) {
    const { Layout } = theme$1;
    const { frontmatter } = useData();
    return (_ctx, _cache) => {
      return openBlock(), createBlock(unref(Layout), null, {
        "doc-before": withCtx(() => [
          unref(frontmatter).blog === "post" ? (openBlock(), createBlock(_sfc_main$b, { key: 0 })) : createCommentVNode("", true),
          unref(frontmatter).blog === "author" ? (openBlock(), createBlock(_sfc_main$4, { key: 1 })) : createCommentVNode("", true)
        ]),
        "doc-footer-before": withCtx(() => [
          unref(frontmatter).blog === "post" ? (openBlock(), createBlock(_sfc_main$9, { key: 0 })) : createCommentVNode("", true)
        ]),
        "aside-top": withCtx(() => [
          unref(frontmatter).blog === "post" ? (openBlock(), createBlock(_sfc_main$7, { key: 0 })) : createCommentVNode("", true)
        ]),
        "aside-bottom": withCtx(() => [
          unref(frontmatter).blog === "post" ? (openBlock(), createBlock(_sfc_main$6, { key: 0 })) : createCommentVNode("", true),
          unref(frontmatter).blog === "author" ? (openBlock(), createBlock(VPBLayoutAuthorAsideBottom, { key: 1 })) : createCommentVNode("", true)
        ]),
        _: 1
      });
    };
  }
});
function useArchives() {
  const postsByYear = [];
  let year = "0";
  let num = -1;
  for (let i = 0; i < data$1.length; i++) {
    const post = data$1[i];
    if (post.date) {
      const y = post.date.raw.split("-")[0];
      if (y === year) {
        postsByYear[num].push(post);
      } else {
        num++;
        postsByYear[num] = [];
        postsByYear[num].push(post);
        year = y;
      }
    }
  }
  return { postsByYear };
}
const _hoisted_1$2 = { class: "mx-auto max-w-screen-xl px-6 lg:px-16 lg:py-16" };
const _hoisted_2$1 = { class: "mx-auto mb-8 max-w-screen-sm text-center lg:mb-16" };
const _hoisted_3$1 = { class: "mb-4 text-3xl font-extrabold tracking-tight text-[color:var(--vp-c-brand-light)] dark:text-[color:var(--vp-c-brand-dark)] lg:text-4xl" };
const _hoisted_4$1 = { class: "font-light text-[color:var(--vp-c-text-light-1)] dark:text-[color:var(--vp-c-text-dark-1)] sm:text-xl" };
const _hoisted_5$1 = { class: "px-0 pb-2 pt-4 text-xl font-semibold leading-6 text-[color:var(--vp-c-brand-light)] dark:text-[color:var(--vp-c-brand-dark)]" };
const _hoisted_6$1 = ["href"];
const _hoisted_7$1 = { class: "cursor-pointer leading-6" };
const _hoisted_8$1 = /* @__PURE__ */ createBaseVNode("div", { class: "title-o" }, null, -1);
const _hoisted_9$1 = { class: "cursor-pointer font-sans leading-6" };
const _sfc_main$2 = /* @__PURE__ */ defineComponent({
  __name: "VPBArchives",
  setup(__props) {
    const { postsByYear } = useArchives();
    const { theme: theme2 } = useData();
    return (_ctx, _cache) => {
      var _a, _b;
      return openBlock(), createElementBlock("div", _hoisted_1$2, [
        createBaseVNode("div", _hoisted_2$1, [
          createBaseVNode("h2", _hoisted_3$1, toDisplayString((_a = unref(theme2).blog) == null ? void 0 : _a.title) + " Archives ", 1),
          createBaseVNode("p", _hoisted_4$1, toDisplayString((_b = unref(theme2).blog) == null ? void 0 : _b.description), 1)
        ]),
        (openBlock(true), createElementBlock(Fragment, null, renderList(unref(postsByYear), (year, yearIndex) => {
          return openBlock(), createElementBlock("div", { key: yearIndex }, [
            createBaseVNode("div", _hoisted_5$1, toDisplayString(year[0].date.raw.split("-")[0]), 1),
            (openBlock(true), createElementBlock(Fragment, null, renderList(year, (post, index) => {
              return openBlock(), createElementBlock("a", {
                key: index,
                href: unref(withBase)(post.url),
                class: "m-2 flex cursor-pointer items-center justify-between leading-6 hover:text-[color:var(--vp-c-brand-dark)] dark:hover:text-[color:var(--vp-c-brand-light)]"
              }, [
                createBaseVNode("div", _hoisted_7$1, [
                  _hoisted_8$1,
                  createTextVNode(" " + toDisplayString(post.title), 1)
                ]),
                createBaseVNode("div", _hoisted_9$1, toDisplayString(post.date.raw.slice(5)), 1)
              ], 8, _hoisted_6$1);
            }), 128))
          ]);
        }), 128))
      ]);
    };
  }
});
function useTags() {
  const postsByTag = {};
  for (let i = 0; i < data$1.length; i++) {
    const post = data$1[i];
    const tags = post.tags;
    if (Array.isArray(tags)) {
      tags.forEach((tag) => {
        if (!postsByTag[tag]) {
          postsByTag[tag] = [];
        }
        postsByTag[tag].push(post);
      });
    }
  }
  return { postsByTag };
}
const _hoisted_1$1 = { class: "mx-auto max-w-screen-xl px-6 lg:px-16 lg:py-16" };
const _hoisted_2 = { class: "mx-auto mb-8 max-w-screen-sm text-center lg:mb-16" };
const _hoisted_3 = { class: "mb-4 text-3xl font-extrabold tracking-tight text-[color:var(--vp-c-brand-light)] dark:text-[color:var(--vp-c-brand-dark)] lg:text-4xl" };
const _hoisted_4 = { class: "font-light text-[color:var(--vp-c-text-light-1)] dark:text-[color:var(--vp-c-text-dark-1)] sm:text-xl" };
const _hoisted_5 = { class: "flex flex-wrap justify-center gap-2 p-4" };
const _hoisted_6 = ["onClick"];
const _hoisted_7 = { key: 0 };
const _hoisted_8 = { class: "px-0 pb-2 pt-4 text-xl font-semibold leading-6 text-[color:var(--vp-c-brand-light)] dark:text-[color:var(--vp-c-brand-dark)]" };
const _hoisted_9 = { class: "text-xs" };
const _hoisted_10 = ["href"];
const _hoisted_11 = { class: "cursor-pointer leading-6" };
const _hoisted_12 = /* @__PURE__ */ createBaseVNode("div", { class: "title-o" }, null, -1);
const _hoisted_13 = { class: "cursor-pointer font-sans leading-6" };
const _sfc_main$1 = /* @__PURE__ */ defineComponent({
  __name: "VPBTags",
  setup(__props) {
    const { postsByTag } = useTags();
    const { theme: theme2 } = useData();
    const selectedTag = ref("");
    function toggleTag(tag) {
      selectedTag.value = tag;
    }
    if (inBrowser) {
      const params = new URLSearchParams(window.location.search);
      const init = params.get("init");
      if (init) {
        toggleTag(init);
      }
    }
    return (_ctx, _cache) => {
      const _component_ClientOnly = resolveComponent("ClientOnly");
      return openBlock(), createBlock(_component_ClientOnly, null, {
        default: withCtx(() => {
          var _a, _b;
          return [
            createBaseVNode("div", _hoisted_1$1, [
              createBaseVNode("div", _hoisted_2, [
                createBaseVNode("h2", _hoisted_3, toDisplayString((_a = unref(theme2).blog) == null ? void 0 : _a.title) + " Tags ", 1),
                createBaseVNode("p", _hoisted_4, toDisplayString((_b = unref(theme2).blog) == null ? void 0 : _b.description), 1)
              ]),
              createBaseVNode("div", _hoisted_5, [
                (openBlock(true), createElementBlock(Fragment, null, renderList(unref(postsByTag), (posts, tagName) => {
                  return openBlock(), createElementBlock("div", {
                    key: tagName,
                    class: normalizeClass({
                      "cursor-pointer rounded-full bg-gray-100 px-3 py-1 text-sm font-semibold text-gray-600": selectedTag.value !== tagName,
                      "rounded-full bg-[color:var(--vp-c-brand-light)] px-3 py-1 text-sm font-semibold text-gray-100 dark:bg-[color:var(--vp-c-brand-dark)]": selectedTag.value === tagName
                    }),
                    onClick: ($event) => toggleTag(tagName)
                  }, [
                    createVNode(_sfc_main$8, { tag: tagName }, null, 8, ["tag"]),
                    createTextVNode(" " + toDisplayString(tagName) + " ", 1),
                    createBaseVNode("span", {
                      class: normalizeClass({
                        "ml-3 text-[color:var(--vp-c-brand-light)] dark:text-[color:var(--vp-c-brand-dark)]": selectedTag.value !== tagName,
                        "ml-3 text-[color:var(--vp-c-brand-dark)] dark:text-[color:var(--vp-c-brand-light)]": selectedTag.value === tagName
                      })
                    }, toDisplayString(posts.length), 3)
                  ], 10, _hoisted_6);
                }), 128))
              ]),
              selectedTag.value ? (openBlock(), createElementBlock("div", _hoisted_7, [
                createBaseVNode("div", _hoisted_8, [
                  createVNode(_sfc_main$8, { tag: selectedTag.value }, null, 8, ["tag"]),
                  createTextVNode(toDisplayString(selectedTag.value) + " ", 1),
                  createBaseVNode("span", _hoisted_9, " ( " + toDisplayString(unref(postsByTag)[selectedTag.value].length) + " )", 1)
                ]),
                (openBlock(true), createElementBlock(Fragment, null, renderList(unref(postsByTag)[selectedTag.value], (post, index) => {
                  return openBlock(), createElementBlock("a", {
                    key: index,
                    href: unref(withBase)(post.url),
                    class: "m-2 flex cursor-pointer items-center justify-between leading-6"
                  }, [
                    createBaseVNode("div", _hoisted_11, [
                      _hoisted_12,
                      createTextVNode(" " + toDisplayString(post.title), 1)
                    ]),
                    createBaseVNode("div", _hoisted_13, toDisplayString(post.date.raw), 1)
                  ], 8, _hoisted_10);
                }), 128))
              ])) : createCommentVNode("", true)
            ])
          ];
        }),
        _: 1
      });
    };
  }
});
const _sfc_main = {};
const _hoisted_1 = { class: "theme-style-div" };
function _sfc_render(_ctx, _cache) {
  return openBlock(), createElementBlock("div", _hoisted_1, "This is a test theme component");
}
const VPBTestComponent = /* @__PURE__ */ _export_sfc(_sfc_main, [["render", _sfc_render]]);
const theme = {
  ...theme$1,
  Layout: _sfc_main$3,
  enhanceApp({ app, router, siteData }) {
    theme$1.enhanceApp({ app, router, siteData });
    app.component("VPBHome", _sfc_main$f);
    app.component("VPBArchives", _sfc_main$2);
    app.component("VPBTags", _sfc_main$1);
    app.component("VPBTestComponent", VPBTestComponent);
    app.component("VPBHomePost", _sfc_main$g);
  }
};
const style = "";
const RawTheme = {
  ...theme,
  // if you need to add more here, this is how to ensure the base theme's components are installed
  enhanceApp({ app, router, siteData }) {
    theme.enhanceApp({ app, router, siteData });
  }
};
function resolveThemeExtends(theme2) {
  if (theme2.extends) {
    const base = resolveThemeExtends(theme2.extends);
    return {
      ...base,
      ...theme2,
      async enhanceApp(ctx) {
        if (base.enhanceApp)
          await base.enhanceApp(ctx);
        if (theme2.enhanceApp)
          await theme2.enhanceApp(ctx);
      }
    };
  }
  return theme2;
}
const Theme = resolveThemeExtends(RawTheme);
const VitePressApp = defineComponent({
  name: "VitePressApp",
  setup() {
    const { site } = useData();
    onMounted(() => {
      watchEffect(() => {
        document.documentElement.lang = site.value.lang;
        document.documentElement.dir = site.value.dir;
      });
    });
    {
      usePrefetch();
    }
    useCopyCode();
    useCodeGroups();
    if (Theme.setup)
      Theme.setup();
    return () => h(Theme.Layout);
  }
});
async function createApp() {
  const router = newRouter();
  const app = newApp();
  app.provide(RouterSymbol, router);
  const data2 = initData(router.route);
  app.provide(dataSymbol, data2);
  app.component("Content", Content);
  app.component("ClientOnly", ClientOnly);
  Object.defineProperties(app.config.globalProperties, {
    $frontmatter: {
      get() {
        return data2.frontmatter.value;
      }
    },
    $params: {
      get() {
        return data2.page.value.params;
      }
    }
  });
  if (Theme.enhanceApp) {
    await Theme.enhanceApp({
      app,
      router,
      siteData: siteDataRef
    });
  }
  return { app, router, data: data2 };
}
function newApp() {
  return createSSRApp(VitePressApp);
}
function newRouter() {
  let isInitialPageLoad = inBrowser;
  let initialPath;
  return createRouter((path) => {
    let pageFilePath = pathToFile(path);
    if (isInitialPageLoad) {
      initialPath = pageFilePath;
    }
    if (isInitialPageLoad || initialPath === pageFilePath) {
      pageFilePath = pageFilePath.replace(/\.js$/, ".lean.js");
    }
    if (inBrowser) {
      isInitialPageLoad = false;
    }
    return __vitePreload(() => import(
      /*@vite-ignore*/
      pageFilePath
    ), true ? [] : void 0);
  }, Theme.NotFound);
}
if (inBrowser) {
  createApp().then(({ app, router, data: data2 }) => {
    router.go().then(() => {
      useUpdateHead(router.route, data2.site);
      app.mount("#app");
    });
  });
}
export {
  createApp
};
