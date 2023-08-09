import { _ as _export_sfc, o as openBlock, c as createElementBlock, C as createBaseVNode, t as toDisplayString, a as createTextVNode, S as pushScopeId, U as popScopeId, J as createVNode, F as Fragment, R as renderList, b as unref } from "./chunks/framework.4f38d4a2.js";
const Hero_vue_vue_type_style_index_0_lang = "";
const _sfc_main$2 = {
  props: {
    name: {
      type: String,
      required: true
    },
    subtitle: {
      type: String,
      required: true
    }
  }
};
const _hoisted_1$1 = { class: "hero" };
const _hoisted_2$1 = { class: "hero-body" };
const _hoisted_3$1 = { class: "container" };
const _hoisted_4$1 = { class: "title" };
const _hoisted_5$1 = { class: "subtitle" };
function _sfc_render$1(_ctx, _cache, $props, $setup, $data, $options) {
  return openBlock(), createElementBlock("div", _hoisted_1$1, [
    createBaseVNode("div", _hoisted_2$1, [
      createBaseVNode("div", _hoisted_3$1, [
        createBaseVNode("h1", _hoisted_4$1, toDisplayString($props.name || "Common Problems") + ".", 1),
        createBaseVNode("h2", _hoisted_5$1, toDisplayString($props.subtitle), 1)
      ])
    ])
  ]);
}
const Hero = /* @__PURE__ */ _export_sfc(_sfc_main$2, [["render", _sfc_render$1]]);
const ArticleCard_vue_vue_type_style_index_0_scoped_d64bf08e_lang = "";
const _sfc_main$1 = {
  props: {
    title: {
      type: String,
      required: true
    },
    excerpt: {
      type: String,
      required: true
    },
    image: {
      type: String,
      required: true
    },
    author: {
      type: String,
      required: true
    },
    date: {
      type: String,
      required: true
    },
    href: {
      type: String,
      required: true
    }
  },
  methods: {
    truncateText(text, length) {
      if (text.length > length) {
        return text.substring(0, length) + "read more...";
      }
      return text;
    }
  }
};
const _withScopeId = (n) => (pushScopeId("data-v-d64bf08e"), n = n(), popScopeId(), n);
const _hoisted_1 = ["href"];
const _hoisted_2 = { class: "card" };
const _hoisted_3 = { class: "flex" };
const _hoisted_4 = {
  class: "details",
  id: "container"
};
const _hoisted_5 = { class: "title" };
const _hoisted_6 = { class: "excerpt" };
const _hoisted_7 = /* @__PURE__ */ _withScopeId(() => /* @__PURE__ */ createBaseVNode("div", null, null, -1));
function _sfc_render(_ctx, _cache, $props, $setup, $data, $options) {
  return openBlock(), createElementBlock("a", { href: $props.href }, [
    createBaseVNode("div", _hoisted_2, [
      createBaseVNode("div", _hoisted_3, [
        createBaseVNode("div", _hoisted_4, [
          createBaseVNode("h2", _hoisted_5, toDisplayString($props.title), 1),
          createBaseVNode("p", _hoisted_6, [
            createTextVNode(toDisplayString($options.truncateText($props.excerpt, 50)) + " ", 1),
            _hoisted_7
          ])
        ])
      ])
    ])
  ], 8, _hoisted_1);
}
const ArticleCard = /* @__PURE__ */ _export_sfc(_sfc_main$1, [["render", _sfc_render], ["__scopeId", "data-v-d64bf08e"]]);
const data = [
  {
    title: "Setting Up Hardware",
    path: "./posts/post1",
    excerpt: "Getting Trouble Setting up the Hardware ?"
  },
  {
    title: "Problem with MPLAB X ?",
    path: "./posts/post2",
    excerpt: "No Such file or directory for include files"
  }
];
const __pageData = JSON.parse('{"title":"","description":"","frontmatter":{},"headers":[],"relativePath":"blog/troubleShooting.md"}');
const __default__ = { name: "blog/troubleShooting.md" };
const _sfc_main = /* @__PURE__ */ Object.assign(__default__, {
  setup(__props) {
    return (_ctx, _cache) => {
      return openBlock(), createElementBlock("div", null, [
        createVNode(Hero, {
          name: "",
          subtitle: "You can find solutions to common problems that you may find while dealing with OSAVC."
        }),
        (openBlock(true), createElementBlock(Fragment, null, renderList(unref(data), (article, index) => {
          return openBlock(), createElementBlock("div", { key: index }, [
            createVNode(ArticleCard, {
              title: article.title,
              excerpt: article.excerpt,
              author: article.Author,
              href: article.path,
              date: article.Updated
            }, null, 8, ["title", "excerpt", "author", "href", "date"])
          ]);
        }), 128))
      ]);
    };
  }
});
export {
  __pageData,
  _sfc_main as default
};
