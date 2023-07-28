import { _ as _export_sfc, o as openBlock, c as createElementBlock, C as createBaseVNode, t as toDisplayString, J as createVNode, F as Fragment, R as renderList, b as unref } from "./chunks/framework.ce59e187.js";
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
const ArticleCard_vue_vue_type_style_index_0_scoped_392e4900_lang = "";
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
        return text.substring(0, length) + "...";
      }
      return text;
    }
  }
};
const _hoisted_1 = ["href"];
const _hoisted_2 = { class: "card" };
const _hoisted_3 = { class: "flex" };
const _hoisted_4 = { class: "media" };
const _hoisted_5 = ["src", "alt"];
const _hoisted_6 = { class: "details" };
const _hoisted_7 = { class: "title" };
const _hoisted_8 = { class: "excerpt" };
const _hoisted_9 = { class: "author" };
const _hoisted_10 = { class: "name" };
const _hoisted_11 = { class: "date" };
function _sfc_render(_ctx, _cache, $props, $setup, $data, $options) {
  return openBlock(), createElementBlock("a", { href: $props.href }, [
    createBaseVNode("div", _hoisted_2, [
      createBaseVNode("div", _hoisted_3, [
        createBaseVNode("div", _hoisted_4, [
          createBaseVNode("img", {
            src: $props.image,
            alt: $props.title
          }, null, 8, _hoisted_5)
        ]),
        createBaseVNode("div", _hoisted_6, [
          createBaseVNode("h2", _hoisted_7, toDisplayString($props.title), 1),
          createBaseVNode("p", _hoisted_8, toDisplayString($options.truncateText($props.excerpt, 50)), 1),
          createBaseVNode("div", _hoisted_9, [
            createBaseVNode("div", null, [
              createBaseVNode("h3", _hoisted_10, toDisplayString($props.author), 1),
              createBaseVNode("p", _hoisted_11, toDisplayString($props.date), 1)
            ])
          ])
        ])
      ])
    ])
  ], 8, _hoisted_1);
}
const ArticleCard = /* @__PURE__ */ _export_sfc(_sfc_main$1, [["render", _sfc_render], ["__scopeId", "data-v-392e4900"]]);
const data = [
  {
    Author: "Aaron Hunter",
    image: "https://avatars.githubusercontent.com/u/7852021?v=4",
    title: "Setting Up Hardware",
    path: "blog/posts/post1",
    excerpt: "Getting Trouble Setting up the Hardware ?"
  },
  {
    Author: "Carlos Espinosa",
    image: "https://avatars.githubusercontent.com/u/34257319?v=4",
    title: "Problem with MPLAB X ?",
    path: "blog/posts/post2",
    excerpt: "No Such file or directory for include files"
  }
];
const __pageData = JSON.parse('{"title":"","description":"","frontmatter":{},"headers":[],"relativePath":"troubleShooting.md"}');
const __default__ = { name: "troubleShooting.md" };
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
              image: article.image,
              author: article.Author,
              href: article.path,
              date: article.Updated
            }, null, 8, ["title", "excerpt", "image", "author", "href", "date"])
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
