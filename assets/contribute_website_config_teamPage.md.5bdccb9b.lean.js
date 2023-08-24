import { V as VPTeamMembers } from "./chunks/theme.4c21da17.js";
import { o as openBlock, c as createElementBlock, J as createVNode, b as unref, V as createStaticVNode, C as createBaseVNode, a as createTextVNode } from "./chunks/framework.ce59e187.js";
const _hoisted_1 = /* @__PURE__ */ createStaticVNode("", 6);
const _hoisted_7 = /* @__PURE__ */ createStaticVNode("", 17);
const _hoisted_24 = /* @__PURE__ */ createBaseVNode("h3", {
  id: "contributors",
  tabindex: "-1"
}, [
  /* @__PURE__ */ createTextVNode("Contributors "),
  /* @__PURE__ */ createBaseVNode("a", {
    class: "header-anchor",
    href: "#contributors",
    "aria-label": 'Permalink to "Contributors"'
  }, "​")
], -1);
const _hoisted_25 = /* @__PURE__ */ createStaticVNode("", 2);
const __pageData = JSON.parse('{"title":"Team Page","description":"","frontmatter":{},"headers":[],"relativePath":"contribute/website/config/teamPage.md"}');
const __default__ = { name: "contribute/website/config/teamPage.md" };
const _sfc_main = /* @__PURE__ */ Object.assign(__default__, {
  setup(__props) {
    const members = [
      {
        avatar: "https://avatars.githubusercontent.com/u/7852021?v=4",
        name: "Aaron Hunter",
        title: "PhD student at University of California Santa Cruz",
        links: [
          { icon: "github", link: "https://github.com/2ahunter" }
        ]
      },
      {
        avatar: "https://avatars.githubusercontent.com/u/34257319?v=4",
        name: "Carlos Espinosa",
        title: "PhD student at University of California Santa Cruz",
        links: [
          { icon: "github", link: "https://github.com/caiespin" },
          { icon: "linkedin", link: "https://www.linkedin.com/in/carlosisaacespinosa/" }
        ]
      }
    ];
    const contributors = [
      {
        avatar: "https://avatars.githubusercontent.com/u/84729149?v=4",
        name: "Aniruddha Thakre",
        title: "B.Tech Student at Veermata Jijabai Technological Institute, Mumbai",
        links: [
          { icon: "github", link: "https://github.com/Aniruddha1261" },
          { icon: "linkedin", link: "https://www.linkedin.com/in/aniruddha-thakre-992a92213/" }
        ]
      },
      {
        avatar: "https://avatars.githubusercontent.com/u/88685125?v=4",
        name: "Chiling Han",
        title: "Student at the Harker School",
        links: [
          { icon: "github", link: "https://github.com/25ChilingH" },
          { icon: "linkedin", link: "https://www.linkedin.com/in/chiling-han-6b551921a" }
        ]
      },
      {
        avatar: "https://avatars.githubusercontent.com/u/33845372?v=4",
        name: "Damodar Datta",
        title: "Research Assistant @ RRC, IIIT Hyderabad",
        links: [
          { icon: "github", link: "https://github.com/damodardatta" },
          { icon: "linkedin", link: "https://www.linkedin.com/in/damodar-datta-kancharla-745919129/?originalSubdomain=in" }
        ]
      }
    ];
    return (_ctx, _cache) => {
      return openBlock(), createElementBlock("div", null, [
        _hoisted_1,
        createVNode(unref(VPTeamMembers), {
          size: "small",
          members
        }),
        _hoisted_7,
        createVNode(unref(VPTeamMembers), {
          size: "medium",
          members
        }),
        _hoisted_24,
        createVNode(unref(VPTeamMembers), {
          size: "small",
          members: contributors
        }),
        _hoisted_25
      ]);
    };
  }
});
export {
  __pageData,
  _sfc_main as default
};
