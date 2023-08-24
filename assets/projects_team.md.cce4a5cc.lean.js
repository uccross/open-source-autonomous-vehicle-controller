import { a as VPTeamPageTitle, V as VPTeamMembers, b as VPTeamPageSection, c as VPTeamPage } from "./chunks/theme.4c21da17.js";
import { o as openBlock, c as createElementBlock, J as createVNode, E as withCtx, b as unref, a as createTextVNode } from "./chunks/framework.ce59e187.js";
const __pageData = JSON.parse('{"title":"","description":"","frontmatter":{"layout":"page"},"headers":[],"relativePath":"projects/team.md"}');
const __default__ = { name: "projects/team.md" };
const _sfc_main = /* @__PURE__ */ Object.assign(__default__, {
  setup(__props) {
    const founders = [
      {
        avatar: "https://avatars.githubusercontent.com/u/7852021?v=4",
        name: "Aaron Hunter",
        title: "PhD student at University of California Santa Cruz",
        links: [
          { icon: "github", link: "https://github.com/2ahunter" }
          // { icon: 'linkedin', link: '' }
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
    const contributor = [
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
        createVNode(unref(VPTeamPage), null, {
          default: withCtx(() => [
            createVNode(unref(VPTeamPageTitle), null, {
              title: withCtx(() => [
                createTextVNode(" Our Team ")
              ]),
              lead: withCtx(() => [
                createTextVNode(" Founders ")
              ]),
              _: 1
            }),
            createVNode(unref(VPTeamMembers), {
              size: "medium",
              members: founders
            }),
            createVNode(unref(VPTeamPageSection), null, {
              title: withCtx(() => [
                createTextVNode("Contributors")
              ]),
              lead: withCtx(() => []),
              members: withCtx(() => [
                createVNode(unref(VPTeamMembers), {
                  size: "small",
                  members: contributor
                })
              ]),
              _: 1
            })
          ]),
          _: 1
        })
      ]);
    };
  }
});
export {
  __pageData,
  _sfc_main as default
};
