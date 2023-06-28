import { defineConfig } from 'vitepress'

export default defineConfig({
  title: "OSAVC",
  siteTitle: "OSAVC",
  base: '/open-source-autonomous-vehicle-controller/',
  themeConfig: {

    blog: {
      title: 'My AI Written Blog',
      description: 'All these articles were written by AI!',
      defaultAuthor: 'AI Writer',
      categoryIcons: {
        article: 'i-[heroicons-outline/book-open]',
        tutorial: 'i-[heroicons-outline/academic-cap]',
        document: 'i-[heroicons-outline/annotation]',
      },
      tagIcons: {
        github: 'i-[carbon/logo-github]',
        vue: 'i-[carbon/logo-vue]',
      },
    },

    logo: '',

    search: {
      provider: 'local'
    },

    footer: {
      message: 'Footer Here',
      copyright: 'Copyright © copyright'
    },

    nav: [
      { text: 'Home', link: '/' },
      { text: 'About', link: '/about/' },
      { text: 'Contacts', link: '/contacts' },
      { text: 'Trouble shooting', link: '/troubleShooting/', activeMatch: '/troubleShooting/$'},
      // {
      //   text: "Trouble Shooting",
      //   items: [
      //     { text: "Connection Failure", link: "/item-1" },
      //     { text: "PICKIT configuration", link: "/item-2" },
      //     { text: "Voltage not detected", link: "/item-3" },
      //   ],
      // }
    ],

    sidebar: [
      {
        text: 'Get Started',
        collapsible: true,
        items: [
          { text: 'Introduction', link: '/introduction' },
          {
            text: 'What You Need',
            items: [
              { text: 'Hardware', link: '/hardware' },
              { text: 'Software', link: '/software' },
            ],
          },
          { text: 'Build your First Project', link: '/project' },
          { text: 'Test Harness', link: '/test_harness' },
        ],
      },
      {
        text: 'About',
        items: [
          { text: 'about', link: '/about/' },
          { text: 'Team', link: '/about/team.md' },
        ],
      },
      {
        text: 'References',
        items: [
          { text: 'api', link: '/api-examples' },
          { text: 'markdown', link: '/markdown-examples' },
        ],
      }
    ],

    socialLinks: [
      { icon: 'github', link: 'https://github.com/uccross/open-source-autonomous-vehicle-controller' },
      { icon: "discord", link: 'https://discord.gg/bkJ6DNdy' },
    ],
  }
})
