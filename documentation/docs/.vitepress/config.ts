import path from 'node:path'
import { createRequire } from 'node:module'
import type { VPBThemeConfig } from '@jcamp/vitepress-blog-theme'
import { defineConfigWithTheme } from 'vitepress'
// import { genFeed, processData } from '@jcamp/vitepress-blog-theme/config'

const require = createRequire(import.meta.url)
const pkg = require('@jcamp/vitepress-blog-theme/package.json')

export default defineConfigWithTheme<VPBThemeConfig>({
  appearance: 'dark',
  base: '/open-source-autonomous-vehicle-controller/',
  vite: {
    build: {
      minify: false,
    },
    resolve: {
      alias: {
        '@jcamp/vitepress-blog-theme/config': path.join(
          __dirname,
          '../../src/config'
        ),
        '@jcamp/vitepress-blog-theme': path.join(__dirname, '../../src/theme'),
      },
    },
  },
  cleanUrls: true,
  title: 'OSAVC',
  description: 'Open Source Autonomous vehical Controller',

  markdown: {
    headers: {
      level: [0, 0],
    },
  },
  themeConfig: {
    logo: '',
    siteTitle: 'OSAVC',
    editLink: {
      pattern:
        'https://github.com/uccross/open-source-autonomous-vehicle-controller',
    },
    blog: {
      title: 'Common Problem',
      description: '',
      defaultAuthor: 'Aaron Hunter',
      categoryIcons: {
        article: 'i-[heroicons-outline/book-open]',
        tutorial: 'i-[heroicons-outline/academic-cap]',
        document: 'i-[heroicons-outline/annotation]',
      },
      tagIcons: {
        github: 'i-[carbon/logo-github]',
        // vue: 'i-[carbon/logo-vue]',
      },
    },
    search: {
      provider: 'local',
    },

    nav: [
      {
        text: 'Guide',
        link: '/guide/introduction',
        activeMatch: '/guide/',
      },
      {
        text: 'Projects',
        link: '/projects/index',
        activeMatch: '/projects/',
      },
      {
        text: 'Contribute',
        items: [
          {
            text: 'Code',
            link: '/contribute/code/getting-started',
            activeMatch: '/contribute/code/',
          },
          {
            text: 'Website',
            link: '/contribute/website/getting-started',
            activeMatch: '/contribute/website/',
          },
        ],
      },
      {
        text: 'Trouble Shooting',
        // activeMatch: '/blog/',
        items: [
          {
            text: 'Common Problems',
            link: '/blog/troubleShooting',
          },
          {
            text: 'By categories',
            link: '/blog/tags',
            activeMatch: '/blog/tags',
          },
          {
            text: 'Archives',
            link: '/blog/archives',
            activeMatch: '/blog/archives',
          },
        ],
      },
    ],

    sidebar: {
      '/guide/': sidebarGuide(),
      '/projects/': sidebarProjects(),
      '/contribute/code': sidebarCode(),
      '/contribute/website': sidebarWebsite(),
    },

    socialLinks: [
      {
        icon: 'github',
        link: 'https://github.com/uccross/open-source-autonomous-vehicle-controller',
      },
      { icon: "discord", link: 'https://discord.gg/bkJ6DNdy' },
    ],
  },
})

function sidebarGuide() {
  return [
    {
      text: 'Getting Started',
      collapsed: false,
      items: [
        {
          text: 'Introduction',
          link: '/guide/introduction',
        },
        {
          text: 'Hardware',
          link: '/guide/Hardware'
        },
        {
          text: 'Software',
          link: '/guide/Software'
        },
        {
          text: 'Building Your First Project',
          link: '/guide/FirstProject'
        },
        {
          text: 'Test Harness',
          link: '/guide/TestHarness'
        },
        {
          text: 'RC_ESC.X',
          link: '/guide/CreatingRC_ESC_X'
        },
        {
          text: 'Companion Computers',
          link: '/guide/CompanionComputers'
        },
      ],
    },
  ]
}

function sidebarProjects() {
  return [
    {
      text: 'Projects',
      collapsed: false,
      items: [
        {
          text: 'Index',
          link: '/projects/index'
        },
        {
          text: 'Team',
          link: '/projects/team'
        },
      ],
    },
  ]
}

function sidebarWebsite() {
  return [
    {
      text: 'Contribute to Code',
      collapsed: true,
      items: [
        {
          text: 'Getting Started',
          link: '/contribute/code/getting-started'
        },
        {
          text: 'Contributing',
          link: '/contribute/code/contribute'
        },
        {
          text: 'Review Pull Request',
          link: '/contribute/code/pullRequest'
        },
      ],
    },
    {
      text: 'Contribute to Website',
      collapsed: false,
      items: [
        {
          text: 'Getting Started',
          link: '/contribute/website/getting-started'
        },
        {
          text: 'Routing',
          link: '/contribute/website/Routing'
        },
        {
          text: 'Deploy',
          items:[
            {
              text: 'GH-Pages',
              link: '/contribute/website/deploy',
            },
            {
              text: 'Creating Pull Request',
              link: '/contribute/website/pullRequest',
            },
          ]
        },
      ],
    },
    {
      text: 'Default Theme',
      collapsed: false,
      items: [
        {
          text: 'Navbar',
          link: '/contribute/website/config/nav'
        },
        {
          text: 'Sidebar',
          link: '/contribute/website/config/sidebar'
        },
        {
          text: 'Team Page',
          link: '/contribute/website/config/teamPage'
        },
        {
          text: 'Prev / Next Links',
          link: '/contribute/website/config/PrevNext'
        },
        {
          text: 'TroubleShooting Section',
          link: '/contribute/website/config/blog'
        },
        {
          text: 'Projects',
          link: '/contribute/website/config/project'
        },
      ],
    },
  ]
}

function sidebarCode() {
  return [
    {
      text: 'Contribute to Code',
      collapsed: false,
      items: [
        {
          text: 'Getting Started',
          link: '/contribute/code/getting-started'
        },
        {
          text: 'Contributing',
          link: '/contribute/code/contribute'
        },
        {
          text: 'Review Pull Request',
          link: '/contribute/code/pullRequest'
        },
      ],
    },
    {
      text: 'Contribute to Website',
      collapsed: false,
      items: [
        {
          text: 'Getting Started',
          link: '/contribute/website/getting-started'
        },
        {
          text: 'Routing',
          link: '/contribute/website/Routing'
        },
        {
          text: 'Deploy',
          items:[
            {
              text: 'GH-Pages',
              link: '/contribute/website/deploy',
            },
            {
              text: 'Creating Pull Request',
              link: '/contribute/website/pullRequest',
            },
          ]
        },
      ],
    },
    {
      text: 'Default Theme',
      collapsed: true,
      items: [
        {
          text: 'Navbar',
          link: '/contribute/website/config/nav'
        },
        {
          text: 'Sidebar',
          link: '/contribute/website/config/sidebar'
        },
        {
          text: 'Team Page',
          link: '/contribute/website/config/teamPage'
        },
        {
          text: 'Prev / Next Links',
          link: '/contribute/website/config/PrevNext'
        },
      ],
    },
  ]
}