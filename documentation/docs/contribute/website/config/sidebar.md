# Sidebar

The sidebar is the main navigation block for your documentation.

```js
export default {
  themeConfig: {
    sidebar: {
      '/guide/': sidebarGuide(),
      '/projects/': sidebarProjects(),
    },
  }
}
```

## The Basics

The simplest form of the sidebar menu is passing in a single array of links. The first level item defines the "section" for the sidebar. It should contain `text`, which is the title of the section, and `items` which are the actual navigation links.

We have created functions for generating sidebarGuide for different sections as shown above.

Below is the code given for generating sidebar for Guide section which contains sidebar topics such Introduction, Hardware, Test Harness etc.

```js
function sidebarGuide() {
  return [
    {
      text: 'Getting Started',
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
      ],
    },
  ]
}
```

Each `link` should specify the path to the actual file starting with `/`. If you add trailing slash to the end of link, it will show `index.md` of the corresponding directory.

```js
function sidebarGuide() {
  return [
    {
      text: 'Getting Started',
      items: [
        {
          text: 'Introduction',
          link: '/guide/introduction',
        },
        {
          text: 'Test Harness',
          link: '/guide/TestHarness'
        },
      ],
    },
  ]
}
```

You may further nest the sidebar items up to 6 level deep counting up from the root level. Note that deeper than 6 level of nested items gets ignored and will not be displayed on the sidebar.

```js

function sidebarGuide() {
    return {
        text: 'Level 1',
        items: [
          {
            text: 'Level 2',
            items: [
              {
                text: 'Level 3',
                items: [
                  ...
                ]
              }
            ]
          }
        ]
    }
}
```

## Multiple Sidebars

You may show different sidebar depending on the page path. For example, as shown on this site, you might want to create a separate sections of content in your documentation like "Guide" page and "Contribute" page.

To do so, first organize your pages into directories for each desired section:

```
.
├─ guide/
│  ├─ index.md
│  ├─ one.md
│  └─ two.md
└─ contribution/
   ├─ index.md
   ├─ three.md
   └─ four.md
```

## Collapsible Sidebar Groups

By adding `collapsed` option to the sidebar group, it shows a toggle button to hide/show each section.

```js
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
    ]
}
```

All sections are "open" by default. If you would like them to be "closed" on initial page load, set `collapsed` option to `true`.

```js
function sidebarCode() {
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
    ]
}
```
