<script setup>
import { VPTeamMembers } from 'vitepress/theme'

const members = [
  {
    avatar: 'https://avatars.githubusercontent.com/u/7852021?v=4',
    name: 'Aaron Hunter',
    title: 'PhD student at University of California Santa Cruz',
    links: [
      { icon: 'github', link: 'https://github.com/2ahunter' },
    ]
  },
  {
      avatar: 'https://avatars.githubusercontent.com/u/34257319?v=4',
      name: 'Carlos Espinosa',
      title: 'PhD student at University of California Santa Cruz',
      links: [
        { icon: 'github', link: 'https://github.com/caiespin' },
        { icon: 'linkedin', link: 'https://www.linkedin.com/in/carlosisaacespinosa/' }
      ]
    },
]
const contributors = [
  {
      avatar: 'https://avatars.githubusercontent.com/u/84729149?v=4',
      name: 'Aniruddha Thakre',
      title: 'B.Tech Student at Veermata Jijabai Technological Institute, Mumbai',
      links: [
        { icon: 'github', link: 'https://github.com/Aniruddha1261' },
        { icon: 'linkedin', link: 'https://www.linkedin.com/in/aniruddha-thakre-992a92213/' }
      ]
    },
    {
      avatar: 'https://avatars.githubusercontent.com/u/88685125?v=4',
      name: 'Chiling Han',
      title: 'Student at the Harker School',
      links: [
        { icon: 'github', link: 'https://github.com/25ChilingH' },
        { icon: 'linkedin', link: 'https://www.linkedin.com/in/chiling-han-6b551921a' }
      ]
    },
    {
      avatar: 'https://avatars.githubusercontent.com/u/33845372?v=4',
      name: 'Damodar Datta',
      title: 'Research Assistant @ RRC, IIIT Hyderabad',
      links: [
        { icon: 'github', link: 'https://github.com/damodardatta' },
        { icon: 'linkedin', link: 'https://www.linkedin.com/in/damodar-datta-kancharla-745919129/?originalSubdomain=in' }
      ]
    },
]
</script>

# Team Page

If you would like to introduce your team, you may use Team components to construct the Team Page.

## Show team members in a page

You may use `<VPTeamMembers>` component exposed from `vitepress/theme` to display a list of team members on any page.

```html
<script setup>
import { VPTeamMembers } from 'vitepress/theme'

const members = [
  {
    avatar: 'https://avatars.githubusercontent.com/u/7852021?v=4',
    name: 'Aaron Hunter',
    title: 'PhD student at University of California Santa Cruz',
    links: [
      { icon: 'github', link: 'https://github.com/2ahunter' },
    ]
  },
  ...
]
</script>

# Our Team

Say hello to our awesome team.

<VPTeamMembers size="small" :members="members" />
```

The above will display a team member in card looking element. It should display something similar to below.

<VPTeamMembers size="small" :members="members" />

`<VPTeamMembers>` component comes in 2 different sizes, `small` and `medium`. While it boils down to your preference, usually `small` size should fit better when used in doc page. Also, you may add more properties to each member such as adding "description" or "sponsor" button. Learn more about it in [`<VPTeamMembers>`](#vpteammembers).

Embedding team members in doc page is good for small size team where having dedicated full team page might be too much, or introducing partial members as a reference to documentation context.

If you have large number of members, or simply would like to have more space to show team members, consider [creating a full team page](#create-a-full-team-page).

## Create a full Team Page

Instead of adding team members to doc page, you may also create a full Team Page, similar to how you can create a custom **Home Page**

To create a team page, first, create a new md file. The file name doesn't matter, but here lets call it `team.md`. In this file, set frontmatter option `layout: page`, and then you may compose your page structure using `TeamPage` components.

```html
---
layout: page
---
<script setup>
import {
  VPTeamPage,
  VPTeamPageTitle,
  VPTeamMembers
} from 'vitepress/theme'

const founders = [
  {
      avatar: 'https://avatars.githubusercontent.com/u/34257319?v=4',
      name: 'Carlos Espinosa',
      title: 'PhD student at University of California Santa Cruz',
      links: [
        { icon: 'github', link: 'https://github.com/caiespin' },
        { icon: 'linkedin', link: 'https://www.linkedin.com/in/carlosisaacespinosa/' }
      ]
    },
  ...
]
</script>

<VPTeamPage>
  <VPTeamPageTitle>
    <template #title>
      Our Team
    </template>
    <template #lead>
      Founders
    </template>
  </VPTeamPageTitle>
  <VPTeamMembers
    :members="founders"
  />
</VPTeamPage>
```

When creating a full team page, remember to wrap all components with `<VPTeamPage>` component. This component will ensure all nested team related components get the proper layout structure like spacings.

`<VPPageTitle>` component adds the page title section. The title being `<h1>` heading. Use `#title` and `#lead` slot to document about your team.

`<VPMembers>` works as same as when used in a doc page. It will display list of members.

### Add sections to divide team members

You may add "sections" to the team page. For example, you may have different types of team members such as Core Team Members and Community Partners. You can divide these members into sections to better explain the roles of each group.

To do so, add `<VPTeamPageSection>` component to the `team.md` file we created previously.

```html
---
layout: page
---
<script setup>
import {
  VPTeamPage,
  VPTeamPageTitle,
  VPTeamMembers,
  VPTeamPageSection
} from 'vitepress/theme'

const founders = [...]
const contributor = [...]
</script>

<VPTeamPage>
  <VPTeamPageTitle>
    <template #title>Our Team</template>
    <template #lead>Founders</template>
  </VPTeamPageTitle>
  <VPTeamMembers size="medium" :members="founders" />
  <VPTeamPageSection>
    <template #title>Contributors</template>
    <template #lead>...</template>
    <template #members>
      <VPTeamMembers size="small" :members="contributor" />
    </template>
  </VPTeamPageSection>
</VPTeamPage>
```

The above will display a team members in card looking element. It should display something similar to below.

## Our Team

### Founders

<VPTeamMembers size="medium" :members="members" />

### Contributors

<VPTeamMembers size="small" :members="contributors" />

The `<VPTeamPageSection>` component can have `#title` and `#lead` slot similar to `VPTeamPageTitle` component, and also `#members` slot for displaying team members.

Remember to put in `<VPTeamMembers>` component within `#members` slot.
