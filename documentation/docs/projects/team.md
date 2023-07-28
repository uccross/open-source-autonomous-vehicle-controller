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
  
  const founders = [
    {
      avatar: 'https://avatars.githubusercontent.com/u/7852021?v=4',
      name: 'Aaron Hunter',
      title: 'PhD student at University of California Santa Cruz',
      links: [
        { icon: 'github', link: 'https://github.com/2ahunter' },
        // { icon: 'linkedin', link: '' }
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

  const contributor = [
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

<VPTeamPage>
  <VPTeamPageTitle>
    <template #title>
      Our Team
    </template>
    <template #lead>
      Founders
    </template>
  </VPTeamPageTitle>
  <VPTeamMembers size="medium" :members="founders"/>
  <VPTeamPageSection>
   <template #title>Contributors</template>
   <template #lead></template>
   <template #members>
  <VPTeamMembers size="small" :members="contributor" />
  </template>
  </VPTeamPageSection>
</VPTeamPage>
