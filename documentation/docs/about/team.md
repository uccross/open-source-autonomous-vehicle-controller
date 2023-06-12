---
layout: page
---

<script setup>

  import { 
    VPTeamPage,
    VPTeamPageTitle,
    VPTeamMembers } from 'vitepress/theme'
  
  const members = [
    {
      avatar: '/assets/images/Aaron.jpg',
      name: 'Aaron Hunter',
      title: 'PhD student at University of California Santa Cruz',
      links: [
        { icon: 'github', link: 'https://github.com/2ahunter' },
        { icon: 'linkedin', link: '' }
      ]
    },
    {
      avatar: '/assets/images/Carlos.jpg',
      name: 'Carlos Espinosa',
      title: 'PhD student at University of California Santa Cruz',
      links: [
        { icon: 'github', link: 'https://github.com/caiespin' },
        { icon: 'linkedin', link: 'https://www.linkedin.com/in/carlosisaacespinosa/' }
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
      OSAVC Team Page
    </template>
  </VPTeamPageTitle>
  <VPTeamMembers
    size="small"
    :members="members"
  />
</VPTeamPage>
