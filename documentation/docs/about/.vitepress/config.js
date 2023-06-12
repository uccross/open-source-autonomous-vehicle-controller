export default {
    themeConfig: {
  
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
      ],
  
      sidebar: [
        {
          text: 'Guide',
          items: [
            { text: 'Getting Started', link: '/board' },
            { text: 'Setup', link: '/setup' },
          ],
        }
      ],
  
      socialLinks: [
        { icon: 'github', link: 'https://github.com/uccross/open-source-autonomous-vehicle-controller' },
      ]
    }
  }
  