---
next:
  text: 'Team'
  link: '../projects/team'
editLink: false
---

<script setup>
  import Hero from './Hero.vue'
  import ArticleCard from './ArticleCard.vue'
  import data from './data.json'
</script>

<Hero
name="Projects"
subtitle="Here are some projects that you can get started with OSAVC." />

<div v-for="(article, index) in data" :key="index">
  <ArticleCard :title="article.title" :excerpt="article.excerpt" :image="article.image" :author="article.Author" :href="article.path" :date="article.Updated" />
</div>
