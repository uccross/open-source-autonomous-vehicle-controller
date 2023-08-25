<script setup>
  import Hero from './Hero.vue'
  import ArticleCard from './ArticleCard.vue'

  import data from './data.json'
</script>

<Hero name="" subtitle="You can find solutions to common problems that you may find while dealing with OSAVC." />

<div v-for="(article, index) in data" :key="index"
>
  <ArticleCard :title="article.title" :excerpt="article.excerpt" :author="article.Author" :href="article.path" :date="article.Updated"
  />
</div>
