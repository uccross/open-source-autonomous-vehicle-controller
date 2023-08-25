<template>
  <div class="hero">
    <div class="hero-body">
      <div class="container">
        <h1 class="title">{{ name || "Projects" }}.</h1>
        <h2 class="subtitle">
          {{ subtitle }}
        </h2>
      </div>
    </div>
  </div>
</template>


<script>
export default {
  props: {
    name: {
      type: String,
      required: true,
    },
    subtitle: {
      type: String,
      required: true,
    },
  },
};
</script>


<style>
h2.subtitle {
  font-size: 1.2rem;
  border-top: none;
}
</style>