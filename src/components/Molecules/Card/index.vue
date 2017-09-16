<template>
  <div class="Card">
    <slot />
  </div>
</template>

<script>

export default {
  name: 'Card',
}
</script>

<!-- Add "scoped" attribute to limit CSS to this component only -->
<style scoped>
.Card {
  box-shadow: 0 1px 3px 0 rgba(0, 0, 0, 0.2),
  0 1px 1px 0 rgba(0, 0, 0, 0.14),
  0 2px 1px -1px rgba(0, 0, 0, 0.12);
}
</style>
