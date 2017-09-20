<template>
  <header class="NavHeader">
    <Card class="row links">
      <router-link
        v-for="route in pageRoutes"
        :key="route.name"
        :to="{ name: route.name }"
      >
        {{ route.name }}
      </router-link>
    </Card>
  </header>
</template>

<script>
import { pageRoutes } from '@/router/routes'
import { Card } from '@/components'

export default {
  name: 'NavHeader',
  data() {
    return {
      pageRoutes,
    }
  },
  components: {
    Card,
  },
}
</script>

<!-- Add "scoped" attribute to limit CSS to this component only -->
<style scoped>
.NavHeader > .links {
  justify-content: center;
}

.NavHeader > .links > * {
  margin: 1rem;
  text-decoration: none;
}
</style>
