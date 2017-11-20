<template>
  <div id="app">
    <NavHeader />
    <router-view></router-view>
  </div>
</template>

<script>
import { NavHeader } from '@/components'

export default {
  name: 'app',
  components: {
    NavHeader,
  },
}
</script>

<style>
html, body, #app {
  width: 100%;
  height: 100%;
  margin: 0;
  padding: 0;
  font-size: small;
}

#app {
  font-family: 'Avenir', Helvetica, Arial, sans-serif;
  -webkit-font-smoothing: antialiased;
  -moz-osx-font-smoothing: grayscale;
  text-align: center;
  color: #2c3e50;
}

h1, h2 {
  font-weight: normal;
}

h1 {
  font-size: 1.75rem;
}

h2 {
  font-size: 1.25rem;
}

section > *:first-child {
  margin-top: 0;
}

pre {
  font-size: smaller;
}

.column {
  display: flex;
  flex-direction: column;
}

.row {
  display: flex;
  flex-direction: row;
}

.align-left {
  text-align: left;
}

.row.align-bottom {
  align-items: flex-end;
}

</style>
