<template>
  <div id="app">
    <NavHeader></NavHeader>
    <router-view></router-view>
  </div>
</template>

<script>
import {
  NavHeader,
} from '@/components'

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

.column {
  display: flex;
  flex-direction: column;
}

.row {
  display: flex;
  flex-direction: row;
}

</style>
