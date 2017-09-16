<template>
  <div class="Dashboard">
    <div class="outer row">

      <div class="column">
        <h1>Action Client</h1>

        <Card>
          <ActionClient></ActionClient>
        </Card>
      </div>

      <div class="vr" />

      <div class="column">
        <h1>Action Server</h1>

        <Card>
          <ActionServer />
        </Card>
      </div>
    </div>
  </div>
</template>

<script>
import {
  ActionClient,
  ActionServer,
  Card,
} from '@/components'

export default {
  name: 'Dashboard',
  data() {
    return {
      msg: 'Welcome to Your Vue.js App',
    }
  },
  components: {
    ActionClient,
    ActionServer,
    Card,
  },
}
</script>

<!-- Add "scoped" attribute to limit CSS to this component only -->
<style scoped>
.Dashboard {
  width: 100%;
  height: 100%;
}

.outer.row {
  justify-content: space-around;
  height: 100%;
}

.outer > * {
  flex-grow: 1;
}

.vr {
  width: 1px;
  border: 0;
  background-color: black;
  flex-grow: 0;
}

.Card {
  border: 1px solid red;
}

</style>
