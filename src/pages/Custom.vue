<template>
  <div class="Custom">
    <div class="outer row">

      <div class="column">
        <Card>
          <CustomActionClient />
        </Card>
      </div>

      <div class="vr" />

      <div class="column">
        <Card>
          <CustomActionServer />
        </Card>
      </div>

    </div>
  </div>
</template>

<script>
import {
  CustomActionClient,
  CustomActionServer,
  Card,
} from '@/components'

export default {
  name: 'Custom',
  data() {
    return {
      msg: 'Welcome to Your Vue.js App',
    }
  },
  components: {
    CustomActionClient,
    CustomActionServer,
    Card,
  },
}
</script>

<!-- Add "scoped" attribute to limit CSS to this component only -->
<style scoped>
.Custom {
  width: 100%;
  height: 100%;
}

.outer.row {
  justify-content: space-around;
  height: 100%;
}

.outer > * {
  flex: 1;
}

.vr {
  display: none; /* optional border */
  width: 1px;
  border: 0;
  background-color: black;
  flex-grow: 0;
}

.Card {
  margin: 1rem;
  padding: 1rem;
}

</style>
