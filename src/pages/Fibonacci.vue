<template>
  <div class="Fibonacci">
    <div class="outer row">

      <div class="column">
        <Card>
          <FibonacciActionClient />
        </Card>
      </div>

      <div class="vr" />

      <div class="column">
        <Card>
          <FibonacciActionServer />
        </Card>
      </div>
    </div>
  </div>
</template>

<script>
import {
  FibonacciActionClient,
  FibonacciActionServer,
  Card,
} from '@/components'

export default {
  name: 'Fibonacci',
  data() {
    return {
    }
  },
  components: {
    FibonacciActionClient,
    FibonacciActionServer,
    Card,
  },
}
</script>

<!-- Add "scoped" attribute to limit CSS to this component only -->
<style scoped>
.Fibonacci {
  width: 100%;
  height: 100%;
}

.outer.row {
  justify-content: space-around;
  height: 100%;
}

.outer > .column {
  flex: 1;
}

.vr {
  display: none;
  width: 1px;
  border: 0;
  background-color: grey;
  flex-grow: 0;
}

.Card {
  margin: 1rem;
  padding: 1rem;
}
</style>
