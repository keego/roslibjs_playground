<template>
  <div>
    <ul>
      <li>
        <pre>{</pre>
      </li>
      <li
        v-for="pair in pairs"
        :key="pair[0]"
      >
        <pre>{{ stringifyPair(pair[0], pair[1]) }}</pre>
      </li>
      <li>
        <pre>}</pre>
      </li>
    </ul>
  </div>
</template>

<script>

export default {
  name: 'JsonViewer',
  props: ['object'],
  computed: {
    pairs() {
      return Object.entries(this.object || {})
    },
  },
  methods: {
    stringifyPair(key, val) {
      const keyStr = typeof key === 'string' ? `"${key}"` : key
      const valStr = typeof val === 'string' ? `"${val}"` : val
      return `    ${keyStr}: ${valStr}`
    },
  },
}
</script>

<!-- Add "scoped" attribute to limit CSS to this component only -->
<style scoped>
ul {
  list-style-type: none;
  margin: 0;
  padding: 0;
}

li, pre {
  margin: 0
}
</style>
