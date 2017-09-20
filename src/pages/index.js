const _ = require('lodash')

// regex format: ./component.vue
const req = require.context('.', true, /\.\/[^/]+\.vue$/)

// This exports all components, resolving circular dependencies in two steps:
// 1. export empty objects in place of each component
// 2. require each component and merge it into the existing export for that module

const componentPaths = []

// export an empty object for each component
req.keys().forEach((key) => {
  // regex format: **/(component).vue
  const componentName = key.replace(/^.+\/([^/]+)\.vue/, '$1')
  componentPaths.push({ componentName, key })
  module.exports[componentName] = {}
})

// actually start exporting each component
componentPaths.forEach(({ componentName, key }) => {
  _.merge(module.exports[componentName], req(key).default)
})
