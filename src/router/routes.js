import _ from 'lodash'
import * as pages from '@/pages'

export const pageRoutes = Object.entries(pages || {})
  .map(([name, component]) => ({
    path: `/${_.kebabCase(name)}`,
    name,
    component,
  }))

export default [
  {
    path: '/',
    name: 'home',
    component: pages.Fibonacci, // homepage
  },
  ...pageRoutes,
]
