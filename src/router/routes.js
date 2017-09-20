import _ from 'lodash'
import * as pages from '@/pages'
import Dashboard from '@/pages/Dashboard'

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
    component: Dashboard, // homepage
  },
  ...pageRoutes,
]
