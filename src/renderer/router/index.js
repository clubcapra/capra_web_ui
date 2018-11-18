import Vue from 'vue'
import Router from 'vue-router'
import Teleop from '@/components/Teleop'
import Victim from '@/components/Victim'
import Settings from '@/components/Settings'

Vue.use(Router)

export default new Router({
  routes: [
    {
      path: '/',
      redirect: {
        name: 'teleop'
      }
    },
    {
      path: '/teleop',
      name: 'teleop',
      component: Teleop
    },
    {
      path: '/victim',
      name: 'victim',
      component: Victim
    },
    {
      path: '/settings',
      name: 'settings',
      component: Settings
    }
  ]
})
