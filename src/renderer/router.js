import Vue from 'vue'
import Router from 'vue-router'
import Teleop from '@/components/Teleop'
import Victim from '@/components/Victim'
import Configuration from '@/components/Configuration'

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
      path: '/configuration',
      name: 'configuration',
      component: Configuration
    }
  ]
})
