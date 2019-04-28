import Vue from 'vue'
import Router from 'vue-router'

const Teleop = () => import('@/components/Teleop.vue')
const Victim = () => import('@/components/Victim.vue')
const Configuration = () => import('@/components/Configuration')

Vue.use(Router)

export default new Router({
  routes: [
    {
      path: '/',
      redirect: {
        name: 'teleop',
      },
    },
    {
      path: '/teleop',
      name: 'teleop',
      component: Teleop,
    },
    {
      path: '/victim',
      name: 'victim',
      component: Victim,
    },
    {
      path: '/configuration',
      name: 'configuration',
      component: Configuration,
    },
  ],
})
