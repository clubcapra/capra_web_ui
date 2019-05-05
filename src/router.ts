import Vue from 'vue'
import Router, { RouteConfig } from 'vue-router'

const Teleop = () =>
  import('@/components/Teleop.vue' /* webpackChunkName: "chunk-teleop" */)
const Victim = () =>
  import('@/components/Victim.vue' /* webpackChunkName: "chunk-victim" */)
const Configuration = () =>
  import(
    '@/components/Configuration/GlobalConfig.vue' /* webpackChunkName: "chunk-config" */
  )

Vue.use(Router)

const routes: RouteConfig[] = [
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
    component: Victim,
  },
  {
    path: '/configuration',
    component: Configuration,
    children: <RouteConfig[]>[
      {
        path: 'ros',
        component: () => import('@/components/Configuration/RosConfig.vue'),
      },
      {
        path: 'teleop',
        component: () => import('@/components/Configuration/TeleopConfig.vue'),
      },
      {
        path: 'camera',
        component: () =>
          import('@/components/Configuration/camera/CameraConfig.vue'),
      },
    ],
  },
]

export default new Router({ mode: 'history', routes })
