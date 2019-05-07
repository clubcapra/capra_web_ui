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
    redirect: 'teleop'
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
      {
        path: 'gamepad',
        component: () => import('@/components/Configuration/GamepadConfig.vue'),
      },
      {
        path: 'victim',
        component: () => import('@/components/Configuration/VictimConfig.vue'),
      },
    ],
  },
]

export default new Router({ base: '/Takin-UI', routes })
