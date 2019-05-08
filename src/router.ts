import Vue, { CreateElement } from 'vue'
import Router, { RouteConfig } from 'vue-router'
import LoadingComponent from '@/components/LoadingComponent.vue'
import ErrorComponent from '@/components/ErrorComponent.vue'
import { AsyncComponentFactory, RenderContext } from 'vue/types/options'

const ifProduction = (a: any, b: any) =>
  process.env.NODE_ENV === 'production' ? a : b

const routes: RouteConfig[] = [
  {
    path: '/',
    redirect: 'teleop',
  },
  {
    path: '/teleop',
    name: 'teleop',
    component: () => lazyLoadComponent('Teleop'),
  },
  {
    path: '/victim',
    component: () => lazyLoadComponent('Victim'),
  },
  {
    path: '/configuration',
    component: () => lazyLoadComponent('configuration/GlobalConfig'),
    children: <RouteConfig[]>[
      {
        path: 'ros',
        component: () => lazyLoadComponent('configuration/RosConfig'),
      },
      {
        path: 'teleop',
        component: () => lazyLoadComponent('configuration/TeleopConfig'),
      },
      {
        path: 'camera',
        component: () => lazyLoadComponent('configuration/camera/CameraConfig'),
      },
      {
        path: 'gamepad',
        component: () => lazyLoadComponent('configuration/GamepadConfig'),
      },
      {
        path: 'victim',
        component: () => lazyLoadComponent('configuration/VictimConfig'),
      },
    ],
  },
]

function lazyLoadComponent(componentName: string) {
  const asyncComponentFactory: AsyncComponentFactory = () => ({
    component: import(
      /* webpackChunkName: "[request]" */
      `@/components/${componentName}.vue`
    ) as any,
    loading: LoadingComponent,
    error: ErrorComponent,
    delay: 0,
    timeout: 5000,
  })

  return Promise.resolve({
    functional: true,
    render: (h: CreateElement, { data, children }: RenderContext) =>
      h(asyncComponentFactory, data, children),
  })
}

Vue.use(Router)
export default new Router({ base: ifProduction('/Takin-UI', '/'), routes })
