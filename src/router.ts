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
    component: () => asyncComponent('Teleop'),
  },
  {
    path: '/victim',
    component: () => asyncComponent('Victim'),
  },
  {
    path: '/configuration',
    component: () => asyncComponent('configuration/GlobalConfig'),
    children: <RouteConfig[]>[
      {
        path: 'ros',
        component: () => asyncComponent('configuration/RosConfig'),
      },
      {
        path: 'teleop',
        component: () => asyncComponent('configuration/TeleopConfig'),
      },
      {
        path: 'camera',
        component: () => asyncComponent('configuration/camera/CameraConfig'),
      },
      {
        path: 'gamepad',
        component: () => asyncComponent('configuration/GamepadConfig'),
      },
      {
        path: 'victim',
        component: () => asyncComponent('configuration/VictimConfig'),
      },
    ],
  },
]

/**
 *
 * @param componentPath relative to @/components/ and adds .vue at the end
 */
function asyncComponent(componentPath: string) {
  const asyncComponentFactory: AsyncComponentFactory = () => ({
    component: import(
      /* webpackChunkName: "[request]" */
      `@/components/${componentPath}.vue`
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
