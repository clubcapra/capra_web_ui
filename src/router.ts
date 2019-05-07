import Vue from 'vue'
import Router, { RouteConfig } from 'vue-router'
import LoadingComponent from '@/components/LoadingComponent.vue'
import ErrorComponent from '@/components/ErrorComponent.vue'

function lazyLoadView(AsyncView: Promise<typeof import("*.vue")>) {
  const AsyncHandler = () => ({
    component: AsyncView,
    // A component to use while the component is loading.
    loading: LoadingComponent,
    // A fallback component in case the timeout is exceeded
    // when loading the component.
    error: ErrorComponent,
    // Delay before showing the loading component.
    // Default: 200 (milliseconds).
    delay: 0,
    // Time before giving up trying to load the component.
    // Default: Infinity (milliseconds).
    timeout: 10000
  })

  return Promise.resolve({
    functional: true,
    render(h: any, { data, children }: any) {
      // Transparently pass any props or children
      // to the view component.
      return h(AsyncHandler, data, children)
    }
  })
}

const routes: RouteConfig[] = [
  {
    path: '/',
    redirect: 'teleop'
  },
  {
    path: '/teleop',
    name: 'teleop',
    component: () => lazyLoadView(import('@/components/Teleop.vue')),
  },
  {
    path: '/victim',
    component: () => lazyLoadView(import('@/components/Victim.vue')),
  },
  {
    path: '/configuration',
    component: () => lazyLoadView(import('@/components/Configuration/GlobalConfig.vue')),
    children: <RouteConfig[]>[
      {
        path: 'ros',
        component: () => lazyLoadView(import('@/components/Configuration/RosConfig.vue')),
      },
      {
        path: 'teleop',
        component: () => lazyLoadView(import('@/components/Configuration/TeleopConfig.vue')),
      },
      {
        path: 'camera',
        component: () => lazyLoadView(import('@/components/Configuration/camera/CameraConfig.vue')),
      },
      {
        path: 'gamepad',
        component: () => lazyLoadView(import('@/components/Configuration/GamepadConfig.vue')),
      },
      {
        path: 'victim',
        component: () => lazyLoadView(import('@/components/Configuration/VictimConfig.vue')),
      },
    ],
  },
]

Vue.use(Router)
export default new Router({ base: '/Takin-UI', routes })
