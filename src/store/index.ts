import Vue from 'vue'
import Vuex, { Store } from 'vuex'
import { getModule } from 'vuex-module-decorators'
import {
  CameraModule,
  TeleopModule,
  RosModule,
  DashboardModule,
} from '@/store/modules'

Vue.use(Vuex)

interface RootState {
  isProduction: boolean
}

const store = new Store<RootState>({
  strict: process.env.NODE_ENV !== 'production',
  state: {
    isProduction: process.env.NODE_ENV !== 'production',
  },
  modules: {
    camera: CameraModule,
    teleop: TeleopModule,
    ros: RosModule,
    dashboard: DashboardModule,
  },
})
export default store

export const cameraModule = getModule(CameraModule, store)
export const teleopModule = getModule(TeleopModule, store)
export const rosModule = getModule(RosModule, store)
export const dashboardModule = getModule(DashboardModule, store)
