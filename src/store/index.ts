import Vue from 'vue'
import Vuex, { Store } from 'vuex'
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
    camera: CameraModule.ExtractVuexModule(CameraModule),
    teleop: TeleopModule.ExtractVuexModule(TeleopModule),
    ros: RosModule.ExtractVuexModule(RosModule),
    dashboard: DashboardModule.ExtractVuexModule(DashboardModule),
  },
})

export default store

export const cameraModule = CameraModule.CreateProxy(store, CameraModule)
export const teleopModule = TeleopModule.CreateProxy(store, TeleopModule)
export const rosModule = RosModule.CreateProxy(store, RosModule)
export const dashboardModule = DashboardModule.CreateProxy(
  store,
  DashboardModule
)
