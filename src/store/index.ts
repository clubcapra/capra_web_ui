import Vue from 'vue'
import Vuex, { Store, Plugin } from 'vuex'
import {
  CameraModule,
  TeleopModule,
  RosModule,
  DashboardModule,
  VictimModule,
  GamepadModule,
} from '@/store/modules'
import VuexPersistence from 'vuex-persist'

Vue.use(Vuex)

interface RootState {
  isProduction: boolean
}

const vuexLocal = new VuexPersistence<RootState>({
  storage: window.localStorage,
})

const plugins: [Plugin<RootState>] = [vuexLocal.plugin]

const store = new Store<RootState>({
  strict: process.env.NODE_ENV !== 'production',
  state: {
    isProduction: process.env.NODE_ENV !== 'production',
  },
  modules: {
    camera: CameraModule.ExtractVuexModule(CameraModule),
    teleop: TeleopModule.ExtractVuexModule(TeleopModule),
    victim: TeleopModule.ExtractVuexModule(VictimModule),
    ros: RosModule.ExtractVuexModule(RosModule),
    dashboard: DashboardModule.ExtractVuexModule(DashboardModule),
    gamepad: GamepadModule.ExtractVuexModule(GamepadModule),
  },
  plugins,
})

export default store

export const cameraModule = CameraModule.CreateProxy(store, CameraModule)
export const teleopModule = TeleopModule.CreateProxy(store, TeleopModule)
export const victimModule = VictimModule.CreateProxy(store, VictimModule)
export const rosModule = RosModule.CreateProxy(store, RosModule)
export const dashboardModule = DashboardModule.CreateProxy(
  store,
  DashboardModule
)
export const gamepadModule = GamepadModule.CreateProxy(store, GamepadModule)
