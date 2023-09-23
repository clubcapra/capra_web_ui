import { createPinia, type Pinia } from 'pinia';

/** Pinia Store */
const pinia: Pinia = createPinia();
export default pinia;

// Pinia Stores
import { useGlobalStore } from './GlobalStore';

export { useGlobalStore };
