import { ref, type Ref } from 'vue';
import { defineStore } from 'pinia';

/** Global Store */
export const useGlobalStore = defineStore('global', () => {
  // State

  /** Loading overlay */
  const loading: Ref<boolean> = ref(true);

  /** Show loading Overlay */
  function setLoading(display: boolean) {
    loading.value = display;
  }

  return {
    loading,
    setLoading,
  };
});
