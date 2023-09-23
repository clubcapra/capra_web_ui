<script setup lang="ts">
import type { WritableComputedRef } from 'vue';
import { computed, onMounted } from 'vue';
import { useGlobalStore } from './store';

/** Global Store */
const globalStore = useGlobalStore();

/** loading overlay visibility */
const loading: WritableComputedRef<boolean> = computed({
  get: () => globalStore.loading,
  set: v => globalStore.setLoading(v),
});

onMounted(() => {
  loading.value = false;
});
</script>

<template>
  <v-app>
    <v-main>
      <router-view />
    </v-main>

    <v-overlay :model-value="loading" app class="justify-center align-center">
      <v-progress-circular indeterminate size="64" />
    </v-overlay>
  </v-app>
</template>
