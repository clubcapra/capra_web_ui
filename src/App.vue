<script setup lang="ts">
import { ref } from 'vue';

const tab = ref(null);
</script>

<template>
  <v-app>
    <v-main>
      <v-tabs v-model="tab">
        <v-tab value="teleop">Teleop</v-tab>
        <v-tab value="victim">Victim</v-tab>
        <v-tab value="config">Config</v-tab>
      </v-tabs>
      <v-window v-model="tab">
        <v-window-item value="teleop">Teleop</v-window-item>
        <v-window-item value="victim">Victim</v-window-item>
        <v-window-item value="config">Config</v-window-item>
      </v-window>
    </v-main>
  </v-app>
</template>
