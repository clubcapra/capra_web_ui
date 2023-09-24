<script setup lang="ts">
import { ref } from 'vue';

// Components
import ConfigWindow from './components/ConfigWindow.vue';
import EmergencyStop from './components/EmergencyStop.vue';

// Assets
import logoCapra from './assets/logo.png';

// Current selected tab
const tab = ref();
</script>

<template>
  <v-app>
    <v-app-bar density="compact" order="1">
      <v-tabs v-model="tab">
        <v-tab value="teleop">Teleop</v-tab>
        <v-tab value="victim">Victim</v-tab>
        <v-tab value="config">Config</v-tab>
      </v-tabs>
      <v-spacer />
      <v-btn icon symbol>
        <v-icon>mdi-timer-outline</v-icon>
      </v-btn>
      <v-btn icon symbol>
        <v-icon>mdi-telescope</v-icon>
      </v-btn>
      <v-btn icon symbol>
        <v-icon>mdi-lightbulb-off-outline</v-icon>
      </v-btn>
      <v-btn icon symbol>
        <v-icon>mdi-battery-remove-outline</v-icon>
      </v-btn>
      <v-btn icon symbol>
        <v-icon>mdi-battery-remove-outline</v-icon>
      </v-btn>
      <v-img
        class="mx-2"
        :src="logoCapra"
        max-height="25"
        max-width="100"
        cover
      />
    </v-app-bar>

    <v-main>
      <v-window v-model="tab">
        <v-window-item value="teleop">Teleop</v-window-item>
        <v-window-item value="victim">Victim</v-window-item>
        <v-window-item value="config">
          <ConfigWindow />
        </v-window-item>
      </v-window>
    </v-main>
    <v-navigation-drawer width="70" permanent location="end">
      <EmergencyStop />
    </v-navigation-drawer>
  </v-app>
</template>
