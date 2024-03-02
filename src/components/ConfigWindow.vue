<script setup lang="ts">
import { ref, onMounted } from 'vue';
import { invoke } from '@tauri-apps/api/tauri';

const selectedTab = ref(null);

const ipAddress = ref('jetson');
const port = ref('8080');
const robotName = ref('markhor');

const tabs = [
  'General',
  'Camera',
  'Graph',
  'ARM Presets',
  'GPIO Control',
  'Gamepad',
  'Launch',
];

/**
 * Updates the state by invoking the 'update_state' function with the provided parameters.
 */
async function updateState() {
  await invoke('set_state', {
    host: ipAddress.value,
    port: Number(port.value),
    name: robotName.value,
  });
}

onMounted(async () => {
  const state = await invoke<{
    host: string;
    port: number;
    robot_name: string;
  }>('get_state');
  ipAddress.value = state.host;
  port.value = state.port.toString();
  robotName.value = state.robot_name;
});
</script>

<template>
  <div class="d-flex">
    <v-tabs v-model="selectedTab" direction="vertical">
      <v-tab v-for="(tab, i) in tabs" :key="i" density="compact" :value="i">
        {{ tab }}
      </v-tab>
    </v-tabs>
    <v-container>
      <v-window v-model="selectedTab">
        <v-window-item :value="0">
          <v-responsive>
            <v-btn>Clear cache</v-btn>
          </v-responsive>
          <v-divider />
          <v-responsive>
            <v-expansion-panels>
              <v-expansion-panel title="Connection">
                <v-form>
                  <v-container>
                    <v-row>
                      <v-col cols="12" md="4">
                        <v-text-field
                          v-model="ipAddress"
                          label="IP Address"
                          required
                          hide-details
                        />
                      </v-col>

                      <v-col cols="12" md="4">
                        <v-text-field
                          v-model="port"
                          label="Port"
                          hide-details
                          required
                        />
                      </v-col>

                      <v-col cols="12" md="4">
                        <v-text-field
                          v-model="robotName"
                          label="Robot Name"
                          hide-details
                          required
                        />
                      </v-col>
                    </v-row>
                    <v-row>
                      <v-btn
                        class="mx-2"
                        variant="outlined"
                        color="green"
                        @click="updateState"
                      >
                        Connect
                      </v-btn>
                      <v-btn class="mx-2" variant="outlined" color="red">
                        Disconnect
                      </v-btn>
                    </v-row>
                  </v-container>
                </v-form>
              </v-expansion-panel>
            </v-expansion-panels>
          </v-responsive>
        </v-window-item>
        <v-window-item :value="1">b</v-window-item>
        <v-window-item :value="2">c</v-window-item>
        <v-window-item :value="3">d</v-window-item>
        <v-window-item :value="4">e</v-window-item>
        <v-window-item :value="5">f</v-window-item>
        <v-window-item :value="6">g</v-window-item>
      </v-window>
    </v-container>
  </div>
</template>
