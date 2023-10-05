<script setup lang="ts">
import { ref } from 'vue';
import { invoke } from '@tauri-apps/api/tauri';

const greetMsg = ref('');
const name = ref('');

/**
 * Greet the user
 */
async function greet() {
  // Learn more about Tauri commands at https://tauri.app/v1/guides/features/command
  greetMsg.value = await invoke('greet', { name: name.value });
}
</script>

<template>
  <v-app>
    <v-sheet width="300" class="mx-auto">
      <form class="row" @submit.prevent="greet">
        <v-form>
          <v-container>
            <v-text-field v-model="name" placeholder="Enter a name...." />
          </v-container>
        </v-form>
        <div class="d-flex flex-column">
          <v-btn block class="mt-4" type="submit" color="primary">Greet</v-btn>
        </div>
      </form>

      <p>{{ greetMsg }}</p>
    </v-sheet>
  </v-app>
</template>
