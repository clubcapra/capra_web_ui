// Load vue core
import { createApp } from 'vue';
import store from './store';

// Load plugins
import vuetify from './plugins/vuetify';

// Load Layout vue.
import App from './App.vue';

/** Register Vue */
const vue = createApp(App);
vue.use(store);
vue.use(vuetify);

// Run!
vue.mount('#app');
