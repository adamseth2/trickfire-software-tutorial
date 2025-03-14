import { createApp } from 'vue';
import App from './App.vue';
import CreateNode from './pages/CreateNode.vue';
import CreatePublisher from './pages/CreatePublisher.vue';
import { createMemoryHistory, createRouter } from 'vue-router';
import CreateSubscriber from './pages/CreateSubscriber.vue';
import { createPinia } from 'pinia';
import PrimeVue from 'primevue/config';
import Aura from '@primeuix/themes/aura';
import SetupEnvironment from './pages/SetupEnvironment.vue';

const routes = [
  { path: '/', component: SetupEnvironment },
  { path: '/Task-0', component: SetupEnvironment },
  { path: '/Task-1', component: CreateNode },
  { path: '/Task-2', component: CreatePublisher },
  { path: '/Task-3', component: CreateSubscriber },
  // { path: '/Task#4', component: Science },
  // { path: '/Task#5', component: Help },
  // { path: '/Task#6', component: Telemetry },
  // { path: '/Task#7', component: Map },
];

const router = createRouter({
  history: createMemoryHistory(),
  routes,
});

const app = createApp(App);
//Set up Pinia
const pinia = createPinia();
app.use(PrimeVue, {
  // Default theme configuration
  theme: {
    preset: Aura,
  },
});
app.use(pinia);

app.use(router).mount('#app');
