import { defineStore } from 'pinia';
import { computed, ref } from 'vue';

export const taskStore = defineStore('taskStore', () => {
  const taskComplete = ref(new Set<number>([1]));
  return { taskComplete };
});
