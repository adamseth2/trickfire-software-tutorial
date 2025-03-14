import { defineStore } from 'pinia';
import { computed, ref } from 'vue';
export const taskStore = defineStore('taskStore', () => {
  const taskComplete = ref(new Set<number>([]));
  const currTask = ref(0);
  const isCurrTaskComplete = computed(() => {
    return taskComplete.value.has(currTask.value);
  });
  const completeCurrTask = () => {
    taskComplete.value.add(currTask.value);
  };

  return { taskComplete, currTask, isCurrTaskComplete, completeCurrTask };
});
