<script setup lang="ts">
import { taskStore } from '@/store/taskStore';
import { Button } from 'primevue';
import { useRouter } from 'vue-router';
const router = useRouter();
const task = taskStore();

const navigateToNextTask = () => {
  const newTask = task.currTask + 1;
  task.currTask += 1;
  router.push(`/task-${newTask}`);
};
</script>

<template>
  <div v-if="task.isCurrTaskComplete">
    <Button @click="navigateToNextTask">Next Task</Button>
  </div>
</template>

<style lang="scss" scoped></style>
