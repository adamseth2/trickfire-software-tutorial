<script setup lang="ts">
import { onMounted } from 'vue';
import Splitter from 'primevue/splitter';
import SplitterPanel from 'primevue/splitterpanel';
import ScrollPanel from 'primevue/scrollpanel';
import SplitButton from 'primevue/splitbutton';
import Divider from 'primevue/divider';
import Card from 'primevue/card';
import Fieldset from 'primevue/fieldset';
import Panel from 'primevue/panel';
import Tag from 'primevue/tag';
import Checkbox from 'primevue/checkbox';
import Button from 'primevue/button';

// panel 2
import ExampleComponent from '@/components/ExampleComponent.vue';
import PublishTesterComponent from '@/components/PublishTesterComponent.vue';
import { vConfetti } from '@neoconfetti/vue';
import { taskStore } from '@/store/taskStore';

const TASK_NUMBER = 0;

onMounted(() => {
  console.log('hello world');
});
const task = taskStore();

const checkIfTaskIsSuccessful = () => {
  task.taskComplete.add(task.currTask);
};
</script>
<template>
  <div>
    <Splitter class="splitter">
      <SplitterPanel id="panel--right" :size="70" :minSize="10">
        <ScrollPanel class="scroll-panel">
          <h1>Task Name</h1>
          <!-- Have to be in style css as setting it in a class does not work correctly:)))))))))👍 -->
          <Fieldset class="fieldset" legend="KeyWord">
            <p>
              Lorem ipsum dolor sit amet consectetur adipisicing elit. Consequatur repellendus
              beatae delectus. Odit, incidunt deserunt?
            </p>
          </Fieldset>
          <div id="instructionsGroup">
            <Panel header="Creating a Package" toggleable>
              <template #icons>
                <Checkbox class="checkbox" inputId="size_large" value="Large" size="large" />
              </template>
              <div></div>
            </Panel>
          </div>
        </ScrollPanel>
      </SplitterPanel>
      <SplitterPanel id="panel--right" :size="30" :minSize="10">
        <!-- <ExampleComponent /> -->
        <h1>How To Complete Task</h1>
        <p>Once it looks like this: (images of console) Click the button Below</p>
        <div>
          <Button @click="checkIfTaskIsSuccessful">Submit</Button>
          <div v-if="task.isCurrTaskComplete">
            <div v-confetti="{ duration: 1000 }" />
          </div>
          <NextTaskButton />
        </div>
        <!-- <PublishTesterComponent /> -->
      </SplitterPanel>
    </Splitter>
  </div>
</template>

<style lang="scss" scoped></style>
