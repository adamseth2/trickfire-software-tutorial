<script setup lang="ts">
import Splitter from 'primevue/splitter';
import SplitterPanel from 'primevue/splitterpanel';
import ScrollPanel from 'primevue/scrollpanel';
import Divider from 'primevue/divider';
import Fieldset from 'primevue/fieldset';
import Panel from 'primevue/panel';
import Tag from 'primevue/tag';
import Checkbox from 'primevue/checkbox';
import Button from 'primevue/button';

// panel 2
import { vConfetti } from '@neoconfetti/vue';
import { taskStore } from '@/store/taskStore';
import NextTaskButton from '@/components/NextTaskButton.vue';
import { roslibManager } from '@/lib/roslibUtils/roslibManager';
import { watch } from 'vue';

const task = taskStore();
const rosStore = roslibManager()
// statement to automatically check when task is complete
// watch(WHEN TO CHECK???, () => {
//   if(taskShouldBeComplete) {
//     task.completeCurrTask();
//   }
// })
</script>
<template>
  <div>
    <Splitter class="splitter">
      <SplitterPanel id="panel--left" :size="75" :minSize="10">
        <ScrollPanel class="scroll-panel">
          <h1>{Task Name}</h1>
          <Divider />
          <!-- Have to be in style css as setting it in a class does not work correctly:)))))))))👍 -->
          <Fieldset class="fieldset" legend="{Header}">
            <p>{Text}</p>
          </Fieldset>
          <div id="instructionsGroup">
            <Panel header="{H}" toggleable collapsed>
              <template #icons>
                <Checkbox class="checkbox" inputId="size_large" value="Large" size="large" />
              </template>
              <div>
                <ol>
                  <li>
                    <p>
                      <a href="">{Example URL}</a>
                      <Tag>{?}</Tag>
                    </p>
                      <pre>
                        <code>
                          {Hello World Code}
                        </code>
                      </pre>
                  </li>
                  <li>
                    <p></p>
                  </li>
                  <li>
                    <p></p>
                  </li>
                </ol>
              </div>
            </Panel>
            <Panel header="{H}" toggleable collapsed>
              <template #icons>
                <Checkbox class="checkbox" inputId="size_large" value="Large" size="large" />
              </template>
              <div>
                <ol>
                  <li>
                    <p>
                      <a href="">{Example URL}</a>
                      <Tag>{?}</Tag>
                      <pre>
                        <code>
                          {Hello World Code}
                          </code>
                          </pre>
                    </p>
                  </li>
                  <li>
                    <p></p>
                  </li>
                  <li>
                    <p></p>
                  </li>
                </ol>
              </div>
            </Panel>
            
          </div>
        </ScrollPanel>
      </SplitterPanel>
      <SplitterPanel id="panel--right" :size="30" :minSize="10">
        <ScrollPanel class="scroll-panel">
        <h1>How To Complete Task</h1>
        <Divider />
        <p>{Des}</p>
        <div style="display: flex; gap: 0.5rem">
          <!-- <Button @click="checkIfTaskIsSuccessful">Submit</Button> -->
          <div v-if="task.isCurrTaskComplete">
            <div v-confetti="{ duration: 1000 }" />
          </div>
          <NextTaskButton />
        </div>
        </ScrollPanel>
      </SplitterPanel>
    </Splitter>
  </div>
</template>

<style lang="scss" scoped></style>
