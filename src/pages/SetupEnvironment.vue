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
import { useRoslibStore } from '@/store/roslibStore';
import PowerPlugIcon from 'vue-material-design-icons/PowerPlug.vue';

const task = taskStore();
const roslib = roslibManager();
watch(roslib.isWebSocketConnected, () => {
  if (roslib.isWebSocketConnected) {
    task.completeCurrTask();
  }
});
</script>
<template>
  <div>
    <Splitter class="splitter">
      <SplitterPanel id="panel--left" :size="70" :minSize="10">
        <ScrollPanel class="scroll-panel">
          <h1>Welcome to the Software Subteam!</h1>
          <Divider />
          <p>This is an interactive tutorial to learn the basic of Ros.</p>
          <p>You can expect to install some programs and do some coding!</p>
          <p>
            The creation of this guide is motivated by Trickfire's unique coding structure. We have
            streamlined parts of development, particularly in compiling and running, so guides
            online and chatbots, for example, may have you run unnecessary commands which are not
            needed for this codebase.
          </p>
          <!-- Have to be in style css as setting it in a class does not work correctly:)))))))))👍 -->
          <div id="instructionsGroup">
            <Panel header="Reading the Software onboard Document" toggleable collapsed>
              <template #icons>
                <Checkbox class="checkbox" inputId="size_large" value="Large" size="large" />
              </template>
              <div>
                <p>
                  Please take your time reading everything, including watching the video:
                  <a
                    href="https://github.com/TrickfireRobotics/urc-2023/blob/main/docs/getting_started.md"
                    >Onboard Document</a
                  >
                </p>
              </div>
            </Panel>
            <Panel header="Setting up Docker for Local Development" toggleable collapsed>
              <template #icons>
                <Checkbox class="checkbox" inputId="size_large" value="Large" size="large" />
              </template>
              <div>
                <p>
                  At the end of the Onboard Documentation there are links on how to set up Docker.
                  Please click the link for your operation system and follow each step very
                  carefully.
                </p>

                <p>
                  This is one of the biggest hurdle for new members, so if you need help, feel free
                  to ask another software member in the
                  <Tag>#general-software</Tag> discord channel.
                </p>
              </div>
            </Panel>
          </div>
        </ScrollPanel>
      </SplitterPanel>
      <SplitterPanel id="panel--right" :size="30" :minSize="10">
        <ScrollPanel class="scroll-panel">
          <h1>How To Complete Task</h1>
          <Divider />
          <p>
            Once you set up your environment, please run <Tag>./build.sh</Tag> and
            <Tag>./launch.sh</Tag> in the terminal.
          </p>
          <p>
            If done successfully, the icon below and the top right corner should turn
            <Tag>Green</Tag>!
          </p>
          <div class="container">
            <component
              :is="PowerPlugIcon"
              class="websocket"
              :class="{
                green: roslib.isWebSocketConnected.value,
                red: !roslib.isWebSocketConnected.value,
              }"
            />
          </div>
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

<style lang="scss" scoped>
.red {
  color: var(--error);
}
.green {
  color: var(--correct);
}

.websocket {
  transform: scale(8);
}

.container {
  margin-top: 8rem;
  margin-left: 6rem;
  margin-bottom: 10rem;
  display: flex;
  position: relative;
  &:before {
    content: 'Is Ros Running';
    position: absolute;
    top: -6rem;
    left: -2.5rem;
  }
}
</style>
