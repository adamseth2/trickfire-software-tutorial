<!-- Should have " setup lang="ts" " in script tag for proper Composition and enforce typescript-->
<script setup lang="ts">
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
</script>
<template>
  <SplitterPanel :size="15" :minSize="10">
    <ScrollPanel style="width: 100%; height: calc(100vh - var(--nav-bar-size))">
      <div v-confetti="{ duration: 1000 }" />
      <slot />
    </ScrollPanel>
  </SplitterPanel>
</template>

<!-- Should have lang="scss" and "scoped" to enable superpower of SCSS and make styles do not accidentally interact with other components styles-->
<style lang="scss" scoped></style>
