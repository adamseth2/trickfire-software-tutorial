<!-- Should have " setup lang="ts" " in script tag for proper Composition and enforce typescript-->
<script setup lang="ts">
import Card from 'primevue/card';
import Panel from 'primevue/panel';

import { ForwardSlots } from 'vue-forward-slots';
</script>
<template>
  <Panel>
    <!-- <h1>TEST</h1> -->
    <slot> </slot>
  </Panel>
</template>

<!-- Should have lang="scss" and "scoped" to enable superpower of SCSS and make styles do not accidentally interact with other components styles-->
<style lang="scss" scoped></style>
