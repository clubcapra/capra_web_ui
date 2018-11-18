<template>
  <input
    v-model="props.model"
    :class="props.class"
    @keydown="props.onKeyDown"
  />
</template>

<script>
export default {
  name: 'BaseInput'
}
</script>
