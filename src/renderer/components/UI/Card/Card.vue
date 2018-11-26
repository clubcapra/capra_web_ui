<template functional>
  <b-card>
    <b-card-header v-if="props.title">
      <b-card-header-title> {{ props.title }} </b-card-header-title>
    </b-card-header>
    <b-card-content><slot /></b-card-content>
  </b-card>
</template>
