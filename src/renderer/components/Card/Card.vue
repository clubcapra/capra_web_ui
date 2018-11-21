<template functional>
  <div class="card">
    <div class="card-header">
      <p class="card-header-title">{{ props.title }}</p>
    </div>
    <div class="card-content"><slot /></div>
  </div>
</template>
