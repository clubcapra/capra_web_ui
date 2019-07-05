<template functional>
  <div class="grid-container">
    <div class="grid-item">
      <div>
        <p style="color: red">Error!</p>
        <p>Failed to load component</p>
      </div>
    </div>
  </div>
</template>

<style lang="scss" scoped>
.grid-container {
  display: grid;
  height: 100%;
  width: 100%;

  .grid-item {
    display: grid;
    align-items: center;
    justify-items: center;
    > div {
      display: grid;
      justify-items: center;
      width: 100%;
    }
  }
}
</style>
