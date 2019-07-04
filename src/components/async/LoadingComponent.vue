<template functional>
  <div class="grid-container">
    <div class="grid-item">
      <div>
        <p>Loading...</p>
        <div class="loading-bar" />
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

.loading-bar {
  height: 2px;
  width: 80%;
  position: relative;
  overflow: hidden;
  background-color: rgba(221, 221, 221, 0.356);

  &:before {
    display: block;
    position: absolute;
    content: '';
    left: -200px;
    width: 200px;
    height: 4px;
    background-color: #ff0000;
    animation: loading 2s linear infinite;
  }
}

@keyframes loading {
  from {
    left: -200px;
    width: 30%;
  }
  50% {
    width: 30%;
  }
  70% {
    width: 70%;
  }
  80% {
    left: 50%;
  }
  95% {
    left: 120%;
  }
  to {
    left: 100%;
  }
}
</style>
