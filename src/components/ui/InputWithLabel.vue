<template functional>
  <b-field>
    <b-label v-if="props.label">{{ props.label }}</b-label>
    <b-field>
      <b-control>
        <input
          :value="props.value"
          :class="`input is-small ${data.class}`"
          type="text"
          @keydown.enter="listeners.click"
          @input="e => data.on.input(e.target.value)"
        />
      </b-control>
    </b-field>
  </b-field>
</template>
