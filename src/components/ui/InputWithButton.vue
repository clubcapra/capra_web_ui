<template functional>
  <b-field>
    <b-label v-if="props.label">{{ props.label }}</b-label>
    <b-field has-addons>
      <b-control>
        <b-input
          :value="props.value"
          :class="data.class"
          is-small
          type="text"
          @keydown:enter="listeners.click"
          @input="listeners.input && listeners.input($event.target.value)"
        />
      </b-control>
      <b-control>
        <b-button is-small :class="data.class" @click="listeners.click">{{
          props.buttonText
        }}</b-button>
      </b-control>
    </b-field>
  </b-field>
</template>
