<script>
export default {
  name: 'BaseInput',
  props: {
    type: {
      type: String,
      default: 'text'
    },
    value: {
      type: String,
      default: ''
    }
  },
  render(h) {
    const { type, value } = this

    return h('b-input', {
      type,
      attrs: { value },
      props: { ...this.$attrs },
      on: { input: e => this.$emit('input', e.target.value) }
    })
  }
}
</script>
