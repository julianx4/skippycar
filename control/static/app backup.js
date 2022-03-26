"use strict"; (async () => {

const app = Vue.createApp({})

app.component('number', {
  template: `
    <div class="row">
      <div class="column">
        <label>{{name}}</label>
      </div>
      <div class="column">
        <input type='number' :value='value' @change='set'>
      </div>
    </div>
  `,
  data: () => ({
    value: 0,
  }),
  props: ['name'],
  created() {
    this.get()
  },
  methods: {
    async get() {
      let r = await fetch('/value?' + new URLSearchParams({
        name: this.name,
      }))
      this.value = await r.json()
    },
    async set(e) {
      this.value = parseFloat(e.target.value)
      await fetch('/value', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          name: this.name,
          value: this.value,
        })
      })
    },
  },
})

app.component('skippy-car', {
  template: `
    <div>
      <img :src="stream_url" v-if="show_map" @click="show_map=false">
      <button class="button-primary button-block" v-else @click="show_map=true">show map</button>
      <img :src="stream_D435_image_url" v-if="show_D435_image" @click="show_D435_image=false">
      <button class="button-primary button-block" v-else @click="show_D435_image=true">show D435 image</button>      
      <number
        :name='variable'
        v-for='variable in variables'
      />
      <button class="button-primary button-block" @click='save'>Save to txt</button>
    </div>
  `,
  data: () => ({
    variables: [],
    show_map: true,
    show_D435_image: true,
  }),
  async created() {
    let r = await fetch('/variables')
    this.variables = await r.json()
  },
  computed: {
    stream_url() {
      let url=new URL(window.location)
      url.port=1234
      url.pathname="/video_feed"
      return url.toString()
    },
    stream_D435_image_url() {
      let url=new URL(window.location)
      url.port=1235
      url.pathname="/video_feed"
      return url.toString()
    }
  },
  methods: {
    async save() {
      await fetch('/save', {
        method: 'POST',
      })
    },
  }
})

app.mount('#app')

})()
