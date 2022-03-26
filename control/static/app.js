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

app.component('logs', {
  template: `
  <div class="row" v-for='log, key in value'>
    <div class="column">
      <label>{{key}}</label>
    </div>
    <div class="column">
      <label>{{log}}</label>
    </div>
  </div>
  `,
  data: () => ({
    value: 0,
  }),
  props: ['key'],
  created() {
    this.timer = setInterval(this.get, 200);
  },
  methods: {
    async get() {
      let r = await fetch('/log_values')
      this.value = await r.json()
    },
  },
})

app.component('skippy-car', {
  template: `
    <div>
      <img id="D435image" :src="stream_D435_image_url" v-if="show_D435_image" @click="setCoord">
      <logs/>
      <br>
      <number
        :name='variable'
        v-for='variable in variables'
      />
      <button class="button-primary button-block" @click='save'>Save as standard</button>
      
    </div>
  `,
  data: () => ({
    variables: [],
    show_map: false,
    show_D435_image: true,
    log_titles: [],
    mouseX: 0,
    mouseY: 0,
  }),
  async created() {
    let r = await fetch('/variables')
    this.variables = await r.json()
    let r2 = await fetch('/log_titles')
    this.log_titles = await r2.json()
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
    setCoord() {
      var mouseX=event.pageX
      var mouseY=event.pageY
      var imageleft=document.getElementById("D435image").offsetTop;
      var imagetop=document.getElementById("D435image").offsetLeft;
      var imagewidth=document.getElementById("D435image").offsetWidth;
      var factor=640/imagewidth;
      var coordX=(mouseX-imageleft)*factor;
      var coordY=(mouseY-imagetop)*factor;
      fetch('/timed_value', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json'},
        body: JSON.stringify({
          name: 'tap_coord_X',
          value: coordX,
          time: 2000 // this needs to be removed, as it was replaced in car.py
        })
      });
      fetch('/timed_value', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json'},
        body: JSON.stringify({
          name: 'tap_coord_Y',
          value: coordY,
          time: 2000 // this needs to be removed, as it was replaced in car.py
          
        })
      })
    },

  }
})

app.mount('#app')

})()
