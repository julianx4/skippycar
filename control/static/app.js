"use strict";
(async () => {

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
      <img id="D435image" :src="imageUrl" v-if="show_D435_image" @click="setCoord">
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
    show_D435_image: true,
    imageUrl: '/image',  // Initial image URL
    imageRefreshInterval: null,  // Handle for the interval
  }),
  async created() {
    let r = await fetch('/variables')
    this.variables = await r.json()
    this.imageRefreshInterval = setInterval(this.fetchImage, 1000);  // Refresh image every second
  },
  beforeDestroy() {
    if (this.imageRefreshInterval) {
      clearInterval(this.imageRefreshInterval);  // Clear the interval on component destroy
    }
  },
  methods: {
    async save() {
      await fetch('/save', {
        method: 'POST',
      })
    },
    setCoord(event) {
      var mouseX = event.pageX;
      var mouseY = event.pageY;
      var imageleft = document.getElementById("D435image").offsetLeft;
      var imagetop = document.getElementById("D435image").offsetTop;
      var imagewidth = document.getElementById("D435image").offsetWidth;
      var imageheight = document.getElementById("D435image").offsetHeight;
      var factorX = 640 / imagewidth;
      var factorY = 480 / imageheight; // Assuming 480 is the height of the original image
      var coordX = (mouseX - imageleft) * factorX;
      var coordY = (mouseY - imagetop) * factorY;
      fetch('/timed_value', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          name: 'tap_coord_X',
          value: coordX,
          time: 2000 // this needs to be removed, as it was replaced in car.py
        })
      });
      fetch('/timed_value', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          name: 'tap_coord_Y',
          value: coordY,
          time: 2000 // this needs to be removed, as it was replaced in car.py
        })
      })
    },
    fetchImage() {
      // Append time to the query to prevent caching
      this.imageUrl = `/image?time=${new Date().getTime()}`;
    },
  }
})

app.mount('#app')

})();
