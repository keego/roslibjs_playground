<template>
  <section class="align-left">
    <h1>Fibonacci ActionClient Example</h1>

    <div>
      <label name="url">
        <legend class="label">Websocket Server Url</legend>
        <input type="text" v-model="url" />
      </label>
      <button type="button" v-on:click="connect">connect</button>
    </div>

    <div>
      <label name="goal">
        <legend class="label">Goal</legend>
        <input type="text" v-model="goalMessage" />
        <button type="button" v-on:click="send">send</button>
      </label>
    </div>

    <div id="statusIndicator">
      <p :style="{ color: status.color || 'black' }">
        Status: {{ status.text }}
      </p>
    </div>

    <template v-if="status.color !== 'green'">
      <p>
        Run the following commands from a local ROS node.
      </p>

      <ol>
        <li><code>roslaunch rosbridge_server rosbridge_websocket.launch</code></li>
      </ol>
    </template>

    <p>
      Check the JavaScript console for the output.
    </p>

  </section>
</template>

<script>
import Roslib from 'roslib'
import logger from '@/services/Logger'

const log = logger('Fibonacci Action Client', {
  infoColor: 'rgba(150, 200, 100, 1)',
})

export default {
  name: 'FibonacciActionClient',
  mounted() {
    // Connecting to ROS
    // -----------------
    // this.connect()

    // If there is an error on the backend, an 'error' emit will be emitted.
    this.ros.on('error', (error) => {
      this.setStatus({
        text: 'Error in the backend!',
        color: 'red',
      })
      log.error(error)
    })

    // Find out exactly when we made a connection.
    this.ros.on('connection', () => {
      log.info('Connection made!')
      this.setStatus({
        text: 'Connected',
        color: 'green',
      })
    })

    this.ros.on('close', () => {
      log.info('Connection closed.')
      this.setStatus({
        text: 'Connection closed.',
        color: 'red',
      })
    })

    // The ActionClient
    // ----------------
    this.fibonacciClient = new Roslib.ActionClient({
      ros: this.ros,
      serverName: '/fibonacci',
      actionName: 'actionlib_tutorials/FibonacciAction',
    })

    // Create a goal.
    this.goal = new Roslib.Goal({
      actionClient: this.fibonacciClient,
      goalMessage: {
        order: this.goalMessage,
      },
    })

    // Print out their output into the terminal.
    this.goal.on('feedback', (feedback) => {
      log.info(`Feedback: ${feedback.sequence}`)
    })

    this.goal.on('result', (result) => {
      log.info(`Final Result: ${result.sequence}`)
    })
  },
  data() {
    return {
      goalMessage: 7,
      status: {
        text: 'Not connected',
        color: 'red',
      },
      url: 'ws://keegs:9090',
      ros: new Roslib.Ros({
        url: this.url,
      }),
    }
  },
  methods: {
    connect() {
      this.ros.connect(this.url)
      this.setStatus({
        text: 'Connecting to rosbridge...',
        color: 'orange',
      })
    },
    send() {
      // Send the goal to the action server.
      this.goal.send()
    },
    setStatus(status) {
      this.status = status
    },
  },
}
</script>

<style scoped>

h1 {
  margin-top: 0;
}

ol {
  font-size: small;
  padding-left: 1rem;
}

legend {
  margin-top: 0.5rem;
  padding: 0;
  text-align: left;
  font-size: smaller;
}

</style>
