<template>
  <section class="align-left">
    <h1>Fibonacci ActionClient Example</h1>

    <input type="text" v-model="url" />
    <button type="button" v-on:click="connect">connect</button>

    <div id="statusIndicator">
      <p :style="{ color: status.color || 'black' }">
        Status: {{ status.text }}
      </p>
    </div>

    <template v-if="status.color !== 'green'">
      <p>
        Run the following commands in the terminal then refresh this page. Check the JavaScript console for the output.
      </p>

      <ol>
        <li><code>roslaunch rosbridge_server rosbridge_websocket.launch</code></li>
      </ol>
    </template>

  </section>
</template>

<script>
import ROSLIB from 'roslib'
import logger from '@/services/Logger'

const log = logger('FibonacciActionClient')

export default {
  name: 'FibonacciActionClient',
  mounted() {
    // Connecting to ROS
    // -----------------
    this.connect()

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
      })
    })

    // The ActionClient
    // ----------------
    const fibonacciClient = new ROSLIB.ActionClient({
      ros: this.ros,
      serverName: '/fibonacci',
      actionName: 'actionlib_tutorials/FibonacciAction',
    })

    // Create a goal.
    const goal = new ROSLIB.Goal({
      actionClient: fibonacciClient,
      goalMessage: {
        order: 7,
      },
    })

    // Print out their output into the terminal.
    goal.on('feedback', (feedback) => {
      log.info(`Feedback: ${feedback.sequence}`)
    })

    goal.on('result', (result) => {
      log.info(`Final Result: ${result.sequence}`)
    })

    // Send the goal to the action server.
    goal.send()
  },
  data() {
    return {
      status: {},
      url: 'ws://keegs:9090',
      ros: new ROSLIB.Ros({
        url: this.url,
      }),
    }
  },
  methods: {
    connect() {
      this.ros.connect(this.url)
      this.setStatus({
        text: 'Connecting to rosbridge...',
      })
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

#statusIndicator {
}

</style>
