<template>
  <section class="align-left">
    <h1>Fibonacci ActionServer Example</h1>

    <div>
      <label name="url">
        <legend class="label">Websocket Server Url</legend>
        <input type="text" v-model="url" />
      </label>
      <button type="button" v-on:click="connect">connect</button>
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
        <li>
          <code>roslaunch rosbridge_server rosbridge_websocket.launch</code>
        </li>
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

const log = logger('Fibonacci Action Server', {
  infoColor: 'rgba(150, 150, 200, 1)',
})

export default {
  name: 'FibonacciActionServer',
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

    // The ActionServer
    // ----------------
    this.fibonacciServer = new Roslib.SimpleActionServer({
      ros: this.ros,
      serverName: '/fibonacci',
      actionName: 'actionlib_tutorials/FibonacciAction',
    })

    // handle fibonacci action request.
    this.fibonacciServer.on('goal', (goalMessage) => {
      log.info('goalMessage:', goalMessage)
      const fibonacciSequence = []
      fibonacciSequence.push(0)
      fibonacciSequence.push(1)

      for (let i = 1; i < goalMessage.order; i += 1) {
        fibonacciSequence.push(fibonacciSequence[i] + fibonacciSequence[i - 1])
        if (this.canceled === true) {
          log.info('Action server preempted')
          this.fibonacciServer.setPreempted()
        }

        log.info(fibonacciSequence)

        // send feedback
        const feedback = { sequence: fibonacciSequence }
        this.fibonacciServer.sendFeedback(feedback)
      }

      // send result
      const result = { sequence: fibonacciSequence }
      this.fibonacciServer.setSucceeded(result)
    })

    this.fibonacciServer.on('cancel', () => {
      this.canceled = true
    })
  },
  data() {
    return {
      canceled: false,
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
