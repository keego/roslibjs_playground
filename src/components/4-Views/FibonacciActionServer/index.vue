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
      <p :style="{ color: statusMessage.color || 'black' }">
        Status: {{ statusMessage.text }}
      </p>
    </div>

    <h2>Steps</h2>
    <em>Note: Check the JavaScript console for output</em>
    <ol>
      <li>
        <span>Run the following command in a terminal:</span>
        <br />
        <code>roslaunch rosbridge_server rosbridge_websocket.launch</code>
      </li>
      <li>Connect to the Websocket Server</li>
    </ol>

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
    this.setStatus(this.Status.NOT_CONNECTED)

    // If there is an error on the backend, an 'error' emit will be emitted.
    this.ros.on('error', (error) => {
      log.error(error)
      this.setStatus(this.Status.ERROR)
    })

    // Find out exactly when we made a connection.
    this.ros.on('connection', () => {
      log.info('Connection made!')
      this.setStatus(this.Status.CONNECTED)
    })

    this.ros.on('close', () => {
      log.info('Connection closed.')
      this.setStatus(this.Status.CLOSED)
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
      status: {},
      statusMessage: {},
      url: 'ws://localhost:9090',
      ros: new Roslib.Ros({
        url: this.url,
      }),
      Status: {
        ERROR: 0,
        NOT_CONNECTED: 1,
        CONNECTING: 2,
        CONNECTED: 3,
        CLOSED: 4,
      },
    }
  },
  methods: {
    connect() {
      this.ros.connect(this.url)
      this.setStatus(this.Status.CONNECTING)
    },
    getStatusMessage(status) {
      const Status = this.Status

      switch (status) {
        case Status.NOT_CONNECTED: return {
          text: 'Not connected',
          color: 'red',
        }
        case Status.CONNECTING: return {
          text: 'Connecting to rosbridge...',
          color: 'orange',
        }
        case Status.CONNECTED: return {
          text: 'Connected',
          color: 'green',
        }
        case Status.CLOSED: return {
          text: 'Connection closed.',
          color: 'red',
        }
        case Status.ERROR: return {
          text: 'Error in the backend!',
          color: 'red',
        }
        default:
          log.error('unknown status', status)
          return {
            text: 'Invalid state',
            color: 'red',
          }
      }
    },
    setStatus(status) {
      this.status = status
      this.statusMessage = this.getStatusMessage(status)
    },
  },
}
</script>

<style scoped>

ol {
  font-size: small;
  padding-left: 1rem;
}

li {
  margin: 0.5rem 0rem;
}

legend {
  margin-top: 0.5rem;
  padding: 0;
  text-align: left;
  font-size: smaller;
}

</style>
