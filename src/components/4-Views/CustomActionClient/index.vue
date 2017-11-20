<template>
  <section class="align-left">
    <h1>Custom ActionClient</h1>

    <h2>ROS Info</h2>
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

    <h2>Action Client Info</h2>
    <div class="row align-bottom">
      <label>
        <legend class="label">Server Name</legend>
        <input type="text" v-model="clientInfo.serverName" />
      </label>

      <label>
        <legend class="label">Action Name</legend>
        <input type="text" v-model="clientInfo.actionName" />
      </label>

      <label>
        <button type="button"
          v-on:click="createClient">
          Create Client
        </button>
      </label>
    </div>

    <h2>Goal Info</h2>
    <table>
      <thead>
        <tr>
          <th>Prop</th>
          <th>Value</th>
          <th>Number?</th>
        </tr>
      </thead>
      <tbody>
        <tr
          v-for="pair in Object.entries(goalMessageInfo)"
          :key="pair[0]">
          <td>
            <input type="text" v-model="pair[0]" />
          </td>
          <td>
            <input type="text" v-model="pair[1].value" />
          </td>
          <td>
            <input type="checkbox" v-model="pair[1].isNumber" />
          </td>
          <td>
            <button type="button"
              v-on:click="deleteGoalMessageProp(pair[0])">
              Delete
            </button>
          </td>
        </tr>
        <tr>
          <td>
            <input type="text" v-model="newGoalProp.key" />
          </td>
          <td>
            <input type="text" v-model="newGoalProp.value" />
          </td>
          <td>
            <input type="checkbox" v-model="newGoalProp.isNumber" />
          </td>
          <td>
            <button type="button"
              v-on:click="setGoalMessageProp(newGoalProp)">
              Add
            </button>
          </td>
        </tr>
      </tbody>
    </table>

    <JsonViewer :object="goalMessage"></JsonViewer>

    <div>
      <label>
        <button type="button"
          v-if="client"
          v-on:click="sendGoal"
        >
          Send Goal
        </button>
      </label>
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
      <li>Create an ActionClient</li>
      <li>Send a Goal</li>
    </ol>

  </section>
</template>

<script>
import Roslib from 'roslib'
import logger from '@/services/Logger'
import {
  JsonViewer,
} from '@/components'

const log = logger('Fibonacci Action Client', {
  infoColor: 'rgba(150, 200, 100, 1)',
})

export default {
  name: 'FibonacciActionClient',
  components: {
    JsonViewer,
  },
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
  },
  data() {
    return {
      // ROS
      url: 'ws://localhost:9090',
      ros: new Roslib.Ros({
        url: this.url,
      }),
      status: {},
      statusMessage: {},
      Status: {
        ERROR: 0,
        NOT_CONNECTED: 1,
        CONNECTING: 2,
        CONNECTED: 3,
        CLOSED: 4,
      },
      // Action Client
      client: null,
      clientInfo: {
        serverName: '/fibonacci',
        actionName: 'actionlib_tutorials/FibonacciAction',
      },
      goal: null,
      goalMessageInfo: {
        order: { value: 7, isNumber: true },
      },
      newGoalProp: {
        key: '',
        value: '',
        isNumber: false,
      },
    }
  },
  computed: {
    goalMessage() {
      return Object.entries(this.goalMessageInfo)
        .reduce((goalMessage, [key, { value, isNumber }]) =>
          Object.assign({}, goalMessage, {
            [key]: isNumber ? +value : `${value}`,
          })
        , {})
    },
  },
  methods: {
    // --------------------------------------
    // ROS
    // --------------------------------------
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
    // --------------------------------------
    // Action Client
    // --------------------------------------
    createClient() {
      this.client = new Roslib.ActionClient({
        ros: this.ros,
        serverName: this.clientInfo.serverName,
        actionName: this.clientInfo.actionName,
      })
    },
    setGoalMessageProp({ key, value, isNumber }) {
      if (key) {
        this.$set(this.goalMessageInfo, key, { value, isNumber })
        this.newGoalProp.key = ''
        this.newGoalProp.value = ''
        this.newGoalProp.isNumber = false
      }
    },
    deleteGoalMessageProp(key) {
      if (key) {
        this.$delete(this.goalMessageInfo, key)
      }
    },
    sendGoal() {
      // Create a goal.
      this.goal = new Roslib.Goal({
        actionClient: this.client,
        goalMessage: this.goalMessage,
      })

      log.debug('Goal Info', {
        actionClient: this.client,
        goalMessage: this.goalMessage,
      })

      // Print out their output into the terminal.
      this.goal.on('feedback', (feedback) => {
        log.info(`Feedback: ${feedback.sequence}`)
      })

      this.goal.on('result', (result) => {
        log.info(`Final Result: ${result.sequence}`)
      })

      // Send the goal to the action server.
      this.goal.send()
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

th {
  font-size: smaller;
  font-weight: normal;
}

</style>
