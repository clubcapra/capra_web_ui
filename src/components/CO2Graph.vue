<template>
  <div id="graph">
    <canvas ref="graph" />
  </div>
</template>

<script lang="ts">
import { Vue, Component, Prop, Inject } from 'vue-property-decorator'
import _ from 'lodash-es'
import Chart, { ChartPoint, ChartDataSets } from 'chart.js'
import 'chartjs-plugin-streaming'

import { dashboardModule } from '@/store'
import RosClient from '@/utils/ros/RosClient'
import { TopicOptions } from '../utils/ros/types'

@Component
export default class CO2Graph extends Vue {
  private chart?: Chart
  private options = {
    legend: {
      display: false,
    },
    tooltips: {
      enabled: false,
    },
    scales: {
      xAxes: [
        {
          type: 'realtime',
          realtime: {
            duration: 20000, // data in the past 20000 ms will be displayed
            delay: 1000, // delay of 1000 ms, so upcoming values are known before plotting a line
            pause: false, // chart is not paused
            ttl: undefined, // data will be automatically deleted as it disappears off the chart
          },
          time: {
            displayFormats: {
              second: 'HH:mm:ss',
            },
          },
        },
      ],
    },
    maintainAspectRatio: false,
    animation: {
      duration: 0,
    },
    plugins: {
      streaming: {
        frameRate: 30,
      },
    },
  }

  private dataset = {
    data: [],
    borderColor: 'rgb(255, 0, 0)',
    lineTension: 0,
    showLine: true,
    pointRadius: 1,
  }

  initChart() {
    const chartRef = this.$refs['graph'] as HTMLCanvasElement
    return new Chart(chartRef, {
      type: 'line',

      data: {
        datasets: [this.dataset],
      },

      options: this.options,
    })
  }

  mounted() {
    const chart = this.initChart()

    const updateChart = (data: ChartPoint) => {
      // @ts-ignore
      chart.data.datasets[0].data.push(data)
      // @ts-ignore
      chart.update({ preservation: true })
    }

    const co2Topic: TopicOptions = {
      name: '/ppm',
      messageType: 'std_msgs/String',
    }

    RosClient.subscribe(co2Topic, (value: string) => {
      const point = { x: Date.now(), y: parseInt(value) }
      updateChart(point)
    })
  }
}
</script>

<style lang="scss" scoped>
#graph {
  height: 75px;
  width: 99%;
}
</style>
