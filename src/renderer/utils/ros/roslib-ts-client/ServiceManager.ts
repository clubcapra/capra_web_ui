import RosClient, { ServiceOptions } from './@types'
import { getServiceSignature } from './getSignature'
import ROSLIB from 'roslib'
import type { Ros, Service } from 'roslib'

export class ServiceManager {
  private ros: Ros
  private services = new Map<string, Service>()
  private client: RosClient

  constructor(ros: Ros, client: RosClient) {
    this.ros = ros
    this.client = client
  }

  callService(options: ServiceOptions, payload?: unknown): Promise<unknown> {
    const service = this.getService(options)

    const request = new ROSLIB.ServiceRequest(payload ?? '')

    const ret = new Promise((resolve, reject) => {
      if (this.client.isLogEnabled) {
        // eslint-disable-next-line no-console
        console.log(service, request)
      }
      service.callService(request, resolve, reject)
    })

    return ret
  }

  private getService(options: ServiceOptions): Service {
    const signature = getServiceSignature(options)

    if (this.services.has(signature)) {
      return this.services.get(signature) as Service
    }

    const service = new ROSLIB.Service({
      ros: this.ros,
      name: options.name,
      serviceType: options.serviceType ?? '',
    })

    this.services.set(signature, service)

    return service
  }
}

export default ServiceManager
