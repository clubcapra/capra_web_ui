import { Ros, Service, ServiceRequest } from 'roslib'
import RosClient, { ServiceOptions } from './@types'
import { getServiceSignature } from './getSignature'

export class ServiceManager {
  private ros: Ros
  private services: Map<string, Service> = new Map()
  private client: RosClient

  constructor(ros: Ros, client: RosClient) {
    this.ros = ros
    this.client = client
  }

  callService(options: ServiceOptions, payload?: unknown): Promise<unknown> {
    const service = this.getService(options)

    const request = new ServiceRequest(payload ?? '')

    const ret = new Promise((resolve, reject) => {
      if (this.client.isLogEnabled) console.log(service, request)
      service.callService(request, resolve, reject)
    })

    return ret
  }

  private getService(options: ServiceOptions): Service {
    const signature = getServiceSignature(options)

    if (this.services.has(signature))
      return this.services.get(signature) as Service

    const service = new Service({
      ros: this.ros,
      name: options.name,
      serviceType: options.serviceType ?? '',
    })

    this.services.set(signature, service)

    return service
  }
}

export default ServiceManager
