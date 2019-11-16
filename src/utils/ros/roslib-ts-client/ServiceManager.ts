import { Ros, Service, ServiceRequest } from 'roslib'
import { ServiceOptions } from './@types'
import { getServiceSignature } from './getSignature'

class ServiceManager {
  private ros: Ros
  private services: Map<string, Service> = new Map()

  constructor(ros: Ros) {
    this.ros = ros
  }

  callService(options: ServiceOptions, payload?: unknown): Promise<unknown> {
    const service = this.getService(options)

    const request = new ServiceRequest(payload)

    const ret = new Promise((resolve, reject) =>
      service.callService(request, resolve, reject)
    )

    return ret
  }

  private getService(options: ServiceOptions): Service {
    const signature = getServiceSignature(options)

    if (this.services.has(signature))
      return this.services.get(signature) as Service

    const service = new Service({
      ros: this.ros,
      ...options,
    })

    this.services.set(signature, service)

    return service
  }
}

export default ServiceManager
