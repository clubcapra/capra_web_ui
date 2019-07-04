import { Ros, Service, ServiceRequest } from 'roslib'
import { ServiceOptions } from './types'

class ServiceManager {
  private ros: Ros
  private services: Map<string, Service> = new Map()

  constructor(ros: Ros) {
    this.ros = ros
  }

  callService(options: ServiceOptions, payload?: any) {
    const service = this.getService(options)

    const request = new ServiceRequest(payload)

    return new Promise((resolve, reject) =>
      service.callService(request, resolve, reject)
    )
  }

  private getSignature({ name, serviceType }: ServiceOptions) {
    return `${name}/${serviceType}`
  }

  private getService(options: ServiceOptions) {
    const signature = this.getSignature(options)

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
