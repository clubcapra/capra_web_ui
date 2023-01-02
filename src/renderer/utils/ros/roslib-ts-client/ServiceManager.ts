import RosClient, { ServiceOptions } from './@types';
import { getServiceSignature } from './getSignature';
import ROSLIB from 'roslib';
import type { Ros, Service } from 'roslib';
import { log } from '@/renderer/logger';

export class ServiceManager {
  private ros: Ros;
  private services = new Map<string, Service>();
  private client: RosClient;

  constructor(ros: Ros, client: RosClient) {
    this.ros = ros;
    this.client = client;
  }

  callService<P>(options: ServiceOptions, payload?: P): Promise<unknown> {
    const service = this.getService(options);

    const request = new ROSLIB.ServiceRequest(payload ?? '');

    return new Promise((resolve, reject) => {
      if (this.client.isLogEnabled) {
        log.info('ServiceManager:', service, request);
      }
      service.callService(request, resolve, reject);
    });
  }

  private getService(options: ServiceOptions): Service {
    const signature = getServiceSignature(options);

    if (this.services.has(signature)) {
      return this.services.get(signature) as Service;
    }

    const service = new ROSLIB.Service({
      ros: this.ros,
      name: options.name,
      serviceType: options.serviceType ?? '',
    });

    this.services.set(signature, service);

    return service;
  }
}

export default ServiceManager;
