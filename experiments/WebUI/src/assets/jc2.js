import {Gateway, Message, Performative} from './fjage.js';

export class Management {

    /**
     * Constructs a Management class.
     *
     * @param {Gateway} gateway fjage gateway.
     */
    constructor(gateway) {
        this._gateway = gateway;

        this._ready = false;
        this._initializationFailed = false;
    }

    /**
     * Waits for ready state.
     *
     * @returns {Promise<Management>} A promise returning this Management object.
     * @private
     */
    _waitForReady() {
        return new Promise((resolve, reject) => {
            if (this._ready) {
                resolve(this);
            } else if (this._initializationFailed) {
                reject('initialization failed');
            } else {
                Promise.all([
                    this._gateway.agentForService('org.arl.jc2.enums.C2Services.CAPTAIN'),
                    this._gateway.agentForService('org.arl.jc2.enums.C2Services.MANAGEMENT'),
                ])
                    .then(agentIds => {
                        this._captainAgentId = agentIds[0];
                        this._managementAgentId = agentIds[1];
                        this._ready = true;
                        console.log('initialized');
                        resolve(this);
                    })
                    .catch(error => {
                        this._initializationFailed = true;
                        console.log('could not acquire services', error);
                        reject(error);
                    });
            }
        });
    }

    /**
     * Runs a mission.
     *
     * @param {int} missionNumber Mission number.
     */
    runMission(missionNumber) {
        this.getVehicleId()
            .then(vehicleId => {
                const request = new OperatorCmdReq({
                    recipient: this._captainAgentId,
                    cmd: 'RUN',
                    vehicleID: vehicleId,
                    missionNumber: missionNumber,
                });
                // NOTE: no response expected
                this._gateway.send(request);
            });
    }

    /**
     * Gets missions.
     *
     * @returns {Promise<Array>} Promise returning the missions.
     */
    getMissions() {
        return this._waitForReady()
            .then(management => {
                return new Promise((resolve, reject) => {
                    const request = new GetMissionsReq({
                        recipient: this._managementAgentId,
                    });
                    this._gateway.request(request)
                        .then(response => {
                            console.log('getMissions', response);
                            if (response.perf === Performative.INFORM) {
                                resolve(response.missions);
                            } else {
                                reject(response.perf);
                            }
                        });
                });
            });
    }

    /**
     * Gets the vehicle ID.
     *
     * @return {Promise<String>} Promise returning the vehicle ID.
     */
    getVehicleId() {
        return this._waitForReady()
            .then(management => {
                return new Promise((resolve, reject) => {
                    const request = new GetVehicleIdReq({
                        recipient: this._managementAgentId,
                    });
                    this._gateway.request(request)
                        .then(response => {
                            if (response.perf === Performative.INFORM) {
                                resolve(response.vehicleId);
                            } else {
                                reject(response.perf);
                            }
                        });
                });
            })
            .catch(reason => {
                return reason;
            });
    }

    getOrigin() {
        return this._waitForReady()
            .then(management => {
                return new Promise((resolve, reject) => {
                    const request = new GetOriginReq({
                        recipient: this._managementAgentId,
                    });
                    this._gateway.request(request)
                        .then(response => {
                            if (response.perf === Performative.INFORM) {
                                resolve(response.origin);
                            } else {
                                reject(response.perf);
                            }
                        });
                });
            });
    }

    getGeofence() {
        return this._waitForReady()
            .then(management => {
                return new Promise((resolve, reject) => {
                    const request = new GetGeofenceReq({
                        recipient: this._managementAgentId,
                    });
                    this._gateway.request(request)
                        .then(response => {
                            if (response.perf === Performative.INFORM) {
                                resolve(response.points);
                            } else {
                                reject(response.perf);
                            }
                        });
                });
            });
    }

    getHealth() {
        return this._waitForReady()
            .then(management => {
                return new Promise((resolve, reject) => {
                    const request = new GetHealthReq({
                        recipient: this._managementAgentId,
                    });
                    this._gateway.request(request)
                        .then(response => {
                            if (response.perf === Performative.INFORM) {
                                resolve(response.records);
                            } else {
                                reject(response.perf);
                            }
                        });
                });
            });
    }

    getScript() {
        return this._waitForReady()
            .then(management => {
                return new Promise((resolve, reject) => {
                    const request = new GetScriptReq({
                        recipient: this._managementAgentId,
                    });
                    this._gateway.request(request)
                        .then(response => {
                            if (response.perf === Performative.INFORM) {
                                resolve(response.script);
                            } else {
                                reject(response.perf);
                            }
                        });
                });
            });
    }

    putScript(script) {
        return this._waitForReady()
            .then(management => {
                return new Promise((resolve, reject) => {
                    const request = new PutScriptReq({
                        recipient: this._managementAgentId,
                        script: script,
                    });
                    this._gateway.request(request)
                        .then(response => {
                            if (response.perf === Performative.INFORM) {
                                resolve(response.perf);
                            } else {
                                reject(response.perf);
                            }
                        });
                });
            });
    }

    runScript(subroutine) {
        return this._waitForReady()
            .then(management => {
                return new Promise((resolve, reject) => {
                    const request = new RunScriptReq({
                        recipient: this._managementAgentId,
                        subroutine: subroutine,
                    });
                    this._gateway.request(request)
                        .then(response => {
                            if (response.perf === Performative.INFORM) {
                                resolve(response.id);
                            } else {
                                reject(response.perf);
                            }
                        });
                });
            });
    }

    getScriptRunResult(id) {
        return this._waitForReady()
            .then(management => {
                return new Promise((resolve, reject) => {
                    const request = new GetScriptRunResultReq({
                        recipient: this._managementAgentId,
                        id: id,
                    });
                    this._gateway.request(request)
                        .then(response => {
                            if (response.perf === Performative.INFORM) {
                                resolve({
                                    id: response.id,
                                    subroutine: response.subroutine,
                                    startedAt: response.startedAt,
                                    completed: response.completed,
                                    completedAt: response.completedAt,
                                    exitValue: response.exitValue,
                                    output: response.output,
                                    error: response.error,
                                });
                            } else {
                                reject(response.perf);
                            }
                        });
                });
            });
    }

    getSentuators() {
        return this._waitForReady()
            .then(management => {
                return new Promise((resolve, reject) => {
                    const request = new GetSentuatorsReq({
                        recipient: this._managementAgentId,
                    });
                    this._gateway.request(request)
                        .then(response => {
                            if (response.perf === Performative.INFORM) {
                                resolve(response.sentuators);
                            } else {
                                reject(response.perf);
                            }
                        });
                });
            });
    }

    getSentuatorHealth(sentuatorName) {
        return this._waitForReady()
            .then(management => {
                return new Promise((resolve, reject) => {
                    const request = new GetSentuatorHealthReq({
                        recipient: this._managementAgentId,
                        sentuatorName: sentuatorName,
                    });
                    this._gateway.request(request)
                        .then(response => {
                            if (response.perf === Performative.INFORM) {
                                resolve(response.sentuatorHealth);
                            } else {
                                reject(response.perf);
                            }
                        });
                });
            });
    }

    getMeasurement(sentuatorName, sensorType, maxAge) {
        return this._waitForReady()
            .then(management => {
                return new Promise((resolve, reject) => {
                    const request = new GetMeasurementReq({
                        recipient: this._managementAgentId,
                        sentuatorName: sentuatorName,
                        sensorType: sensorType,
                        maxAge: maxAge,
                    });
                    this._gateway.request(request)
                        .then(response => {
                            if (response.perf === Performative.INFORM) {
                                resolve(response.measurement);
                            } else {
                                reject(response.perf);
                            }
                        });
                });
            });
    }
}

// ----

class AbstractRequest extends Message {

    /**
     * Constructs an AbstractRequest message.
     *
     * @param {String} className class name.
     * @param {Object} params parameters.
     */
    constructor(className, params) {
        super(new Message(), Performative.REQUEST);
        this.__clazz__ = className;
        if (params) {
            const keys = Object.keys(params);
            for (let k of keys) {
                this[k] = params[k];
            }
        }
    }
}

export class OperatorCmdReq extends AbstractRequest {

    /**
     * Constructs an OperatorCmdReq message.
     *
     * @param {Object} params parameters.
     */
    constructor(params) {
        super('org.arl.jc2.messages.OperatorCmdReq', params);
    }
}

export class MissionTasksReq extends AbstractRequest {

    /**
     * Constructs an MissionTasksReq message.
     *
     * @param {Object} params parameters.
     */
    constructor(params) {
        super('org.arl.jc2.messages.MissionTasksReq', params);
    }
}

export class GetVehicleIdReq extends AbstractRequest {

    /**
     * Constructs an GetVehicleIdReq message.
     *
     * @param {Object} params parameters.
     */
    constructor(params) {
        super('org.arl.jc2.messages.management.GetVehicleIdReq', params);
    }
}

export class GetOriginReq extends AbstractRequest {

    /**
     * Constructs an GetOriginReq message.
     *
     * @param {Object} params parameters.
     */
    constructor(params) {
        super('org.arl.jc2.messages.management.GetOriginReq', params);
    }
}

export class GetGeofenceReq extends AbstractRequest {

    /**
     * Constructs an GetGeofenceReq message.
     *
     * @param {Object} params parameters.
     */
    constructor(params) {
        super('org.arl.jc2.messages.management.GetGeofenceReq', params);
    }
}

export class GetHealthReq extends AbstractRequest {

    /**
     * Constructs an GetHealthReq message.
     *
     * @param {Object} params parameters.
     */
    constructor(params) {
        super('org.arl.jc2.messages.management.GetHealthReq', params);
    }
}

export class GetScriptReq extends AbstractRequest {

    /**
     * Constructs an GetScriptReq message.
     *
     * @param {Object} params parameters.
     */
    constructor(params) {
        super('org.arl.jc2.messages.management.GetScriptReq', params);
    }
}

export class PutScriptReq extends AbstractRequest {

    /**
     * Constructs an PutScriptReq message.
     *
     * @param {Object} params parameters.
     */
    constructor(params) {
        super('org.arl.jc2.messages.management.PutScriptReq', params);
    }
}

export class RunScriptReq extends AbstractRequest {

    /**
     * Constructs an RunScriptReq message.
     *
     * @param {Object} params parameters.
     */
    constructor(params) {
        super('org.arl.jc2.messages.management.RunScriptReq', params);
    }
}

export class GetScriptRunResultReq extends AbstractRequest {

    /**
     * Constructs an GetScriptRunResultReq message.
     *
     * @param {Object} params parameters.
     */
    constructor(params) {
        super('org.arl.jc2.messages.management.GetScriptRunResultReq', params);
    }
}

export class GetSentuatorsReq extends AbstractRequest {

    /**
     * Constructs an GetSentuatorsReq message.
     *
     * @param {Object} params parameters.
     */
    constructor(params) {
        super('org.arl.jc2.messages.management.GetSentuatorsReq', params);
    }
}

export class GetSentuatorHealthReq extends AbstractRequest {

    /**
     * Constructs an GetSentuatorHealthReq message.
     *
     * @param {Object} params parameters.
     */
    constructor(params) {
        super('org.arl.jc2.messages.management.GetSentuatorHealthReq', params);
    }
}

export class GetMeasurementReq extends AbstractRequest {

    /**
     * Constructs an GetMeasurementReq message.
     *
     * @param {Object} params parameters.
     */
    constructor(params) {
        super('org.arl.jc2.messages.management.GetMeasurementReq', params);
    }
}

export class GetMissionsReq extends AbstractRequest {

    /**
     * Constructs an GetMissionsReq message.
     *
     * @param {Object} params parameters.
     */
    constructor(params) {
        super('org.arl.jc2.messages.management.GetMissionsReq', params);
    }
}
