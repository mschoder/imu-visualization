import * as THREE from 'three';

export interface AttitudeEstimates {
    accel_mag?: THREE.Quaternion;
    complementary?: THREE.Quaternion;
    ekf?: THREE.Quaternion;
    madgwick?: THREE.Quaternion;
}

/**
 * Parses an AttitudeEstimates object from JSON, ensuring Quaternion fields are properly instantiated.
 * @param json The JSON object to parse.
 * @returns The parsed AttitudeEstimates object.
 */
export function parseAttitudeEstimates(json: any): AttitudeEstimates {
    const parseQuaternion = (q: any): THREE.Quaternion | undefined => {
        if (q) {
            return new THREE.Quaternion(q[0], q[1], q[2], q[3]);
        }
        return undefined;
    };

    return {
        accel_mag: parseQuaternion(json.accel_mag),
        complementary: parseQuaternion(json.complementary),
        ekf: parseQuaternion(json.ekf),
        madgwick: parseQuaternion(json.madgwick),
    };
}
