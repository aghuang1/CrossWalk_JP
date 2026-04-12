/*
 Extensions.swift
 CrossWalk_Tokyo

 Helper functions for converting between ARKit and RealityKit types.
 Adapted from ExtendedTouch_AVP.
*/

import ARKit
import RealityKit
import UIKit

// MARK: - CollisionGroup Definitions

extension CollisionGroup {
    static let obstacle = CollisionGroup(rawValue: 1 << 0)  // Specific obstacle entities (SpawnCube, characters)
    static let skeleton = CollisionGroup(rawValue: 1 << 2)  // Virtual skeleton limbs/triggers
}

// MARK: - simd_float4x4

extension simd_float4x4 {
    var position: SIMD3<Float> {
        let t = columns.3
        return SIMD3<Float>(t.x, t.y, t.z)
    }

    var forward: SIMD3<Float> {
        let z = columns.2
        return normalize(SIMD3<Float>(-z.x, -z.y, -z.z))
    }

    /// Rotates a vector by the rotation component of this transform matrix
    func rotateVector(_ vector: SIMD3<Float>) -> SIMD3<Float> {
        let rotationMatrix = simd_float3x3(
            SIMD3<Float>(self.columns.0.x, self.columns.0.y, self.columns.0.z),
            SIMD3<Float>(self.columns.1.x, self.columns.1.y, self.columns.1.z),
            SIMD3<Float>(self.columns.2.x, self.columns.2.y, self.columns.2.z)
        )
        return rotationMatrix * vector
    }

    /// Extracts the upper-left 3x3 rotation/scale matrix
    var upperLeft3x3: simd_float3x3 {
        return simd_float3x3(
            SIMD3<Float>(self.columns.0.x, self.columns.0.y, self.columns.0.z),
            SIMD3<Float>(self.columns.1.x, self.columns.1.y, self.columns.1.z),
            SIMD3<Float>(self.columns.2.x, self.columns.2.y, self.columns.2.z)
        )
    }
}

// MARK: - SIMD4

extension SIMD4 where Scalar == Float {
    var xyz: SIMD3<Float> { SIMD3<Float>(x, y, z) }
}

// MARK: - simd_quatf

extension simd_quatf {
    /// Converts quaternion to Euler angles (in radians)
    /// Returns SIMD3<Float> with (pitch, roll, yaw) in radians
    var eulerAngles: SIMD3<Float> {
        let qw = self.real
        let qx = self.imag.x
        let qy = self.imag.y
        let qz = self.imag.z

        // Roll (x-axis rotation)
        let sinr_cosp = 2 * (qw * qx + qy * qz)
        let cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        let roll = atan2(sinr_cosp, cosr_cosp)

        // Pitch (y-axis rotation)
        let sinp = 2 * (qw * qy - qz * qx)
        let pitch = abs(sinp) >= 1 ? copysign(.pi / 2, sinp) : asin(sinp)

        // Yaw (z-axis rotation)
        let siny_cosp = 2 * (qw * qz + qx * qy)
        let cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        let yaw = atan2(siny_cosp, cosy_cosp)

        return SIMD3<Float>(pitch, roll, yaw)
    }
}

// MARK: - ModelEntity

extension ModelEntity {
    /// Creates an invisible sphere that can interact with dropped cubes in the scene.
    class func createFingertip() -> ModelEntity {
        let fingertipRadius: Float = 0.05
        let entity = ModelEntity(
            mesh: .generateSphere(radius: fingertipRadius),
            materials: [UnlitMaterial(color: UIColor.clear)],
            collisionShape: .generateSphere(radius: fingertipRadius),
            mass: 0.0)

        entity.components.set(PhysicsBodyComponent(
            shapes: [.generateSphere(radius: fingertipRadius)],
            mass: 0.01,
            mode: .dynamic))
        entity.physicsBody?.isAffectedByGravity = false
        entity.components.set(CollisionComponent(shapes: [.generateSphere(radius: fingertipRadius)]))
        entity.name = "fingertip"
        entity.components.remove(ModelComponent.self)

        return entity
    }
}
