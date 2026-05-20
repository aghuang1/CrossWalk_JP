/*
 HandTrackingManager.swift
 CrossWalk_Tokyo

 Tracks both hands via HandTrackingProvider and builds a single virtual skeleton
 (center) superimposed on the user. Ported from ExtendedTouch_AVP isotropicExpansion.
*/

import ARKit
import RealityKit
import UIKit

/// Holds all entities for the single "center" skeleton superimposed on the user.
struct SkeletonInstance {
    let id: String

    // Arm hierarchy
    var leftShoulderAnchor: Entity?
    var leftShoulderPivot: Entity?
    var leftElbowPivot: Entity?
    var rightShoulderAnchor: Entity?
    var rightShoulderPivot: Entity?
    var rightElbowPivot: Entity?

    // Leg hierarchy
    var leftHipAnchor: Entity?
    var leftHipPivot: Entity?
    var leftKneePivot: Entity?
    var rightHipAnchor: Entity?
    var rightHipPivot: Entity?
    var rightKneePivot: Entity?

    // Cylinders (for collision indicator color changes)
    var leftUpperArmCylinder: ModelEntity?
    var leftForearmCylinder: ModelEntity?
    var rightUpperArmCylinder: ModelEntity?
    var rightForearmCylinder: ModelEntity?
    var leftThighCylinder: ModelEntity?
    var leftShankCylinder: ModelEntity?
    var rightThighCylinder: ModelEntity?
    var rightShankCylinder: ModelEntity?
}

@MainActor
class HandTrackingManager {
    let handTracking = HandTrackingProvider()

    /// Palm entities: one ModelEntity per hand (left/right). Invisible collision proxies.
    private(set) var palmEntities: [HandAnchor.Chirality: ModelEntity] = [:]

    /// IMU-tracked body segments (8 segments: both arms + both legs)
    enum IMUBodySegment: String, CaseIterable {
        case leftUpperArm = "leftUpperArm"
        case leftForearm = "leftForearm"
        case rightUpperArm = "rightUpperArm"
        case rightForearm = "rightForearm"
        case leftThigh = "leftThigh"
        case leftShank = "leftShank"
        case rightThigh = "rightThigh"
        case rightShank = "rightShank"
    }

    /// Four limb groups that can be independently shown/hidden.
    enum LimbGroup {
        case leftArm
        case rightArm
        case leftLeg
        case rightLeg
    }

    // MARK: - Skeleton Instance

    /// Single skeleton instance keyed by id ("center"). Dictionary preserves the
    /// multi-skeleton API so callers iterating over skeletons keep working.
    var skeletons: [String: SkeletonInstance] = [:]

    // Segment lengths (in meters)
    private let upperArmLength: Float = 0.28
    private let forearmLength: Float = 0.25
    private let thighLength: Float = 0.45
    private let shankLength: Float = 0.17

    // Cylinder radii
    let armRadius: Float = 0.05
    let legRadius: Float = 0.06

    // Per-limb trigger volumes were removed in favor of a single headset proximity
    // trigger + per-frame distance math (see BodyTrackingModel).

    // MARK: - Parent Orientation Cache (world-absolute)

    private var lastParentOrientations: [IMUBodySegment: simd_quatf] = [
        .leftUpperArm: simd_quatf(ix: 0, iy: 0, iz: 0, r: 1),
        .rightUpperArm: simd_quatf(ix: 0, iy: 0, iz: 0, r: 1),
        .leftThigh: simd_quatf(ix: 0, iy: 0, iz: 0, r: 1),
        .rightThigh: simd_quatf(ix: 0, iy: 0, iz: 0, r: 1),
    ]

    /// Updates a single parent segment's stored orientation. Called from the
    /// subscription handler (parents-first pass) so children always have fresh
    /// parent data before computing their local rotation.
    func updateParentOrientation(segment: IMUBodySegment, orientation: simd_quatf) {
        lastParentOrientations[segment] = orientation
    }

    /// Resets parent orientations to identity — call during calibration reset so
    /// child segments (forearm, shank) don't use stale parent data.
    func resetParentOrientations() {
        lastParentOrientations = [
            .leftUpperArm: simd_quatf(ix: 0, iy: 0, iz: 0, r: 1),
            .rightUpperArm: simd_quatf(ix: 0, iy: 0, iz: 0, r: 1),
            .leftThigh: simd_quatf(ix: 0, iy: 0, iz: 0, r: 1),
            .rightThigh: simd_quatf(ix: 0, iy: 0, iz: 0, r: 1),
        ]
    }

    // MARK: - Heading / Chest Yaw

    /// Chest yaw delta: rotation from calibration-time facing to current chest facing.
    /// Updated by BodyTrackingModel from chest IMU data each frame.
    var chestYawDelta: simd_quatf = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)
    var chestYawDeltaInverse: simd_quatf = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)

    /// Calibration heading: yaw-only rotation representing which direction the user
    /// faced during calibration. Set once at calibration time by BodyTrackingModel.
    var calibrationHeadingQ: simd_quatf = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)

    /// Absolute heading = calibration heading + chest yaw delta.
    var absoluteHeading: simd_quatf {
        calibrationHeadingQ * chestYawDelta
    }

    /// World-space forward projection vector used to position the virtual skeleton
    /// ahead of the user. Subtract from virtual limb positions to get body positions.
    func getForwardProjectionVector(forwardOffset: Float) -> SIMD3<Float> {
        return simd_act(absoluteHeading, SIMD3<Float>(0, 0, forwardOffset))
    }

    // MARK: - Setup

    /// Create palm placeholders and build the single "center" skeleton.
    func setupPalms(on contentEntity: Entity) {
        for chirality in [HandAnchor.Chirality.left, HandAnchor.Chirality.right] {
            let placeholder = ModelEntity()
            placeholder.name = "palm_\(chirality)"
            palmEntities[chirality] = placeholder
            contentEntity.addChild(placeholder)
        }

        // Single skeleton superimposed on the user.
        for skeletonID in ["center"] {
            let instance = buildSkeleton(id: skeletonID, on: contentEntity)
            skeletons[skeletonID] = instance
        }
    }

    // MARK: - Build Skeleton

    private func buildSkeleton(id: String, on contentEntity: Entity) -> SkeletonInstance {
        var instance = SkeletonInstance(id: id)

        let (lsa, lsp, lep, luaCyl, lfaCyl) = buildArmLimb(skeletonID: id, side: "left", on: contentEntity)
        instance.leftShoulderAnchor = lsa
        instance.leftShoulderPivot = lsp
        instance.leftElbowPivot = lep
        instance.leftUpperArmCylinder = luaCyl
        instance.leftForearmCylinder = lfaCyl

        let (rsa, rsp, rep, ruaCyl, rfaCyl) = buildArmLimb(skeletonID: id, side: "right", on: contentEntity)
        instance.rightShoulderAnchor = rsa
        instance.rightShoulderPivot = rsp
        instance.rightElbowPivot = rep
        instance.rightUpperArmCylinder = ruaCyl
        instance.rightForearmCylinder = rfaCyl

        let (lha, lhp, lkp, ltCyl, lsCyl) = buildLegLimb(skeletonID: id, side: "left", on: contentEntity)
        instance.leftHipAnchor = lha
        instance.leftHipPivot = lhp
        instance.leftKneePivot = lkp
        instance.leftThighCylinder = ltCyl
        instance.leftShankCylinder = lsCyl

        let (rha, rhp, rkp, rtCyl, rsCyl) = buildLegLimb(skeletonID: id, side: "right", on: contentEntity)
        instance.rightHipAnchor = rha
        instance.rightHipPivot = rhp
        instance.rightKneePivot = rkp
        instance.rightThighCylinder = rtCyl
        instance.rightShankCylinder = rsCyl

        return instance
    }

    private func buildArmLimb(
        skeletonID: String,
        side: String,
        on contentEntity: Entity
    ) -> (Entity, Entity, Entity, ModelEntity, ModelEntity) {
        let segUA = "\(side)UpperArm"
        let segFA = "\(side)Forearm"

        let shoulderAnchor = Entity()
        shoulderAnchor.name = "\(skeletonID)_\(side)ShoulderAnchor"
        contentEntity.addChild(shoulderAnchor)

        let shoulderPivot = Entity()
        shoulderPivot.name = "\(skeletonID)_\(side)ShoulderPivot"
        shoulderAnchor.addChild(shoulderPivot)

        let upperArmCylinder = createCylinder(
            height: upperArmLength, radius: armRadius,
            color: .systemBlue, name: "\(skeletonID)_\(segUA)"
        )
        upperArmCylinder.position = SIMD3<Float>(0, -upperArmLength / 2, 0)
        shoulderPivot.addChild(upperArmCylinder)

        let shoulderMarker = ModelEntity(
            mesh: .generateSphere(radius: 0.035),
            materials: [SimpleMaterial(color: UIColor.white.withAlphaComponent(0.9), isMetallic: false)]
        )
        shoulderMarker.name = "\(skeletonID)_\(side)ShoulderMarker"
        applyNoOcclusion(to: shoulderMarker)
        shoulderAnchor.addChild(shoulderMarker)

        let elbowPivot = Entity()
        elbowPivot.name = "\(skeletonID)_\(side)ElbowPivot"
        elbowPivot.position = SIMD3<Float>(0, -upperArmLength, 0)
        shoulderPivot.addChild(elbowPivot)

        let forearmCylinder = createCylinder(
            height: forearmLength, radius: armRadius,
            color: .systemCyan, name: "\(skeletonID)_\(segFA)"
        )
        forearmCylinder.position = SIMD3<Float>(0, -forearmLength / 2, 0)
        elbowPivot.addChild(forearmCylinder)

        let elbowMarker = ModelEntity(
            mesh: .generateSphere(radius: 0.03),
            materials: [SimpleMaterial(color: UIColor.yellow.withAlphaComponent(0.9), isMetallic: false)]
        )
        elbowMarker.name = "\(skeletonID)_\(side)ElbowMarker"
        applyNoOcclusion(to: elbowMarker)
        elbowPivot.addChild(elbowMarker)

        let wristMarker = ModelEntity(
            mesh: .generateSphere(radius: 0.025),
            materials: [SimpleMaterial(color: UIColor.green.withAlphaComponent(0.9), isMetallic: false)]
        )
        wristMarker.name = "\(skeletonID)_\(side)WristMarker"
        wristMarker.position = SIMD3<Float>(0, -forearmLength, 0)
        applyNoOcclusion(to: wristMarker)
        elbowPivot.addChild(wristMarker)

        return (shoulderAnchor, shoulderPivot, elbowPivot, upperArmCylinder, forearmCylinder)
    }

    private func buildLegLimb(
        skeletonID: String,
        side: String,
        on contentEntity: Entity
    ) -> (Entity, Entity, Entity, ModelEntity, ModelEntity) {
        let segThigh = "\(side)Thigh"
        let segShank = "\(side)Shank"

        let hipAnchor = Entity()
        hipAnchor.name = "\(skeletonID)_\(side)HipAnchor"
        contentEntity.addChild(hipAnchor)

        let hipPivot = Entity()
        hipPivot.name = "\(skeletonID)_\(side)HipPivot"
        hipAnchor.addChild(hipPivot)

        let thighCylinder = createCylinder(
            height: thighLength, radius: legRadius,
            color: .systemRed, name: "\(skeletonID)_\(segThigh)"
        )
        thighCylinder.position = SIMD3<Float>(0, -thighLength / 2, 0)
        hipPivot.addChild(thighCylinder)

        let hipMarker = ModelEntity(
            mesh: .generateSphere(radius: 0.045),
            materials: [SimpleMaterial(color: UIColor.white.withAlphaComponent(0.9), isMetallic: false)]
        )
        hipMarker.name = "\(skeletonID)_\(side)HipMarker"
        applyNoOcclusion(to: hipMarker)
        hipAnchor.addChild(hipMarker)

        let kneePivot = Entity()
        kneePivot.name = "\(skeletonID)_\(side)KneePivot"
        kneePivot.position = SIMD3<Float>(0, -thighLength, 0)
        hipPivot.addChild(kneePivot)

        let shankCylinder = createCylinder(
            height: shankLength, radius: legRadius * 0.85,
            color: .systemOrange, name: "\(skeletonID)_\(segShank)"
        )
        shankCylinder.position = SIMD3<Float>(0, -shankLength / 2, 0)
        kneePivot.addChild(shankCylinder)

        let kneeMarker = ModelEntity(
            mesh: .generateSphere(radius: 0.04),
            materials: [SimpleMaterial(color: UIColor.yellow.withAlphaComponent(0.9), isMetallic: false)]
        )
        kneeMarker.name = "\(skeletonID)_\(side)KneeMarker"
        applyNoOcclusion(to: kneeMarker)
        kneePivot.addChild(kneeMarker)

        let ankleMarker = ModelEntity(
            mesh: .generateSphere(radius: 0.03),
            materials: [SimpleMaterial(color: UIColor.green.withAlphaComponent(0.9), isMetallic: false)]
        )
        ankleMarker.name = "\(skeletonID)_\(side)AnkleMarker"
        ankleMarker.position = SIMD3<Float>(0, -shankLength, 0)
        applyNoOcclusion(to: ankleMarker)
        kneePivot.addChild(ankleMarker)

        return (hipAnchor, hipPivot, kneePivot, thighCylinder, shankCylinder)
    }

    // MARK: - Helpers

    private func createCylinder(height: Float, radius: Float, color: UIColor, name: String) -> ModelEntity {
        let cylinder = ModelEntity(
            mesh: .generateCylinder(height: height, radius: radius),
            materials: [SimpleMaterial(color: color.withAlphaComponent(0.8), isMetallic: false)]
        )
        cylinder.name = name
        applyNoOcclusion(to: cylinder)
        installLimbCollision(on: cylinder, height: height, radius: radius)
        return cylinder
    }

    /// Adds (or refreshes) a CollisionComponent on a skeleton-cylinder
    /// `ModelEntity` so it emits CollisionEvents when it intersects an
    /// obstacle. Filter: `.skeleton` group, `.obstacle` mask. Shape is a
    /// box bounding the cylinder (axis-aligned in cylinder-local space).
    func installLimbCollision(on cylinder: ModelEntity, height: Float, radius: Float) {
        let h = max(0.02, height)
        let r = max(0.01, radius)
        let shape = ShapeResource.generateBox(width: r * 2, height: h, depth: r * 2)
        var collision = CollisionComponent(shapes: [shape])
        collision.filter = CollisionFilter(group: .skeleton, mask: .obstacle)
        cylinder.components.set(collision)
    }

    private func applyNoOcclusion(to entity: ModelEntity) {
        var modelComponent = entity.components[ModelComponent.self]!
        for i in 0..<modelComponent.materials.count {
            if var material = modelComponent.materials[i] as? SimpleMaterial {
                material.faceCulling = .none
                modelComponent.materials[i] = material
            }
        }
        entity.components.set(modelComponent)
        entity.components.set(ModelSortGroupComponent(group: ModelSortGroup(depthPass: nil), order: 1000))
    }

    // MARK: - Skeleton Positioning

    /// Positions the center skeleton superimposed on the user at the headset position,
    /// offset radially by `radius` in the direction set by `angles[0]` (typically 0).
    func updateAllSkeletonPositions(
        headsetTransform: simd_float4x4,
        radius: Float,
        angles: [Float],
        shoulderVerticalOffset: Float,
        shoulderLateralOffset: Float,
        hipVerticalOffset: Float,
        hipLateralOffset: Float
    ) {
        let headsetPosition = SIMD3<Float>(
            headsetTransform.columns.3.x,
            headsetTransform.columns.3.y,
            headsetTransform.columns.3.z
        )

        let heading = absoluteHeading
        let skeletonIDs = ["center"]

        for (index, skeletonID) in skeletonIDs.enumerated() {
            guard index < angles.count, let instance = skeletons[skeletonID] else { continue }
            let angle = angles[index]

            let radialDir = simd_act(heading, SIMD3<Float>(sin(angle), 0, -cos(angle)))
            let skeletonCenter = headsetPosition + radialDir * radius

            let lateralDir = simd_act(heading, SIMD3<Float>(cos(angle), 0, sin(angle)))

            let shoulderV = SIMD3<Float>(0, shoulderVerticalOffset, 0)
            let hipV = SIMD3<Float>(0, hipVerticalOffset, 0)

            instance.leftShoulderAnchor?.position = skeletonCenter + lateralDir * (-shoulderLateralOffset) + shoulderV
            instance.rightShoulderAnchor?.position = skeletonCenter + lateralDir * shoulderLateralOffset + shoulderV
            instance.leftHipAnchor?.position = skeletonCenter + lateralDir * (-hipLateralOffset) + hipV
            instance.rightHipAnchor?.position = skeletonCenter + lateralDir * hipLateralOffset + hipV
        }
    }

    // MARK: - IMU Orientation

    /// Updates the orientation of a body segment pivot. For child segments
    /// (forearm, shank), computes local rotation relative to the last-known
    /// parent orientation so the cylinder renders in the parent's frame.
    func updateIMUCylinderOrientation(segment: IMUBodySegment, orientation: simd_quatf) {
        let localRotation: simd_quatf?
        switch segment {
        case .leftForearm:
            let p = lastParentOrientations[.leftUpperArm] ?? simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)
            localRotation = p.inverse * orientation
        case .rightForearm:
            let p = lastParentOrientations[.rightUpperArm] ?? simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)
            localRotation = p.inverse * orientation
        case .leftShank:
            let p = lastParentOrientations[.leftThigh] ?? simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)
            localRotation = p.inverse * orientation
        case .rightShank:
            let p = lastParentOrientations[.rightThigh] ?? simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)
            localRotation = p.inverse * orientation
        default:
            localRotation = nil
        }

        for (_, instance) in skeletons {
            switch segment {
            case .leftUpperArm:  instance.leftShoulderPivot?.orientation = orientation
            case .leftForearm:   instance.leftElbowPivot?.orientation = localRotation!
            case .rightUpperArm: instance.rightShoulderPivot?.orientation = orientation
            case .rightForearm:  instance.rightElbowPivot?.orientation = localRotation!
            case .leftThigh:     instance.leftHipPivot?.orientation = orientation
            case .leftShank:     instance.leftKneePivot?.orientation = localRotation!
            case .rightThigh:    instance.rightHipPivot?.orientation = orientation
            case .rightShank:    instance.rightKneePivot?.orientation = localRotation!
            }
        }

        switch segment {
        case .leftUpperArm:  lastParentOrientations[.leftUpperArm] = orientation
        case .rightUpperArm: lastParentOrientations[.rightUpperArm] = orientation
        case .leftThigh:     lastParentOrientations[.leftThigh] = orientation
        case .rightThigh:    lastParentOrientations[.rightThigh] = orientation
        default: break
        }
    }

    // MARK: - Visibility

    func setAllLimbsActive(_ active: Bool) {
        for (_, instance) in skeletons {
            instance.leftShoulderAnchor?.isEnabled = active
            instance.rightShoulderAnchor?.isEnabled = active
            instance.leftHipAnchor?.isEnabled = active
            instance.rightHipAnchor?.isEnabled = active
        }
    }

    /// Upper body (both arms).
    func setUpperLimbsActive(_ active: Bool) {
        for (_, instance) in skeletons {
            instance.leftShoulderAnchor?.isEnabled = active
            instance.rightShoulderAnchor?.isEnabled = active
        }
    }

    /// Lower body (both legs).
    func setLowerLimbsActive(_ active: Bool) {
        for (_, instance) in skeletons {
            instance.leftHipAnchor?.isEnabled = active
            instance.rightHipAnchor?.isEnabled = active
        }
    }

    // MARK: - Runtime Geometry Refresh

    /// Updates all visual cylinders, joint markers, and pivot positions to reflect
    /// current dimension values. Call when control panel values change.
    func refreshGeometry(
        upperArmLength: Float, upperArmRadius: Float,
        forearmLength: Float, forearmRadius: Float,
        thighLength: Float, thighRadius: Float,
        shankLength: Float, shankRadius: Float
    ) {
        for (_, instance) in skeletons {
            // Arms
            updateCylinder(instance.leftUpperArmCylinder, height: upperArmLength, radius: upperArmRadius)
            updateCylinder(instance.rightUpperArmCylinder, height: upperArmLength, radius: upperArmRadius)
            updateCylinder(instance.leftForearmCylinder, height: forearmLength, radius: forearmRadius)
            updateCylinder(instance.rightForearmCylinder, height: forearmLength, radius: forearmRadius)
            // Refresh collision shapes so cylinder-vs-obstacle hit tests
            // track the new mesh dimensions.
            if let c = instance.leftUpperArmCylinder  { installLimbCollision(on: c, height: upperArmLength, radius: upperArmRadius) }
            if let c = instance.rightUpperArmCylinder { installLimbCollision(on: c, height: upperArmLength, radius: upperArmRadius) }
            if let c = instance.leftForearmCylinder   { installLimbCollision(on: c, height: forearmLength, radius: forearmRadius) }
            if let c = instance.rightForearmCylinder  { installLimbCollision(on: c, height: forearmLength, radius: forearmRadius) }

            instance.leftElbowPivot?.position.y = -upperArmLength
            instance.rightElbowPivot?.position.y = -upperArmLength

            instance.leftElbowPivot?.children.first(where: { $0.name.contains("WristMarker") })?.position.y = -forearmLength
            instance.rightElbowPivot?.children.first(where: { $0.name.contains("WristMarker") })?.position.y = -forearmLength

            // Legs
            updateCylinder(instance.leftThighCylinder, height: thighLength, radius: thighRadius)
            updateCylinder(instance.rightThighCylinder, height: thighLength, radius: thighRadius)
            updateCylinder(instance.leftShankCylinder, height: shankLength, radius: shankRadius)
            updateCylinder(instance.rightShankCylinder, height: shankLength, radius: shankRadius)
            if let c = instance.leftThighCylinder  { installLimbCollision(on: c, height: thighLength, radius: thighRadius) }
            if let c = instance.rightThighCylinder { installLimbCollision(on: c, height: thighLength, radius: thighRadius) }
            if let c = instance.leftShankCylinder  { installLimbCollision(on: c, height: shankLength, radius: shankRadius) }
            if let c = instance.rightShankCylinder { installLimbCollision(on: c, height: shankLength, radius: shankRadius) }

            instance.leftKneePivot?.position.y = -thighLength
            instance.rightKneePivot?.position.y = -thighLength

            instance.leftKneePivot?.children.first(where: { $0.name.contains("AnkleMarker") })?.position.y = -shankLength
            instance.rightKneePivot?.children.first(where: { $0.name.contains("AnkleMarker") })?.position.y = -shankLength
        }
    }

    // MARK: - Limb contact points (for proximity-based distance queries)

    /// Returns the world-space point on each limb that should be used for
    /// obstacle-distance calculations. Per-limb policy:
    ///   - Upper arms / thighs: cylinder midpoint (the limb segment center).
    ///   - Forearms: distal end (wrist marker — closest to the hand the
    ///     user actually leads with toward an obstacle).
    ///   - Shanks: distal end (ankle marker — closest to the foot, which
    ///     is what touches a low obstacle like a curb first).
    /// Used by BodyTrackingModel's per-frame proximity loop.
    func limbContactPoints() -> [IMUBodySegment: SIMD3<Float>] {
        guard let instance = skeletons["center"] else { return [:] }
        var out: [IMUBodySegment: SIMD3<Float>] = [:]
        if let e = instance.leftUpperArmCylinder  { out[.leftUpperArm]  = e.position(relativeTo: nil) }
        if let e = instance.rightUpperArmCylinder { out[.rightUpperArm] = e.position(relativeTo: nil) }
        if let e = instance.leftThighCylinder     { out[.leftThigh]     = e.position(relativeTo: nil) }
        if let e = instance.rightThighCylinder    { out[.rightThigh]    = e.position(relativeTo: nil) }

        // Forearm distal end = wrist marker (child of elbow pivot). Falls
        // back to the cylinder midpoint if the marker isn't found.
        if let pivot = instance.leftElbowPivot,
           let wrist = pivot.children.first(where: { $0.name.contains("WristMarker") }) {
            out[.leftForearm] = wrist.position(relativeTo: nil)
        } else if let e = instance.leftForearmCylinder {
            out[.leftForearm] = e.position(relativeTo: nil)
        }
        if let pivot = instance.rightElbowPivot,
           let wrist = pivot.children.first(where: { $0.name.contains("WristMarker") }) {
            out[.rightForearm] = wrist.position(relativeTo: nil)
        } else if let e = instance.rightForearmCylinder {
            out[.rightForearm] = e.position(relativeTo: nil)
        }

        // Shank distal end = ankle marker (child of knee pivot). Same
        // fallback to the cylinder midpoint if missing.
        if let pivot = instance.leftKneePivot,
           let ankle = pivot.children.first(where: { $0.name.contains("AnkleMarker") }) {
            out[.leftShank] = ankle.position(relativeTo: nil)
        } else if let e = instance.leftShankCylinder {
            out[.leftShank] = e.position(relativeTo: nil)
        }
        if let pivot = instance.rightKneePivot,
           let ankle = pivot.children.first(where: { $0.name.contains("AnkleMarker") }) {
            out[.rightShank] = ankle.position(relativeTo: nil)
        } else if let e = instance.rightShankCylinder {
            out[.rightShank] = e.position(relativeTo: nil)
        }

        return out
    }

    private func updateCylinder(_ entity: ModelEntity?, height: Float, radius: Float) {
        guard let entity = entity else { return }
        if var modelComp = entity.components[ModelComponent.self] {
            modelComp.mesh = .generateCylinder(height: height, radius: radius)
            entity.components.set(modelComp)
        }
        entity.position.y = -height / 2
    }

    // MARK: - Collision Indicator

    /// Changes a cylinder color to green on collision, or restores the original color when cleared.
    func setSegmentCollisionIndicator(skeletonID: String, segment: IMUBodySegment, isColliding: Bool) {
        guard let instance = skeletons[skeletonID] else { return }

        let cylinder: ModelEntity?
        let defaultColor: UIColor

        switch segment {
        case .leftUpperArm:
            cylinder = instance.leftUpperArmCylinder; defaultColor = .systemBlue
        case .leftForearm:
            cylinder = instance.leftForearmCylinder; defaultColor = .systemCyan
        case .rightUpperArm:
            cylinder = instance.rightUpperArmCylinder; defaultColor = .systemBlue
        case .rightForearm:
            cylinder = instance.rightForearmCylinder; defaultColor = .systemCyan
        case .leftThigh:
            cylinder = instance.leftThighCylinder; defaultColor = .systemRed
        case .leftShank:
            cylinder = instance.leftShankCylinder; defaultColor = .systemOrange
        case .rightThigh:
            cylinder = instance.rightThighCylinder; defaultColor = .systemRed
        case .rightShank:
            cylinder = instance.rightShankCylinder; defaultColor = .systemOrange
        }

        guard let cyl = cylinder else { return }
        let color: UIColor = isColliding ? .systemGreen : defaultColor
        let newMaterial = SimpleMaterial(color: color.withAlphaComponent(0.8), isMetallic: false)

        if var modelComponent = cyl.components[ModelComponent.self] {
            modelComponent.materials = [newMaterial]
            cyl.components.set(modelComponent)
        }
    }

    // MARK: - Hand Tracking

    func processHandUpdates() async {
        for await update in handTracking.anchorUpdates {
            let handAnchor = update.anchor
            guard handAnchor.isTracked, let handSkeleton = handAnchor.handSkeleton else { continue }

            let jointsToAverage: [HandSkeleton.JointName] = [
                .indexFingerMetacarpal, .middleFingerMetacarpal, .ringFingerMetacarpal,
                .littleFingerMetacarpal, .thumbKnuckle, .forearmWrist
            ]

            var positions: [SIMD3<Float>] = []
            for jointName in jointsToAverage {
                let joint = handSkeleton.joint(jointName)
                if joint.isTracked {
                    let worldMat = handAnchor.originFromAnchorTransform * joint.anchorFromJointTransform
                    let pos = SIMD3<Float>(worldMat.columns.3.x, worldMat.columns.3.y, worldMat.columns.3.z)
                    positions.append(pos)
                }
            }
            guard !positions.isEmpty else { continue }

            let sum = positions.reduce(SIMD3<Float>(repeating: 0), +)
            let avg = sum / Float(positions.count)

            var palmTransform = Transform(matrix: handAnchor.originFromAnchorTransform)
            palmTransform.translation = avg

            let projectionDistance: Float = 0.1
            let forward = SIMD3<Float>(palmTransform.matrix.columns.0.x,
                                       palmTransform.matrix.columns.0.y,
                                       palmTransform.matrix.columns.0.z)
            let directionMultiplier: Float = (handAnchor.chirality == .right) ? -1.0 : 1.0
            let projectedPosition = avg + forward * (projectionDistance * directionMultiplier)
            palmTransform.translation = projectedPosition

            if let palm = palmEntities[handAnchor.chirality] {
                palm.transform = palmTransform
            }
        }
    }
}
