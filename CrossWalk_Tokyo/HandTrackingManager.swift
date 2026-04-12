import ARKit
import RealityKit
import UIKit

/// Holds all entities for one complete skeleton copy (center, left, or right).
struct SkeletonInstance {
    let id: String  // "center", "left", "right"

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

    // Palm entities: one ModelEntity per hand (left/right).
    private(set) var palmEntities: [HandAnchor.Chirality: ModelEntity] = [:]

    // IMU-tracked body segments (8 segments: both arms + both legs)
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

    /// Defines the four limb groups that can be independently shown/hidden.
    enum LimbGroup {
        case leftArm
        case rightArm
        case leftLeg
        case rightLeg
    }

    // MARK: - Skeleton Instances

    var skeletons: [String: SkeletonInstance] = [:]

    // Segment lengths (in meters)
    private let upperArmLength: Float = 0.28
    private let forearmLength: Float = 0.25
    private let thighLength: Float = 0.45
    private let shankLength: Float = 0.17

    // Cylinder radii
    let armRadius: Float = 0.05
    let legRadius: Float = 0.06

    // MARK: - Detection Trigger Volumes
    var upperArmDetectionRadius: Float = 0.08
    var upperArmDetectionHeight: Float = 0.28
    var forearmDetectionRadius: Float = 0.08
    var forearmDetectionHeight: Float = 0.25
    var thighDetectionRadius: Float = 0.10
    var thighDetectionHeight: Float = 0.45
    var shankDetectionRadius: Float = 0.10
    var shankDetectionHeight: Float = 0.17

    // Store the last known absolute world orientations for parent segments.
    private var lastParentOrientations: [IMUBodySegment: simd_quatf] = [
        .leftUpperArm: simd_quatf(ix: 0, iy: 0, iz: 0, r: 1),
        .rightUpperArm: simd_quatf(ix: 0, iy: 0, iz: 0, r: 1),
        .leftThigh: simd_quatf(ix: 0, iy: 0, iz: 0, r: 1),
        .rightThigh: simd_quatf(ix: 0, iy: 0, iz: 0, r: 1),
    ]

    /// Chest yaw delta: rotation from calibration-time facing to current chest facing.
    var chestYawDelta: simd_quatf = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)
    var chestYawDeltaInverse: simd_quatf = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)

    /// Calibration heading: yaw-only rotation representing which direction the user faced
    /// during calibration.
    var calibrationHeadingQ: simd_quatf = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)

    /// Absolute heading = calibration heading + chest yaw delta.
    var absoluteHeading: simd_quatf {
        calibrationHeadingQ * chestYawDelta
    }

    func getForwardProjectionVector(forwardOffset: Float) -> SIMD3<Float> {
        return simd_act(absoluteHeading, SIMD3<Float>(0, 0, forwardOffset))
    }

    // MARK: - Setup

    func setupPalms(on contentEntity: Entity) {
        for chirality in [HandAnchor.Chirality.left, HandAnchor.Chirality.right] {
            let placeholder = ModelEntity()
            placeholder.name = "palm_\(chirality)"
            palmEntities[chirality] = placeholder
            contentEntity.addChild(placeholder)
        }

        for skeletonID in ["center", "left", "right"] {
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

        let uaDetection = createTriggerVolume(
            name: "\(skeletonID)_segmentDetectionTrigger_\(segUA)",
            radius: upperArmDetectionRadius, height: upperArmDetectionHeight, isHorizontal: false
        )
        uaDetection.position = SIMD3<Float>(0, -upperArmDetectionHeight / 2, 0)
        shoulderPivot.addChild(uaDetection)

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

        let faDetection = createTriggerVolume(
            name: "\(skeletonID)_segmentDetectionTrigger_\(segFA)",
            radius: forearmDetectionRadius, height: forearmDetectionHeight, isHorizontal: false
        )
        faDetection.position = SIMD3<Float>(0, -forearmDetectionHeight / 2, 0)
        elbowPivot.addChild(faDetection)

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

        let thighDetection = createTriggerVolume(
            name: "\(skeletonID)_segmentDetectionTrigger_\(segThigh)",
            radius: thighDetectionRadius, height: thighDetectionHeight, isHorizontal: false
        )
        thighDetection.position = SIMD3<Float>(0, -thighDetectionHeight / 2, 0)
        hipPivot.addChild(thighDetection)

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

        let shankDetection = createTriggerVolume(
            name: "\(skeletonID)_segmentDetectionTrigger_\(segShank)",
            radius: shankDetectionRadius, height: shankDetectionHeight, isHorizontal: false
        )
        shankDetection.position = SIMD3<Float>(0, -shankDetectionHeight / 2, 0)
        kneePivot.addChild(shankDetection)

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
        return cylinder
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

    private func createTriggerVolume(name: String, radius: Float, height: Float, isHorizontal: Bool) -> Entity {
        let shape = ShapeResource.generateCapsule(height: height + radius * 2, radius: radius)

        let trigger = TriggerVolume(shape: shape)
        trigger.name = name

        var collisionComponent = trigger.collision ?? CollisionComponent(shapes: [shape])
        collisionComponent.filter = CollisionFilter(group: .skeleton, mask: .obstacle)
        trigger.components.set(collisionComponent)

        if isHorizontal {
            trigger.orientation = simd_quatf(angle: .pi / 2, axis: SIMD3<Float>(0, 0, 1))
        }

        return trigger
    }

    // MARK: - Skeleton Positioning

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
        let skeletonIDs = ["center", "left", "right"]

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
        for (skeletonID, instance) in skeletons {
            instance.leftShoulderAnchor?.isEnabled = active
            instance.rightShoulderAnchor?.isEnabled = active
            instance.leftHipAnchor?.isEnabled = active
            instance.rightHipAnchor?.isEnabled = active
            _ = skeletonID
        }
    }

    // MARK: - Collision Indicator

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
