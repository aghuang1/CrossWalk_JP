/*
 BodyTrackingModel.swift
 CrossWalk_Tokyo

 Central orchestrator for body limb tracking, IMU calibration, collision detection,
 and haptic motor control. Ported from ExtendedTouch_AVP's EntityModel.swift
 (isotropicExpansion branch).

 CrossWalk-specific differences vs. ExtendedTouch:
 - Uses IMUUDPClient instead of UDPClient (compatible API).
 - No SceneReconstructionManager / PlaneDetectionManager (no mesh scanning).
 - No floor-collision filtering or hand-level haptic state machine.
*/

import ARKit
import RealityKit
import UIKit
import Combine
import Network

// MARK: - Node ID to Body Segment Mapping

// RPi relay address — all commands are sent here; RPi forwards to the correct node
//let rpiIP = "192.168.1.7"
let rpiIP = "172.20.10.7"

// IMU node IDs (byte 0 of each packet) -> segment name
let nodeIDToSegment: [String: String] = [
    "1": "leftForearm",
    "2": "leftUpperArm",
    "3": "chest",
    "4": "rightForearm",
    "5": "rightUpperArm",
    "6": "leftThigh",
    "7": "leftShank",
    "8": "rightThigh",
    "9": "rightShank",
]

let segmentToNodeID: [String: String] = {
    var result: [String: String] = [:]
    for (id, seg) in nodeIDToSegment {
        result[seg] = id
    }
    return result
}()

let activeSegments: [String] = Array(segmentToNodeID.keys)
let legSegments: Set<String>   = ["leftThigh", "leftShank", "rightThigh", "rightShank"]
let thighSegments: Set<String> = ["leftThigh", "rightThigh"]
let shankSegments: Set<String> = ["leftShank", "rightShank"]
let chestSegment: String = "chest"

// Single IMU UDP client for all connections
let imuClient = IMUUDPClient(
    deviceConnections: [rpiIP],
    remotePort: 61616,
    localPort: 61616
)

// MARK: - BodyTrackingModel

@Observable
@MainActor
class BodyTrackingModel {
    let session = ARKitSession()
    let worldTracking = WorldTrackingProvider()
    let handManager = HandTrackingManager()

    var errorMessage: String?
    private var contentEntity = Entity()
    private var hasSetupContentEntity = false
    private var hasAttachedToScene = false
    private var lastHeadsetTransform: simd_float4x4?

    // Combine subscriptions
    nonisolated(unsafe) private var orientationSubscription: AnyCancellable?
    nonisolated(unsafe) private var collisionBeganSubscription: (any Cancellable)?
    nonisolated(unsafe) private var collisionEndedSubscription: (any Cancellable)?
    nonisolated(unsafe) private var skeletonTrackingSubscription: (any Cancellable)?

    // MARK: - Collision & Motor State (algorithm 3: proximity trigger + distance math)

    /// Obstacles currently inside the headset proximity trigger. Populated by
    /// `CollisionEvents.Began/.Ended`. Each frame we compute per-limb distance
    /// to each tracked obstacle, pick the nearest, and drive motors accordingly.
    private var trackedObstacles: Set<Entity> = []

    /// Most recently sent motor level per segment, used for UDP dedup.
    /// nil = motor is OFF.
    private var motorCurrentLevel: [String: String?] = {
        var d: [String: String?] = [:]
        for seg in activeSegments { d[seg] = nil }
        return d
    }()

    var motorIsOn: [String: Bool] = {
        var d: [String: Bool] = [:]
        for seg in activeSegments { d[seg] = false }
        return d
    }()

    nonisolated(unsafe) private var motorOffDebounceTimers: [String: Timer] = [:]
    private let motorOffDebounceDelay: TimeInterval = 1.0

    private var lastSentCommand: [String: String] = [:]

    // Headset-anchored proximity trigger (sphere). Obstacles entering it get
    // added to `trackedObstacles`; per-frame distance math maps them to limbs.
    private var proximityTrigger: Entity?
    var proximityTriggerRadius: Float = 3.0 { didSet { geometryNeedsRefresh = true } }

    // Body-sized trigger, also headset-anchored, sized to the user's
    // shoulder radius. A CollisionEvents.Began on this trigger is the source
    // of truth for COLLIDED / VICTORY — no per-limb distance math needed, so
    // torso and head contacts register even when the arms aren't extended.
    private var bodyCollisionTrigger: Entity?
    // Sphere radius in meters. Default ~half a shoulder width (≈0.45m across).
    var bodyCollisionRadius: Float = 0.10 { didSet { geometryNeedsRefresh = true } }

    // Proximity visualizers: per-limb sphere at the closest obstacle surface
    // point + thin cylinder from limb midpoint to that point. Color-coded by
    // distance bucket (red/yellow/blue). Hidden when a limb is out of range.
    var showProximityVisualizers: Bool = true
    private var closestPointMarkers: [HandTrackingManager.IMUBodySegment: ModelEntity] = [:]
    private var distanceConnectors: [HandTrackingManager.IMUBodySegment: ModelEntity] = [:]
    private let connectorBaseHeight: Float = 1.0

    // Distance buckets (meters). Distance is from a limb midpoint to the closest
    // point on an obstacle's AABB.
    //   CLOSE = [0, distCloseMax)
    //   MED   = [distCloseMax, distMedMax)
    //   FAR   = [distMedMax, distFarMax]
    //   OFF   > distFarMax
    var distCloseMax: Float = 0.5
    var distMedMax: Float = 1.0
    var distFarMax: Float = 2.0

    // Test obstacle tunables (read live by the ImmersiveView car update loop).
    // carSpeed: m/s the cars travel along the R→L track.
    // carDistance: forward distance (meters) from the user origin to the car lane;
    //   stored as a positive value, applied as -Z in world space.
    var carSpeed: Float = 3.0
    var carDistance: Float = 2.0
    // Independent multipliers for the car hitbox (collision shape) and the
    // toy-car visual mesh. Default 1.0 each. Read live by the ImmersiveView
    // timer so panel changes apply immediately without restarting the run.
    var carVisualScale: Float = 2.5
    var carHitboxScale: Float = 0.9

    // Limbs that may receive vibrotactile feedback. Per obstacle, only the single
    // closest limb in this set vibrates — preventing both arms (etc.) from firing
    // on the same nearby object. Scale by adding more `IMUBodySegment` cases.
    private let motorEligibleLimbs: Set<HandTrackingManager.IMUBodySegment> = [
        .leftUpperArm, .rightUpperArm
    ]

    // Collision display. `lastCollisionTime` / `lastVictoryTime` are bumped
    // by `registerBodyContact` whenever the shoulder-width body trigger
    // overlaps an obstacle. The UI shows the banner for
    // `collisionDisplayDuration` seconds after each bump.
    let collisionDisplayDuration: CFTimeInterval = 5.0
    var lastCollisionTime: CFTimeInterval = -.infinity
    let victoryDisplayDuration: CFTimeInterval = 5.0
    var lastVictoryTime: CFTimeInterval = -.infinity

    // Run lifecycle + stats. `runStartTime == nil` before the user has pressed
    // START; `runEndTime != nil` once VICTORY has fired. `isRunActive` is
    // true only between those two events — during which time the cars move,
    // spawn counting accumulates, and contacts are tallied.
    var runStartTime: CFTimeInterval? = nil
    var runEndTime: CFTimeInterval? = nil
    var totalCarsSpawned: Int = 0
    // Per-pass car IDs. The caller (ImmersiveView) renames each car as
    // "CarCube_<lane>_p<passIndex>" on cycle rollovers so each pass is a
    // distinct key here, even though the underlying entity is reused.
    var carsHitInstanceIDs: Set<String> = []

    var isRunActive: Bool {
        runStartTime != nil && runEndTime == nil
    }
    var runDuration: CFTimeInterval? {
        guard let s = runStartTime, let e = runEndTime else { return nil }
        return e - s
    }
    var collisionRatio: Double {
        guard totalCarsSpawned > 0 else { return 0 }
        return Double(carsHitInstanceIDs.count) / Double(totalCarsSpawned)
    }

    func startRun() {
        runStartTime = CACurrentMediaTime()
        runEndTime = nil
        totalCarsSpawned = 0
        carsHitInstanceIDs.removeAll()
        lastCollisionTime = -.infinity
        lastVictoryTime = -.infinity
    }

    func recordCarSpawn() {
        guard isRunActive else { return }
        totalCarsSpawned += 1
    }

    func recordCarContact(_ instanceID: String) {
        guard isRunActive else { return }
        carsHitInstanceIDs.insert(instanceID)
    }

    func recordVictory() {
        guard isRunActive else { return }
        runEndTime = CACurrentMediaTime()
    }

    private var imuStreamingSegments: Set<String> = []

    let shellMotorLabels: [String] = ["CLOSE", "MED", "FAR"]

    // MARK: - Manual Override Flags

    var isMotorEnabled: Bool = false
    var isIMUOverrideActive: Bool = false

    var isCalibrated: Bool { calibrationState == .calibrated }
    var isCalibrating: Bool { calibrationState != .notCalibrated && calibrationState != .calibrated }

    var calibrationStatusTitle: String = "Not Calibrated"
    var calibrationStatusDetail: String = "Press Re-Calibrate to begin two-pose calibration."
    var calibrationCountdownSeconds: Int?
    var calibrationCountdownTotalSeconds: Int?
    var calibrationProgressFraction: Double {
        guard let remaining = calibrationCountdownSeconds,
              let total = calibrationCountdownTotalSeconds,
              total > 0 else {
            return 0
        }
        return min(1.0, max(0.0, Double(total - remaining) / Double(total)))
    }

    // MARK: - Skeleton Positioning (runtime-adjustable)

    var skeletonForwardOffset: Float = -0.05
    var shoulderVerticalOffset: Float = -0.20
    var shoulderLateralOffset: Float = 0.25
    var hipVerticalOffset: Float = -0.70
    var hipLateralOffset: Float = 0.10

    // Superimposed skeleton: zero radius, single angle (facing forward).
    var skeletonRadius: Float = 0.0
    var skeletonAngles: [Float] = [0]

    // Dirty flag: set true when any dimension changes; checked in frame loop.
    var geometryNeedsRefresh: Bool = false

    var upperArmLength: Float = 0.28 { didSet { geometryNeedsRefresh = true } }
    var upperArmRadius: Float = 0.08 { didSet { geometryNeedsRefresh = true } }
    var forearmLength: Float = 0.25 { didSet { geometryNeedsRefresh = true } }
    var forearmRadius: Float = 0.08 { didSet { geometryNeedsRefresh = true } }
    var thighLength: Float = 0.45 { didSet { geometryNeedsRefresh = true } }
    var thighRadius: Float = 0.10 { didSet { geometryNeedsRefresh = true } }
    var shankLength: Float = 0.17 { didSet { geometryNeedsRefresh = true } }
    var shankRadius: Float = 0.10 { didSet { geometryNeedsRefresh = true } }

    // MARK: - IMU Calibration (Two-Pose, SlimeVR-style left/right split)
    //
    // Pose 1: STANDING — all limbs hanging straight down.
    // Pose 2: SITTING with arms forward — thighs parallel to ground,
    //         arms extended straight forward (parallel to ground).
    //
    // Per sensor we produce TWO corrections:
    //   leftFix  — world-frame rotation (ENU→AVP basis + yaw alignment). Left-multiplied on raw.
    //   rightFix — body-frame rotation (preRotation · axialFix). Right-multiplied on raw.
    //
    // Runtime formula (per sensor i):
    //   q_calibrated^(i) = leftFix^(i) · raw^(i) · rightFix^(i)

    enum CalibrationState {
        case notCalibrated
        case waitingForPose1
        case pose1Captured
        case waitingForPose2
        case calibrated
    }

    private var calibrationState: CalibrationState = .notCalibrated
    private var pose1IMUData: [String: simd_quatf] = [:]
    private var pose2IMUData: [String: simd_quatf] = [:]
    private var imuLeftFix:  [String: simd_quatf] = [:]
    private var imuRightFix: [String: simd_quatf] = [:]
    private var imuLimbDownAxis: [String: SIMD3<Float>] = [:]

    // Yaw-drift auto-compensation on STOP→START cycles. BNO086 re-anchors its yaw
    // reference on stream restart, which invalidates the per-sensor leftFix solved
    // at calibration time. Snapshot each limb's raw before STOP; on first packet
    // after START, fold inverse-drift into leftFix.
    private var rawBeforeStop: [String: simd_quatf] = [:]
    private var pendingYawResync: Set<String> = []

    // Last raw quaternion dispatched per segment. Suppresses re-processing of stale
    // segments when UDP delivers one segment per packet — otherwise an upper-arm
    // packet refreshes lastParentOrientations and then the forearm is re-dispatched
    // with a time-mismatched (fresh parent, stale child) pair.
    private var lastDispatchedRaw: [String: simd_quatf] = [:]

    private var lastQuatLogTime: [String: CFTimeInterval] = [:]
    private var lastIMUOrientations: [String: simd_quatf] = [:]
    private var calibrationHeadsetYaw: Float = 0.0
    private var calibrationHeadingQ: simd_quatf = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)
    private var calibrationHeadsetForwardAVP: SIMD3<Float> = SIMD3<Float>(0, 0, -1)
    private var chestForwardLocalAxis: SIMD3<Float> = SIMD3<Float>(0, 0, -1)
    private var autoCalibrationScheduled: Bool = false
    private let pose1HoldTime: TimeInterval = 3.0
    private let pose2HoldTime: TimeInterval = 5.0
    private var calibrationCountdownTask: Task<Void, Never>?
    private var autoCalibrationSampleCount: Int = 0

    private var isIMUCalibrated: Bool {
        return calibrationState == .calibrated
    }

    // Chest yaw smoothing
    private var chestYawTarget: simd_quatf = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)
    private var chestYawDisplay: simd_quatf = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)
    private var hasChestYawTarget: Bool = false
    private let chestYawBlend: Float = 0.15

    // Chest yaw reference captured from the first post-calibration chest packet.
    // chestYawDelta is measured relative to THIS — not to calibrationHeadingQ —
    // so any residual chest-solve mismatch cancels out and the skeleton starts
    // aligned with calibrationHeadingQ regardless of chest-solve Gram-Schmidt fit.
    private var chestYawReference: simd_quatf = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)
    private var hasChestYawReference: Bool = false

    // MARK: - Setup

    func setupContentEntity() -> Entity {
        guard !hasSetupContentEntity else { return contentEntity }
        hasSetupContentEntity = true

        updateCalibrationStatus(
            title: "Waiting for IMUs",
            detail: "Connect sensors and hold still. Calibration will auto-start when packets arrive."
        )

        handManager.setupPalms(on: contentEntity)

        print("Sending BEGIN_CALIBRATION to RPi at launch...")
        imuClient.sendMessage(to: rpiIP, message: "BEGIN_CALIBRATION")

        setupIMUOrientationSubscription()

        // All limbs start hidden; shown after calibration completes.
        handManager.setAllLimbsActive(false)

        // One headset-anchored sphere proximity trigger. Obstacles entering it
        // get tracked; per-frame distance math handles per-limb motor output.
        let sphere = ShapeResource.generateSphere(radius: proximityTriggerRadius)
        let proximity = TriggerVolume(shape: sphere)
        proximity.name = "headsetProximityTrigger"
        var proximityCollision = proximity.collision ?? CollisionComponent(shapes: [sphere])
        proximityCollision.filter = CollisionFilter(group: .skeleton, mask: .obstacle)
        proximity.components.set(proximityCollision)
        contentEntity.addChild(proximity)
        proximityTrigger = proximity

        // Shoulder-width body trigger, concentric with the proximity sphere
        // but small enough to represent the user's actual silhouette. Its
        // Began event is treated as direct physical contact (COLLIDED or
        // VICTORY), so detection never depends on limb-tip math.
        let bodyShape = ShapeResource.generateSphere(radius: bodyCollisionRadius)
        let body = TriggerVolume(shape: bodyShape)
        body.name = "bodyCollisionTrigger"
        var bodyCollision = body.collision ?? CollisionComponent(shapes: [bodyShape])
        bodyCollision.filter = CollisionFilter(group: .skeleton, mask: .obstacle)
        body.components.set(bodyCollision)
        contentEntity.addChild(body)
        bodyCollisionTrigger = body

        // One visualizer pair (marker sphere + thin connector cylinder) per limb.
        // Hidden at startup; shown once calibration + proximity tracking are active.
        for imuSeg in HandTrackingManager.IMUBodySegment.allCases {
            let marker = ModelEntity(
                mesh: .generateSphere(radius: 0.04),
                materials: [UnlitMaterial(color: .white)]
            )
            marker.name = "closestPointMarker_\(imuSeg.rawValue)"
            marker.isEnabled = false
            contentEntity.addChild(marker)
            closestPointMarkers[imuSeg] = marker

            let connector = ModelEntity(
                mesh: .generateCylinder(height: connectorBaseHeight, radius: 0.006),
                materials: [UnlitMaterial(color: .white)]
            )
            connector.name = "distanceConnector_\(imuSeg.rawValue)"
            connector.isEnabled = false
            contentEntity.addChild(connector)
            distanceConnectors[imuSeg] = connector
        }

        return contentEntity
    }

    func attachToSceneIfReady() {
        guard !hasAttachedToScene, contentEntity.scene != nil else { return }
        hasAttachedToScene = true

        guard let scene = contentEntity.scene else { return }

        collisionBeganSubscription = scene.subscribe(to: CollisionEvents.Began.self) { [weak self] event in
            self?.handleCollisionBegan(event)
        }
        collisionEndedSubscription = scene.subscribe(to: CollisionEvents.Ended.self) { [weak self] event in
            self?.handleCollisionEnded(event)
        }

        // Update skeleton positions every render frame (90Hz on Vision Pro)
        skeletonTrackingSubscription = scene.subscribe(to: SceneEvents.Update.self) { [weak self] _ in
            guard let self = self else { return }

            let now = CACurrentMediaTime()

            if let deviceAnchor = self.worldTracking.queryDeviceAnchor(atTimestamp: now) {
                let cameraTransform = deviceAnchor.originFromAnchorTransform
                self.lastHeadsetTransform = cameraTransform
                self.handManager.updateAllSkeletonPositions(
                    headsetTransform: cameraTransform,
                    radius: self.skeletonRadius,
                    angles: self.skeletonAngles,
                    shoulderVerticalOffset: self.shoulderVerticalOffset,
                    shoulderLateralOffset: self.shoulderLateralOffset,
                    hipVerticalOffset: self.hipVerticalOffset,
                    hipLateralOffset: self.hipLateralOffset
                )

                // Keep the proximity trigger centered on the headset. Sphere is
                // rotationally symmetric so no orientation update needed.
                let headPos = SIMD3<Float>(cameraTransform.columns.3.x, cameraTransform.columns.3.y, cameraTransform.columns.3.z)
                self.proximityTrigger?.position = headPos
                // Anchor the body trigger ~40 cm below the head — roughly chest
                // level — so a small (e.g. 25 cm) sphere centered there actually
                // intersects a real car's hitbox (which sits below head height)
                // instead of floating above it.
                self.bodyCollisionTrigger?.position = SIMD3<Float>(headPos.x, headPos.y - 0.4, headPos.z)

                if self.geometryNeedsRefresh {
                    self.geometryNeedsRefresh = false
                    self.handManager.refreshGeometry(
                        upperArmLength: self.upperArmLength, upperArmRadius: self.upperArmRadius,
                        forearmLength: self.forearmLength, forearmRadius: self.forearmRadius,
                        thighLength: self.thighLength, thighRadius: self.thighRadius,
                        shankLength: self.shankLength, shankRadius: self.shankRadius
                    )
                    self.refreshProximityTrigger()
                    self.refreshBodyCollisionTrigger()
                }
            }

            // Per-frame: recompute per-limb min distance to each tracked obstacle,
            // quantize to {close, med, far, off}, and send motor commands.
            self.updateMotorsByProximity()

            // Chest yaw: SLERP toward target each frame for smooth rotation.
            if self.hasChestYawTarget {
                var target = self.chestYawTarget
                if simd_dot(self.chestYawDisplay.vector, target.vector) < 0 {
                    target = simd_quatf(ix: -target.imag.x, iy: -target.imag.y,
                                        iz: -target.imag.z, r: -target.real)
                }
                self.chestYawDisplay = simd_slerp(self.chestYawDisplay, target, self.chestYawBlend)
                self.handManager.chestYawDelta = self.chestYawDisplay
                self.handManager.chestYawDeltaInverse = self.chestYawDisplay.inverse
            }
        }
    }

    // MARK: - IMU Orientation Subscription

    private func setupIMUOrientationSubscription() {
        orientationSubscription = imuClient.$orientations
            .receive(on: RunLoop.main)
            .sink { [weak self] orientations in
                guard let self = self else { return }
                if orientations.isEmpty { return }

                self.autoCalibrationSampleCount += 1
                if self.autoCalibrationSampleCount % 100 == 1 {
                    print("[IMU] receiving \(orientations.count) segment(s)")
                }

                // Cache latest raw orientations and run yaw-resync on first packet after restart.
                for (nodeID, quat) in orientations {
                    if let segName = nodeIDToSegment[nodeID] {
                        if let prev = self.lastIMUOrientations[segName] {
                            let dot = abs(prev.real * quat.real + prev.imag.x * quat.imag.x + prev.imag.y * quat.imag.y + prev.imag.z * quat.imag.z)
                            let angleDeg = 2 * acos(min(dot, 1.0)) * 180 / .pi
                            if angleDeg > 10 {
                                print("[\(segName)] raw jump \(String(format: "%.1f", angleDeg))°")
                            }
                        }
                        if self.isIMUCalibrated {
                            self.applyYawResyncIfNeeded(segment: segName, currentRaw: quat)
                        }
                        self.lastIMUOrientations[segName] = quat
                    }
                }

                // Auto-start calibration when first IMU data arrives.
                if !self.autoCalibrationScheduled && self.calibrationState == .notCalibrated && !orientations.isEmpty {
                    self.autoCalibrationScheduled = true
                    print("IMU data received. Starting calibration in 3 seconds...")

                    self.calibrationStatusTitle = "IMUs Connected"
                    self.calibrationStatusDetail = "Starting calibration. Prepare pose 1: stand still with arms down."
                    self.calibrationCountdownSeconds = 3

                    Task { @MainActor in
                        try? await Task.sleep(nanoseconds: 3_000_000_000)
                        if self.calibrationState == .notCalibrated {
                            self.startCalibration()
                        }
                    }
                }

                // Dispatch calibrated orientations to active segments.
                // Two-pass: parents first so lastParentOrientations is fresh before children.
                guard self.isIMUCalibrated else { return }

                let parentSegments: Set<String> = ["leftUpperArm", "rightUpperArm", "leftThigh", "rightThigh"]

                for pass in 0..<2 {
                    for (nodeID, rawQuat) in orientations {
                        guard let segmentName = nodeIDToSegment[nodeID] else { continue }
                        let isParent = parentSegments.contains(segmentName)
                        if pass == 0 && !isParent && segmentName != chestSegment { continue }
                        if pass == 1 && (isParent || segmentName == chestSegment) { continue }

                        // Skip stale segments.
                        if let prev = self.lastDispatchedRaw[segmentName],
                           prev.vector == rawQuat.vector {
                            continue
                        }
                        self.lastDispatchedRaw[segmentName] = rawQuat

                        // Chest: extract yaw for skeleton rotation only.
                        if segmentName == chestSegment {
                            let calibrated = self.applyIMUCalibration(segment: segmentName, raw: rawQuat)
                            self.updateChestYaw(calibrated: calibrated)
                            continue
                        }

                        let calibrated = self.applyIMUCalibration(segment: segmentName, raw: rawQuat)
                        self.logSegmentIfDue(segment: segmentName, raw: rawQuat, calibrated: calibrated)

                        // Update parent orientation before children use it.
                        if isParent, let imuSeg = HandTrackingManager.IMUBodySegment(rawValue: segmentName) {
                            self.handManager.updateParentOrientation(segment: imuSeg, orientation: calibrated)
                        }

                        if let imuSeg = HandTrackingManager.IMUBodySegment(rawValue: segmentName) {
                            self.handManager.updateIMUCylinderOrientation(segment: imuSeg, orientation: calibrated)
                        }
                    }
                }
            }
    }

    // MARK: - Two-Pose Calibration

    private func updateCalibrationStatus(title: String, detail: String, countdown: Int? = nil) {
        calibrationStatusTitle = title
        calibrationStatusDetail = detail
        calibrationCountdownSeconds = countdown
        calibrationCountdownTotalSeconds = countdown
    }

    private func startCalibrationCountdown(
        seconds: Int,
        expectedState: CalibrationState,
        onComplete: @escaping @MainActor () -> Void
    ) {
        calibrationCountdownTask?.cancel()
        calibrationCountdownTotalSeconds = seconds
        calibrationCountdownTask = Task { @MainActor in
            for remaining in stride(from: seconds, through: 1, by: -1) {
                guard self.calibrationState == expectedState else { return }
                self.calibrationCountdownSeconds = remaining
                try? await Task.sleep(nanoseconds: 1_000_000_000)
                if Task.isCancelled { return }
            }
            self.calibrationCountdownSeconds = nil
            self.calibrationCountdownTotalSeconds = nil
            guard self.calibrationState == expectedState else { return }
            onComplete()
        }
    }

    func startCalibration() {
        // Full state reset first.
        resetIMUCalibration()

        print("IMU CALIBRATION (Two-Pose Gram-Schmidt)")
        print("POSE 1 - STANDING: all limbs hanging straight down")
        print("POSE 2 - SITTING with arms + shanks extended forward")

        updateCalibrationStatus(
            title: "Calibration: Pose 1",
            detail: "Stand still with both arms hanging down."
        )

        imuClient.sendMessage(to: rpiIP, message: "BEGIN_CALIBRATION")

        handManager.setAllLimbsActive(true)

        calibrationState = .waitingForPose1

        startCalibrationCountdown(seconds: Int(pose1HoldTime), expectedState: .waitingForPose1) {
            self.capturePose1()
        }
    }

    private func capturePose1() {
        guard calibrationState == .waitingForPose1 else { return }

        // Prefer the scene-loop-cached transform (refreshed every frame).
        // queryDeviceAnchor can sporadically return nil even when tracking is healthy,
        // which caused spurious "Headset tracking not ready" on re-calibration.
        let headsetTransform: simd_float4x4
        if let cached = lastHeadsetTransform {
            headsetTransform = cached
        } else if let deviceAnchor = worldTracking.queryDeviceAnchor(atTimestamp: CACurrentMediaTime()) {
            headsetTransform = deviceAnchor.originFromAnchorTransform
        } else {
            updateCalibrationStatus(
                title: "Calibration Failed",
                detail: "Headset tracking not ready. Please wait and try again."
            )
            calibrationState = .notCalibrated
            return
        }
        let headsetForward = SIMD3<Float>(
            -headsetTransform.columns.2.x, 0, -headsetTransform.columns.2.z
        )
        let headsetForwardNormalized = simd_normalize(headsetForward)
        calibrationHeadsetForwardAVP = headsetForwardNormalized
        calibrationHeadsetYaw = atan2(headsetForwardNormalized.x, -headsetForwardNormalized.z)
        calibrationHeadingQ = simd_quatf(angle: -calibrationHeadsetYaw, axis: SIMD3<Float>(0, 1, 0))
        handManager.calibrationHeadingQ = calibrationHeadingQ

        var capturedCount = 0
        for segment in activeSegments {
            if let rawIMU = lastIMUOrientations[segment] {
                pose1IMUData[segment] = rawIMU
                capturedCount += 1
            }
        }

        guard capturedCount > 0 else {
            updateCalibrationStatus(title: "Calibration Failed", detail: "Pose 1 capture failed. Check IMU connection.")
            calibrationState = .notCalibrated
            return
        }

        calibrationState = .pose1Captured

        updateCalibrationStatus(
            title: "Calibration: Pose 2",
            detail: "Sit with thighs, shanks, and arms extended forward."
        )

        calibrationState = .waitingForPose2

        startCalibrationCountdown(seconds: Int(pose2HoldTime), expectedState: .waitingForPose2) {
            self.capturePose2()
        }
    }

    private func capturePose2() {
        guard calibrationState == .waitingForPose2 else { return }

        updateCalibrationStatus(title: "Calibration", detail: "Capturing pose 2 and computing calibration...")

        var capturedCount = 0
        for segment in activeSegments {
            if let rawIMU = lastIMUOrientations[segment] {
                pose2IMUData[segment] = rawIMU
                capturedCount += 1
            }
        }

        guard capturedCount > 0 else {
            updateCalibrationStatus(title: "Calibration Failed", detail: "Pose 2 capture failed.")
            calibrationState = .pose1Captured
            return
        }

        computeCalibration()
    }

    /// Computes per-sensor leftFix + rightFix from the two captured poses.
    /// Limb segments: leftFix via Gram-Schmidt (pose1/pose2 world-frame basis → canonical AVP).
    /// rightFix = preRotation · axialFix (cylinder axis → sensor axis, then axial residual).
    /// Chest: leftFix from detected gravity + forward axes; rightFix = identity.
    private func computeCalibration() {
        var calibratedCount = 0

        let limbSegmentList = activeSegments.filter { $0 != chestSegment }

        let yawQ = simd_quatf(angle: -calibrationHeadsetYaw, axis: SIMD3<Float>(0, 1, 0))
        let target_down_p1   = rotateVector(SIMD3<Float>(0, -1, 0), by: yawQ)
        let target_fwd_p1    = rotateVector(SIMD3<Float>(0,  0, -1), by: yawQ)
        let target_normal_p1 = simd_normalize(simd_cross(target_down_p1, target_fwd_p1))
        let target_up_p2     = SIMD3<Float>(0, 1, 0)

        // Auto-detect limb-down axis for every limb segment (arms + legs).
        for segment in limbSegmentList {
            if solveLimbCorrection(segment: segment,
                                   useAutoDetectedLimbAxis: true,
                                   target_down: target_down_p1,
                                   target_fwd:  target_fwd_p1,
                                   target_normal: target_normal_p1,
                                   target_up_pose2: target_up_p2) {
                calibratedCount += 1
            }
        }

        // Chest (single-pose, gravity + headset forward).
        if let chestPose1 = pose1IMUData[chestSegment] {
            let gravityLocal = detectLimbDownAxis(pose: chestPose1)
            let forwardLocal = detectChestForwardAxis(pose: chestPose1, headsetForward: calibrationHeadsetForwardAVP, excludeAxis: gravityLocal)
            chestForwardLocalAxis = forwardLocal

            let e_down = simd_normalize(rotateVector(gravityLocal, by: chestPose1))
            let fwdRaw = rotateVector(forwardLocal, by: chestPose1)
            let fwdProj = simd_dot(fwdRaw, e_down) * e_down
            let e_fwd = simd_normalize(fwdRaw - fwdProj)
            let e_normal = simd_normalize(simd_cross(e_down, e_fwd))

            let M_src = simd_float3x3(columns: (e_normal, e_down, e_fwd))
            let M_tgt = simd_float3x3(columns: (target_normal_p1, target_down_p1, target_fwd_p1))
            let R = M_tgt * M_src.transpose

            imuLeftFix[chestSegment]  = simd_quatf(R)
            imuRightFix[chestSegment] = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)
            calibratedCount += 1
        }

        guard calibratedCount > 0 else {
            updateCalibrationStatus(title: "Calibration Failed", detail: "Calibration solve failed. Please re-calibrate.")
            calibrationState = .notCalibrated
            return
        }

        calibrationState = .calibrated
        print("CALIBRATION COMPLETE (\(calibratedCount)/\(activeSegments.count) devices)")

        // Chest always streams — it provides the body reference frame.
        sendIMUCommand(segment: chestSegment, command: "START")
        imuStreamingSegments.insert(chestSegment)
        Task {
            try? await Task.sleep(for: .milliseconds(200))
            await MainActor.run {
                if let nodeID = segmentToNodeID[chestSegment] {
                    self.lastSentCommand.removeValue(forKey: "\(nodeID)_imu")
                }
                self.sendIMUCommand(segment: chestSegment, command: "START")
            }
        }

        // Reset motor state. Proximity tracking is independent of calibration.
        for seg in activeSegments {
            motorCurrentLevel[seg] = nil
            motorIsOn[seg] = false
        }
        for (_, timer) in motorOffDebounceTimers { timer.invalidate() }
        motorOffDebounceTimers.removeAll()

        // Show all skeletons after calibration and start all limb IMUs.
        handManager.setAllLimbsActive(true)
        for seg in activeSegments where seg != chestSegment {
            if let nodeID = segmentToNodeID[seg] {
                lastSentCommand.removeValue(forKey: "\(nodeID)_imu")
            }
        }
        startAllLimbIMUs()

        isMotorEnabled = false

        calibrationCountdownTask?.cancel()
        calibrationCountdownTask = nil
        calibrationCountdownSeconds = nil
        calibrationCountdownTotalSeconds = nil
        updateCalibrationStatus(
            title: "Calibration Complete",
            detail: "Motors remain disabled. Use Enable Motors when ready."
        )
    }

    func resetIMUCalibration(triggerAutoRecalibration: Bool = false) {
        calibrationCountdownTask?.cancel()
        calibrationCountdownTask = nil
        calibrationCountdownSeconds = nil
        calibrationCountdownTotalSeconds = nil
        calibrationState = .notCalibrated
        imuLeftFix.removeAll()
        imuRightFix.removeAll()
        imuLimbDownAxis.removeAll()
        pose1IMUData.removeAll()
        pose2IMUData.removeAll()
        rawBeforeStop.removeAll()
        pendingYawResync.removeAll()
        lastDispatchedRaw.removeAll()
        handManager.resetParentOrientations()
        hasChestYawTarget = false
        chestYawTarget = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)
        chestYawDisplay = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)
        hasChestYawReference = false
        chestYawReference = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)

        handManager.chestYawDelta = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)
        handManager.chestYawDeltaInverse = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)
        calibrationHeadingQ = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)
        handManager.calibrationHeadingQ = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)
        chestForwardLocalAxis = SIMD3<Float>(0, 0, -1)

        lastSentCommand.removeAll()

        for segment in activeSegments where segment != chestSegment {
            sendIMUCommand(segment: segment, command: "STOP")
        }
        imuStreamingSegments.removeAll()

        for seg in activeSegments {
            if motorIsOn[seg] ?? false {
                sendMotorCommand(segment: seg, on: false)
            }
            motorIsOn[seg] = false
            motorCurrentLevel[seg] = nil
        }
        for (_, timer) in motorOffDebounceTimers { timer.invalidate() }
        motorOffDebounceTimers.removeAll()

        trackedObstacles.removeAll()

        isMotorEnabled = false

        if triggerAutoRecalibration {
            autoCalibrationScheduled = false
            updateCalibrationStatus(title: "Calibration Reset", detail: "Waiting for IMU data to auto-start calibration.")
        } else {
            updateCalibrationStatus(title: "Not Calibrated", detail: "Press Re-Calibrate to begin two-pose calibration.")
        }
    }

    /// q_calibrated = leftFix · raw · rightFix
    private func applyIMUCalibration(segment: String, raw: simd_quatf) -> simd_quatf {
        guard isIMUCalibrated,
              let leftFix  = imuLeftFix[segment],
              let rightFix = imuRightFix[segment] else {
            return raw
        }
        return leftFix * raw * rightFix
    }

    /// Solves leftFix + rightFix for one limb segment from pose1 and pose2 IMU data.
    @discardableResult
    private func solveLimbCorrection(
        segment: String,
        useAutoDetectedLimbAxis: Bool,
        target_down: SIMD3<Float>,
        target_fwd: SIMD3<Float>,
        target_normal: SIMD3<Float>,
        target_up_pose2: SIMD3<Float>
    ) -> Bool {
        guard let pose1 = pose1IMUData[segment],
              let pose2 = pose2IMUData[segment] else {
            return false
        }

        // 1. Body-frame limb-down axis.
        let limbAxisBody: SIMD3<Float>
        if useAutoDetectedLimbAxis {
            // BNO086 reports ENU; ENU down = (0,0,-1). Express it in sensor body frame.
            let enuDown = SIMD3<Float>(0, 0, -1)
            limbAxisBody = simd_normalize(rotateVector(enuDown, by: pose1.inverse))
        } else {
            limbAxisBody = SIMD3<Float>(0, -1, 0)
        }
        imuLimbDownAxis[segment] = limbAxisBody

        // 2. Gram-Schmidt source basis in world (ENU).
        let limbDown1 = rotateVector(limbAxisBody, by: pose1)
        let e_down = simd_normalize(limbDown1)
        let limbDown2 = rotateVector(limbAxisBody, by: pose2)
        let proj = simd_dot(limbDown2, e_down) * e_down
        let swingRaw = limbDown2 - proj
        guard simd_length(swingRaw) > 0.01 else {
            return false
        }
        let e_swing  = simd_normalize(swingRaw)
        let e_normal = simd_normalize(simd_cross(e_down, e_swing))

        let M_src = simd_float3x3(columns: (e_normal, e_down, e_swing))
        let M_tgt = simd_float3x3(columns: (target_normal, target_down, target_fwd))
        let R = M_tgt * M_src.transpose
        let leftFix = simd_quatf(R)

        // 3. preRotation maps cylinder-local (0,-1,0) onto the sensor's physical limb axis.
        let preRotation = shortestRotation(from: SIMD3<Float>(0, -1, 0), to: limbAxisBody)

        // 4. Axial-DOF correction: at pose 2 the limb is horizontal; compare cylinder +Z
        // world direction against target_up_pose2 (+Y), measured in the plane ⟂ to limb.
        let calibrated_p2_pre = leftFix * pose2 * preRotation
        let observed_top = rotateVector(SIMD3<Float>(0, 0, 1), by: calibrated_p2_pre)
        let limbDir_p2   = rotateVector(SIMD3<Float>(0, -1, 0), by: calibrated_p2_pre)
        let axialAngle = signedAngleAround(axis: limbDir_p2,
                                           from: observed_top,
                                           to:   target_up_pose2)
        let axialFix = simd_quatf(angle: axialAngle, axis: SIMD3<Float>(0, -1, 0))
        let rightFix = preRotation * axialFix

        imuLeftFix[segment]  = leftFix
        imuRightFix[segment] = rightFix
        return true
    }

    /// Signed angle from `a` to `b` measured around `axis` (right-hand rule).
    /// `a` and `b` are projected onto the plane perpendicular to `axis`.
    private func signedAngleAround(axis: SIMD3<Float>, from a: SIMD3<Float>, to b: SIMD3<Float>) -> Float {
        let n = simd_normalize(axis)
        let aPerp = a - n * simd_dot(a, n)
        let bPerp = b - n * simd_dot(b, n)
        if simd_length(aPerp) < 1e-4 || simd_length(bPerp) < 1e-4 { return 0 }
        let aN = simd_normalize(aPerp)
        let bN = simd_normalize(bPerp)
        let s = simd_dot(simd_cross(aN, bN), n)
        let c = simd_dot(aN, bN)
        return atan2(s, c)
    }

    // MARK: - Chest Yaw Tracking

    /// Extracts chest yaw as a pure twist around AVP world +Y.
    /// Avoids coupling to chest local forward-axis auto-detection and prevents
    /// roll/pitch (or wrong-axis twist) from leaking into the heading estimate.
    private func updateChestYaw(calibrated: simd_quatf) {
        let currentYawOnly = extractYawOnlyWorldY(from: calibrated)

        // Anchor to the FIRST post-calibration reading so chestYawDelta starts at
        // identity — the skeleton's heading matches calibrationHeadingQ at T=0.
        if !hasChestYawReference {
            chestYawReference = currentYawOnly
            hasChestYawReference = true
        }

        let target = chestYawReference.inverse * currentYawOnly
        chestYawTarget = target

        if !hasChestYawTarget {
            chestYawDisplay = target
            hasChestYawTarget = true
            handManager.chestYawDelta = target
            handManager.chestYawDeltaInverse = target.inverse
        }
    }

    /// Returns the yaw-only component of `q` as a twist around AVP world +Y
    /// via swing-twist decomposition.
    private func extractYawOnlyWorldY(from q: simd_quatf) -> simd_quatf {
        let qn = q.normalized
        let axis = SIMD3<Float>(0, 1, 0)
        let v = SIMD3<Float>(qn.imag.x, qn.imag.y, qn.imag.z)
        let proj = axis * simd_dot(v, axis)
        let twist = simd_quatf(ix: proj.x, iy: proj.y, iz: proj.z, r: qn.real)

        let mag2 = twist.real * twist.real
            + twist.imag.x * twist.imag.x
            + twist.imag.y * twist.imag.y
            + twist.imag.z * twist.imag.z
        guard mag2 > 1e-8 else {
            return simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)
        }
        return twist.normalized
    }

    // MARK: - Calibration Helpers

    private func rotateVector(_ v: SIMD3<Float>, by q: simd_quatf) -> SIMD3<Float> {
        let qv = simd_quatf(ix: v.x, iy: v.y, iz: v.z, r: 0)
        let result = q * qv * q.inverse
        return SIMD3<Float>(result.imag.x, result.imag.y, result.imag.z)
    }

    private func shortestRotation(from: SIMD3<Float>, to: SIMD3<Float>) -> simd_quatf {
        let d = simd_dot(from, to)
        if d > 0.9999 { return simd_quatf(ix: 0, iy: 0, iz: 0, r: 1) }
        if d < -0.9999 {
            var perp = simd_cross(from, SIMD3<Float>(1, 0, 0))
            if simd_length(perp) < 0.01 { perp = simd_cross(from, SIMD3<Float>(0, 0, 1)) }
            perp = simd_normalize(perp)
            return simd_quatf(angle: .pi, axis: perp)
        }
        let axis = simd_normalize(simd_cross(from, to))
        let angle = acos(max(min(d, 1.0), -1.0))
        return simd_quatf(angle: angle, axis: axis)
    }

    private func detectChestForwardAxis(pose: simd_quatf, headsetForward: SIMD3<Float>, excludeAxis: SIMD3<Float>? = nil) -> SIMD3<Float> {
        let candidateAxes: [SIMD3<Float>] = [
            SIMD3<Float>( 1, 0, 0), SIMD3<Float>(-1, 0, 0),
            SIMD3<Float>( 0, 1, 0), SIMD3<Float>( 0,-1, 0),
            SIMD3<Float>( 0, 0, 1), SIMD3<Float>( 0, 0,-1),
        ]
        let enuForward = simd_normalize(SIMD3<Float>(headsetForward.x, -headsetForward.z, 0))

        var bestAxis = SIMD3<Float>(0, 1, 0)
        var bestDot: Float = -2.0
        for candidate in candidateAxes {
            if let exclude = excludeAxis, abs(simd_dot(candidate, exclude)) > 0.9 { continue }
            let worldDir = rotateVector(candidate, by: pose)
            let horizontal = SIMD3<Float>(worldDir.x, worldDir.y, 0)
            guard simd_length(horizontal) > 0.1 else { continue }
            let d = simd_dot(simd_normalize(horizontal), enuForward)
            if d > bestDot {
                bestDot = d
                bestAxis = candidate
            }
        }
        return bestAxis
    }

    private func detectLimbDownAxis(pose: simd_quatf) -> SIMD3<Float> {
        let candidateAxes: [SIMD3<Float>] = [
            SIMD3<Float>( 1, 0, 0), SIMD3<Float>(-1, 0, 0),
            SIMD3<Float>( 0, 1, 0), SIMD3<Float>( 0,-1, 0),
            SIMD3<Float>( 0, 0, 1), SIMD3<Float>( 0, 0,-1),
        ]
        // ENU down is (0,0,-1).
        let enuDown = SIMD3<Float>(0, 0, -1)

        var bestAxis = SIMD3<Float>(0, -1, 0)
        var bestDot: Float = -2.0
        for candidate in candidateAxes {
            let worldDir = rotateVector(candidate, by: pose)
            let d = simd_dot(worldDir, enuDown)
            if d > bestDot {
                bestDot = d
                bestAxis = candidate
            }
        }
        return bestAxis
    }

    // MARK: - Yaw-Resync on STOP->START

    /// Snapshots each currently-streaming limb's latest raw quaternion before a STOP.
    private func snapshotRawForYawResync() {
        rawBeforeStop.removeAll()
        pendingYawResync.removeAll()
        for seg in imuStreamingSegments where seg != chestSegment && imuLeftFix[seg] != nil {
            if let raw = lastIMUOrientations[seg] {
                rawBeforeStop[seg] = raw
                pendingYawResync.insert(seg)
            }
        }
    }

    /// Projects a quaternion onto a specified axis and returns the twist component.
    private func extractYawOnly(from q: simd_quatf, axis: SIMD3<Float>) -> simd_quatf {
        let qn = q.normalized
        let n  = simd_normalize(axis)
        let v  = SIMD3<Float>(qn.imag.x, qn.imag.y, qn.imag.z)
        let proj = n * simd_dot(v, n)
        let twist = simd_quatf(ix: proj.x, iy: proj.y, iz: proj.z, r: qn.real)
        let mag2 = twist.real * twist.real
            + twist.imag.x * twist.imag.x
            + twist.imag.y * twist.imag.y
            + twist.imag.z * twist.imag.z
        guard mag2 > 1e-8 else {
            return simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)
        }
        return twist.normalized
    }

    /// First post-restart packet: compare yaw to the pre-stop snapshot and fold
    /// the inverse drift into leftFix so AVP-frame output stays aligned.
    private func applyYawResyncIfNeeded(segment: String, currentRaw: simd_quatf) {
        guard pendingYawResync.contains(segment) else { return }

        guard let rawOld = rawBeforeStop[segment] else {
            pendingYawResync.remove(segment)
            return
        }
        guard let leftFix = imuLeftFix[segment] else {
            pendingYawResync.remove(segment)
            rawBeforeStop.removeValue(forKey: segment)
            return
        }

        let driftFull = currentRaw * rawOld.inverse
        let enuVertical = SIMD3<Float>(0, 0, 1)
        let driftYawENU = extractYawOnly(from: driftFull, axis: enuVertical)

        imuLeftFix[segment] = leftFix * driftYawENU.inverse

        pendingYawResync.remove(segment)
        rawBeforeStop.removeValue(forKey: segment)
    }

    private func logSegmentIfDue(segment: String, raw: simd_quatf, calibrated: simd_quatf) {
        let now = CACurrentMediaTime()
        let last = lastQuatLogTime[segment] ?? 0
        if now - last < 1.0 { return }
        lastQuatLogTime[segment] = now
    }

    // MARK: - Proximity Trigger

    /// Rebuilds the proximity sphere collision shape when radius changes.
    private func refreshProximityTrigger() {
        guard let trigger = proximityTrigger else { return }
        let shape = ShapeResource.generateSphere(radius: proximityTriggerRadius)
        var collision = CollisionComponent(shapes: [shape])
        collision.filter = CollisionFilter(group: .skeleton, mask: .obstacle)
        trigger.components.set(collision)
    }

    /// Rebuilds the shoulder-width body sphere collision shape when
    /// `bodyCollisionRadius` changes.
    private func refreshBodyCollisionTrigger() {
        guard let trigger = bodyCollisionTrigger else { return }
        let shape = ShapeResource.generateSphere(radius: bodyCollisionRadius)
        var collision = CollisionComponent(shapes: [shape])
        collision.filter = CollisionFilter(group: .skeleton, mask: .obstacle)
        trigger.components.set(collision)
    }

    // MARK: - Collision Handling

    private func handleCollisionBegan(_ event: CollisionEvents.Began) {
        print("[Collision] BEGAN a=\(event.entityA.name) b=\(event.entityB.name)")
        guard let pair = classifyCollision(a: event.entityA, b: event.entityB) else { return }
        switch pair.trigger {
        case .proximity:
            trackedObstacles.insert(pair.obstacle)
        case .body:
            // Direct physical contact. Trigger-volume overlap is the source
            // of truth — no limb-distance threshold check required.
            print("[Collision] BODY contact with \(pair.obstacle.name)")
            registerBodyContact(with: pair.obstacle)
        }
    }

    private func handleCollisionEnded(_ event: CollisionEvents.Ended) {
        guard let pair = classifyCollision(a: event.entityA, b: event.entityB) else { return }
        switch pair.trigger {
        case .proximity:
            trackedObstacles.remove(pair.obstacle)
            // Motor state for this obstacle is re-evaluated on the next proximity tick.
        case .body:
            break
        }
    }

    /// Fires COLLIDED or VICTORY based on the obstacle's name. Called from the
    /// body trigger's Began event, so by definition the user's silhouette
    /// overlaps the obstacle's collision shape at this moment.
    private func registerBodyContact(with obstacle: Entity) {
        let now = CACurrentMediaTime()
        if obstacle.name.contains("Victory") {
            lastVictoryTime = now
            recordVictory()
        } else if obstacle.name.hasPrefix("CarCube") {
            lastCollisionTime = now
            recordCarContact(obstacle.name)
        } else {
            // Any other obstacle (e.g. stationary test pillar) still counts
            // as a generic collision for the banner, but isn't tallied as a car hit.
            lastCollisionTime = now
        }
    }

    private enum TriggerKind { case proximity, body }

    /// Classifies a collision pair. Returns the obstacle entity along with
    /// which of our two headset-anchored triggers it overlapped, or nil if
    /// neither entity is one of our triggers (e.g. obstacle-vs-obstacle).
    private func classifyCollision(a: Entity, b: Entity) -> (obstacle: Entity, trigger: TriggerKind)? {
        if a.name == "bodyCollisionTrigger"      { return (b, .body) }
        if b.name == "bodyCollisionTrigger"      { return (a, .body) }
        if a.name == "headsetProximityTrigger"   { return (b, .proximity) }
        if b.name == "headsetProximityTrigger"   { return (a, .proximity) }
        return nil
    }

    // MARK: - Proximity-Based Motor Update

    /// Called per frame from the scene update loop. Two-pass algorithm:
    ///   Pass 1 — for each tracked obstacle, find the single closest *eligible*
    ///   limb (`motorEligibleLimbs`) and assign the obstacle to it.
    ///   Pass 2 — for each limb, take the minimum distance across the obstacles
    ///   assigned to it, quantize, drive the motor, update the visualizer.
    /// Non-eligible limbs and eligible limbs that won no obstacle are forced
    /// to OFF (and their visualizers hidden), preventing simultaneous vibration
    /// across multiple limbs from a single nearby object.
    private func updateMotorsByProximity() {
        guard isIMUCalibrated else { return }

        let midpoints = handManager.limbMidpoints()

        // Pass 1: per-obstacle, pick the single closest eligible limb.
        // limbAssignments[limb] holds the smallest (distance, closestPoint)
        // among all obstacles whose closest eligible limb is `limb`.
        var limbAssignments: [HandTrackingManager.IMUBodySegment: (dist: Float, closest: SIMD3<Float>)] = [:]

        // Contact (COLLIDED / VICTORY) is handled event-driven by the
        // shoulder-width body trigger — see `registerBodyContact`. This loop
        // is now purely motor/visualizer feedback against the larger
        // proximity sphere.
        for obstacle in trackedObstacles {
            let box = obstacle.visualBounds(relativeTo: nil)
            let isVictory = obstacle.name.contains("Victory")

            // Victory entities never buzz; skip them for motor feedback.
            guard !isVictory else { continue }

            var winner: HandTrackingManager.IMUBodySegment?
            var winnerDist: Float = .infinity
            var winnerClosest: SIMD3<Float> = .zero

            for (imuSeg, limbPos) in midpoints {
                guard motorEligibleLimbs.contains(imuSeg) else { continue }
                let closest = clampPointToAABB(limbPos, boxMin: box.min, boxMax: box.max)
                let d = simd_length(closest - limbPos)
                if d < winnerDist {
                    winnerDist = d
                    winnerClosest = closest
                    winner = imuSeg
                }
            }

            guard let winningLimb = winner else { continue }
            // Keep the dominant (closest) obstacle for this limb across the frame.
            if let existing = limbAssignments[winningLimb], existing.dist <= winnerDist { continue }
            limbAssignments[winningLimb] = (winnerDist, winnerClosest)
        }

        // Pass 2: drive every limb. Non-eligible and unassigned limbs go to OFF.
        for (imuSeg, limbPos) in midpoints {
            if let assignment = limbAssignments[imuSeg] {
                let level = quantizeDistance(assignment.dist)
                applyLevel(segment: imuSeg.rawValue, newLevel: level)
                updateProximityVisualizer(imuSeg: imuSeg, limbPos: limbPos, closestPoint: assignment.closest, level: level)
            } else {
                applyLevel(segment: imuSeg.rawValue, newLevel: nil)
                updateProximityVisualizer(imuSeg: imuSeg, limbPos: limbPos, closestPoint: limbPos, level: nil)
            }
        }
    }

    /// Positions, colors, and orients the per-limb visualizer marker + connector.
    /// Hidden when the limb is out of range (level == nil) or the toggle is off.
    private func updateProximityVisualizer(
        imuSeg: HandTrackingManager.IMUBodySegment,
        limbPos: SIMD3<Float>,
        closestPoint: SIMD3<Float>,
        level: String?
    ) {
        guard let marker = closestPointMarkers[imuSeg],
              let connector = distanceConnectors[imuSeg] else { return }

        guard showProximityVisualizers, let lvl = level else {
            marker.isEnabled = false
            connector.isEnabled = false
            return
        }

        let color: UIColor
        switch lvl {
        case "close": color = .systemRed
        case "med":   color = .systemYellow
        default:      color = .systemBlue   // far
        }
        setVisualizerColor(marker, color: color)
        setVisualizerColor(connector, color: color)

        marker.position = closestPoint
        marker.isEnabled = true

        // Scale + orient the connector to span from limbPos to closestPoint.
        let dir = closestPoint - limbPos
        let length = simd_length(dir)
        guard length > 1e-4 else {
            connector.isEnabled = false
            return
        }
        connector.position = (limbPos + closestPoint) * 0.5
        let up = SIMD3<Float>(0, 1, 0)
        connector.orientation = shortestRotation(from: up, to: dir / length)
        connector.scale = SIMD3<Float>(1, length / connectorBaseHeight, 1)
        connector.isEnabled = true
    }

    private func setVisualizerColor(_ entity: ModelEntity, color: UIColor) {
        guard var mc = entity.components[ModelComponent.self] else { return }
        mc.materials = [UnlitMaterial(color: color)]
        entity.components.set(mc)
    }

    /// Clamps `p` into the AABB defined by `[boxMin, boxMax]`. Returns the closest
    /// point on (or inside) the box. For query points outside, this is the surface
    /// point; for points inside, this returns the query point itself (distance 0).
    private func clampPointToAABB(_ p: SIMD3<Float>, boxMin: SIMD3<Float>, boxMax: SIMD3<Float>) -> SIMD3<Float> {
        return SIMD3<Float>(
            min(max(p.x, boxMin.x), boxMax.x),
            min(max(p.y, boxMin.y), boxMax.y),
            min(max(p.z, boxMin.z), boxMax.z)
        )
    }

    /// Quantizes a surface distance to a motor level. nil = OFF.
    private func quantizeDistance(_ d: Float) -> String? {
        if d < distCloseMax { return "close" }
        if d < distMedMax   { return "med" }
        if d <= distFarMax  { return "far" }
        return nil
    }

    /// Updates motor output for a segment to match `newLevel`, deduping against
    /// `motorCurrentLevel` to avoid redundant UDP sends. When motors are disabled,
    /// ON transitions are skipped (state stays at previous) so they fire correctly
    /// once the user re-enables motors. OFF always goes through.
    private func applyLevel(segment: String, newLevel: String?) {
        let previous = motorCurrentLevel[segment] ?? nil
        guard newLevel != previous else { return }

        if let level = newLevel {
            guard isMotorEnabled else { return }
            let shellIndex: Int
            switch level {
            case "close": shellIndex = 0
            case "med":   shellIndex = 1
            default:      shellIndex = 2   // far
            }
            sendMotorCommand(segment: segment, on: true, shellIndex: shellIndex)
            motorIsOn[segment] = true
            motorCurrentLevel[segment] = level
        } else {
            sendMotorCommand(segment: segment, on: false)
            motorIsOn[segment] = false
            motorCurrentLevel[segment] = nil
        }
    }

    // MARK: - IMU Control

    private func startAllLimbIMUs() {
        guard !isIMUOverrideActive else { return }
        for segment in activeSegments where segment != chestSegment {
            if !imuStreamingSegments.contains(segment) {
                sendIMUCommand(segment: segment, command: "START")
                imuStreamingSegments.insert(segment)
            }
        }
    }

    private func stopAllLimbIMUs() {
        for segment in activeSegments where segment != chestSegment {
            if imuStreamingSegments.contains(segment) {
                sendIMUCommand(segment: segment, command: "STOP")
                imuStreamingSegments.remove(segment)
            }
        }
    }

    // MARK: - UDP Commands

    private func sendIMUCommand(segment: String, command: String) {
        guard let nodeID = segmentToNodeID[segment] else { return }
        let dedupKey = "\(nodeID)_imu"
        if lastSentCommand[dedupKey] == command { return }
        lastSentCommand[dedupKey] = command
        imuClient.sendMessage(to: rpiIP, message: "\(nodeID) \(command)")
    }

    private func sendMotorCommand(segment: String, on: Bool, shellIndex: Int? = nil) {
        guard let nodeID = segmentToNodeID[segment] else { return }
        guard isMotorEnabled || !on else { return }
        let label: String
        if !on {
            label = "OFF"
        } else if let idx = shellIndex {
            label = shellMotorLabels[min(idx, shellMotorLabels.count - 1)]
        } else {
            label = "CLOSE"
        }
        let dedupKey = "\(nodeID)_motor"
        if lastSentCommand[dedupKey] == label { return }
        lastSentCommand[dedupKey] = label
        imuClient.sendMessage(to: rpiIP, message: "\(nodeID) \(label)")
        print("Motor \(label) -> \(segment) (node \(nodeID))")
    }

    // MARK: - Data Providers

    var dataProvidersAreSupported: Bool {
        HandTrackingProvider.isSupported &&
        WorldTrackingProvider.isSupported
    }

    var isReadyToRun: Bool {
        handManager.handTracking.state == .initialized &&
        worldTracking.state == .initialized
    }

    // MARK: - Process Updates

    func processHandUpdates() async {
        await handManager.processHandUpdates()
    }

    func monitorSessionEvents() async {
        for await event in session.events {
            switch event {
            case .authorizationChanged(let type, let status):
                if type == .worldSensing && status != .allowed {
                    errorMessage = "World sensing authorization denied."
                }
            case .dataProviderStateChanged(_, _, let error):
                _ = error
            @unknown default:
                break
            }
        }
    }

    // MARK: - Manual Override Toggles

    func toggleMotorEnabled() {
        isMotorEnabled.toggle()
        print("Motors \(isMotorEnabled ? "ENABLED" : "DISABLED")")

        if !isMotorEnabled {
            for seg in activeSegments where motorIsOn[seg] ?? false {
                sendMotorCommand(segment: seg, on: false)
                motorIsOn[seg] = false
                motorCurrentLevel[seg] = nil
                motorOffDebounceTimers[seg]?.invalidate()
                motorOffDebounceTimers.removeValue(forKey: seg)
            }
        }
    }

    func toggleIMUOverride() {
        isIMUOverrideActive.toggle()
        print("IMU override \(isIMUOverrideActive ? "ON (all STOPPED)" : "OFF (re-syncing)")")

        if isIMUOverrideActive {
            // Snapshot first so yaw-resync can compensate when streams restart.
            snapshotRawForYawResync()
            for seg in activeSegments where imuStreamingSegments.contains(seg) {
                if seg == chestSegment { continue }
                sendIMUCommand(segment: seg, command: "STOP")
                imuStreamingSegments.remove(seg)
            }
        } else {
            if isIMUCalibrated {
                startAllLimbIMUs()
            }
        }
    }

    // MARK: - Cleanup

    deinit {
        orientationSubscription?.cancel()
        collisionBeganSubscription?.cancel()
        collisionEndedSubscription?.cancel()
        skeletonTrackingSubscription?.cancel()
        for (_, timer) in motorOffDebounceTimers { timer.invalidate() }
    }
}
