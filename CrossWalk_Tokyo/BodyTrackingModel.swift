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

    // Debug toggle: when false, the Tokyo crossing scene model and the
    // sky-blue skydome are hidden so the user can iterate on body tracking
    // / skeleton / haptics without the immersive environment in the way.
    // Default off for now — flip on once the rest of the system is solid.
    var showVREnvironment: Bool = true

    private var closestPointMarkers: [HandTrackingManager.IMUBodySegment: ModelEntity] = [:]
    private var distanceConnectors: [HandTrackingManager.IMUBodySegment: ModelEntity] = [:]
    private let connectorBaseHeight: Float = 1.0

    // Chest virtual-point visualizers. The position marker is the always-on
    // orange sphere showing where the back-centerline candidate sits; the
    // closest-point marker + connector behave like the limb pair but only
    // light up while chest is winning the per-obstacle proximity contest.
    private var chestPositionMarker: ModelEntity?
    private var chestClosestPointMarker: ModelEntity?
    private var chestConnector: ModelEntity?

    // Translucent flat sector showing the rear cone in which the chest
    // candidate is allowed to compete. Apex at the head; axis along
    // `backFlat`; half-angle `rearConeHalfDegrees`. Built lazily and
    // rebuilt only when the half-angle or radius changes.
    var showRearConeVisual: Bool = true
    private var rearConeVisual: ModelEntity?
    private var lastBuiltConeHalfAngle: Float = -1
    private var lastBuiltConeRadius: Float = -1

    // Distance buckets (meters). Distance is from a limb midpoint to the closest
    // point on an obstacle's AABB.
    //   CLOSE = [0, distCloseMax)
    //   MED   = [distCloseMax, distMedMax)
    //   FAR   = [distMedMax, distFarMax]
    //   OFF   > distFarMax
    var distCloseMax: Float = 0.15
    var distMedMax: Float = 0.20
    var distFarMax: Float = 0.40

    // Forward obstacle-course tunables. The user walks along -Z through a
    // corridor of randomly-placed rectangular-prism obstacles to reach a
    // VictoryGoal at the far end.
    //
    //   coursePathLength     — meters along -Z covered by the obstacle slots.
    //                          The VictoryGoal sits a bit past this.
    //   corridorHalfWidth    — meters; the playable corridor runs ±this.
    //   obstacleCount        — number of obstacles per run. Each one is a
    //                          uniform-random pick from the 4 type specs in
    //                          ImmersiveView (curb / trash can / signpost / wall).
    //   obstacleMinSpacing   — meters between consecutive obstacle Z slots.
    var coursePathLength: Float = 7.5    // user-to-goal distance, meters (auto-extended if obstacle layout needs more)
    var corridorHalfWidth: Float = 2.0
    var obstacleCount: Int = 5
    var obstacleMinSpacing: Float = 1.5  // fixed Z gap between consecutive obstacles
    // Seconds between Start being pressed and the obstacle course
    // populating. Lets a blindfolded subject press Start, close their
    // eyes, and have the course materialize after a known prep window.
    // The head-anchored elapsed-time readout is hidden during this window
    // and reset to 0 at the moment the course spawns.
    var courseStartGraceSec: Float = 3.0
    // Single-side testing aid. When true, every obstacle spawns to the
    // right of the corridor centerline (positive X) so a subject with
    // motors only on the right side can encounter every obstacle on the
    // instrumented limbs. Set false for normal alternating-side play.
    var spawnRightSideOnly: Bool = false

    // Set by ImmersiveView at Start so the on-victory stats line can render
    // "touched N / TOTAL". Held here (not in the view) so SwiftUI re-reads
    // it via @Observable.
    private(set) var totalObstaclesInCourse: Int = 0
    func setObstacleCourseTotal(_ n: Int) { totalObstaclesInCourse = n }

    // Haptic tie-tolerance. Per obstacle, every eligible limb whose
    // distance to that obstacle is within `limbSwitchMargin` of the closest
    // limb's distance fires (and has its visualizer drawn). So if both
    // shoulders are roughly equidistant to a frontal wall, both vibrate;
    // if the user is clearly leaning to one side, only that side fires.
    // Sized just above the skeleton noise floor so single-limb reads stay
    // single-limb and real near-ties trigger together.
    var limbSwitchMargin: Float = 0.05

    // Virtual back-centerline haptic candidate. Computed from the headset
    // transform each frame so it tracks where the user is actually facing
    // (drift-free, unlike the chest IMU). When this point wins the per-
    // obstacle proximity contest, the motor segment "chest" (UDP node 3) is
    // driven instead of either shoulder. Disabled with `chestEligible = false`.
    //
    //   chestBackOffset      — meters behind the headset (along headset +Z).
    //                           Positive = further behind the body.
    //   chestVerticalOffset  — meters relative to head Y (negative = below).
    //                           ~-0.40 puts the point at chest level, matching
    //                           the body collision trigger.
    var chestEligible: Bool = true
    var chestBackOffset: Float = 0.18
    var chestVerticalOffset: Float = -0.40

    // Rear cone (half-angle, degrees) within which the chest candidate is
    // allowed to compete. Obstacles whose horizontal bearing relative to
    // headset-forward sits *outside* this cone (i.e. clearly to the side)
    // exclude the chest candidate, so the dodge-direction signal stays
    // unambiguous: chest = "behind, dodge either way", shoulder = "side,
    // dodge the other way". 0° disables chest entirely; 180° always
    // includes it (no gating).
    var rearConeHalfDegrees: Float = 5.0

    // Limbs that may receive vibrotactile feedback. Per obstacle, every
    // eligible limb within `limbSwitchMargin` of the closest limb's
    // distance fires — so genuinely-tied limbs all vibrate together.
    // Contact points come from `handManager.limbContactPoints()` which
    // uses the cylinder midpoint for upper arm + thigh and the distal
    // end (wrist / ankle) for forearm + shank.
    private let motorEligibleLimbs: Set<HandTrackingManager.IMUBodySegment> = [
        .leftUpperArm,  .rightUpperArm,
        .leftForearm,   .rightForearm,
        .leftThigh,     .rightThigh,
        .leftShank,     .rightShank
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
    // true only between those two events — during which time obstacles
    // exist, contacts are tallied, and the timer ticks.
    var runStartTime: CFTimeInterval? = nil
    var runEndTime: CFTimeInterval? = nil
    // Per-run obstacle entity names that registered a body-trigger contact.
    // ImmersiveView gives each obstacle a unique `_p<runIndex>` suffix so
    // the dedupe set never collides across runs.
    var obstaclesHitInstanceIDs: Set<String> = []

    var isRunActive: Bool {
        runStartTime != nil && runEndTime == nil
    }
    var runDuration: CFTimeInterval? {
        guard let s = runStartTime, let e = runEndTime else { return nil }
        return e - s
    }

    func startRun() {
        runStartTime = CACurrentMediaTime()
        runEndTime = nil
        obstaclesHitInstanceIDs.removeAll()
        lastCollisionTime = -.infinity
        lastVictoryTime = -.infinity
    }

    /// Cancels an in-progress run without recording a victory. Sets
    /// `runStartTime = nil` so `isRunActive` flips false; ImmersiveView's
    /// 10 Hz observer detects the transition and tears down the spawned
    /// obstacles + hides the victory goal. Stats are cleared so the
    /// post-run results panel doesn't linger from a half-finished run.
    func stopRun() {
        runStartTime = nil
        runEndTime = nil
        obstaclesHitInstanceIDs.removeAll()
        lastCollisionTime = -.infinity
        lastVictoryTime = -.infinity
    }

    func recordObstacleContact(_ instanceID: String) {
        guard isRunActive else { return }
        obstaclesHitInstanceIDs.insert(instanceID)
    }

    func recordVictory() {
        guard isRunActive else { return }
        runEndTime = CACurrentMediaTime()
    }

    private var imuStreamingSegments: Set<String> = []

    let shellMotorLabels: [String] = ["CLOSE", "MED", "FAR"]

    // MARK: - Manual Override Flags

    var isMotorEnabled: Bool = true
    var isIMUOverrideActive: Bool = false

    var isCalibrated: Bool { calibrationState == .calibrated }
    var isCalibrating: Bool { calibrationState != .notCalibrated && calibrationState != .calibrated }

    var calibrationStatusTitle: String = "Not Calibrated"
    var calibrationStatusDetail: String = "Press Calibrate when ready to begin two-pose calibration."

    // Per-segment timestamp of the most recent quaternion packet. The control
    // panel's live "Receiving IMU data" indicator filters this against
    // CACurrentMediaTime() to count segments that arrived in the last second
    // — gives an at-a-glance confirmation that sensors are streaming before
    // the user presses Calibrate.
    var lastSegmentPacketTime: [String: CFTimeInterval] = [:]
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
    var shoulderLateralOffset: Float = 0.30
    var hipVerticalOffset: Float = -0.70
    var hipLateralOffset: Float = 0.20

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
    var shankLength: Float = 0.26 { didSet { geometryNeedsRefresh = true } }
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
            detail: "Connect sensors. The status panel will confirm quaternion data is arriving — then press Calibrate when you're ready to start the two-pose routine."
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

        // Chest virtual-point visualizers. The orange sphere is always
        // visible (when `showProximityVisualizers && chestEligible`) so the
        // user can see where the back-centerline candidate sits relative to
        // their body. The closest-point marker + connector only light up
        // while chest is the winning candidate for some obstacle.
        let chestPos = ModelEntity(
            mesh: .generateSphere(radius: 0.06),
            materials: [UnlitMaterial(color: .systemOrange)]
        )
        chestPos.name = "chestPositionMarker"
        chestPos.isEnabled = false
        contentEntity.addChild(chestPos)
        chestPositionMarker = chestPos

        let chestClosest = ModelEntity(
            mesh: .generateSphere(radius: 0.04),
            materials: [UnlitMaterial(color: .white)]
        )
        chestClosest.name = "closestPointMarker_chest"
        chestClosest.isEnabled = false
        contentEntity.addChild(chestClosest)
        chestClosestPointMarker = chestClosest

        let chestConn = ModelEntity(
            mesh: .generateCylinder(height: connectorBaseHeight, radius: 0.006),
            materials: [UnlitMaterial(color: .white)]
        )
        chestConn.name = "distanceConnector_chest"
        chestConn.isEnabled = false
        contentEntity.addChild(chestConn)
        chestConnector = chestConn

        // Flat translucent sector showing the rear-cone gate. Mesh is built
        // lazily on the first proximity update so it can read the current
        // `rearConeHalfDegrees` / `proximityTriggerRadius`.
        let coneEntity = ModelEntity()
        coneEntity.name = "rearConeVisual"
        coneEntity.isEnabled = false
        contentEntity.addChild(coneEntity)
        rearConeVisual = coneEntity

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

        // If the user closed and reopened the immersive space mid-session,
        // the RPi may have dropped one or more IMU streams during the gap
        // (chest is the most-reported casualty). Re-kick every segment so
        // START actually goes back out — the sendIMUCommand dedupe is
        // cleared per-node so the resends aren't suppressed. Skipped if
        // the user is mid-calibration (the calibrate path manages its own
        // START sequencing).
        if isCalibrated {
            kickIMUStreamsAfterReattach()
        }
    }

    /// Tears down the scene-bound subscriptions so the next call to
    /// `attachToSceneIfReady()` re-subscribes against the new scene.
    /// Without this, closing + reopening the immersive space leaves the
    /// skeleton update loop and collision haptic events bound to the
    /// destroyed scene — IMU/UDP keeps flowing (its subscription is on
    /// imuClient.$orientations, not scene-bound), but the rendered
    /// skeleton freezes and proximity/contact haptics stop firing.
    func detachFromScene() {
        skeletonTrackingSubscription?.cancel()
        skeletonTrackingSubscription = nil
        collisionBeganSubscription?.cancel()
        collisionBeganSubscription = nil
        collisionEndedSubscription?.cancel()
        collisionEndedSubscription = nil
        hasAttachedToScene = false
    }

    /// Resends START to chest + every active limb after a reopen, clearing
    /// the per-node dedup so the commands actually leave the socket. The
    /// RPi treats START as idempotent — extra ones are harmless and they
    /// recover any stream the bridge dropped while the app was suspended.
    private func kickIMUStreamsAfterReattach() {
        if let chestNode = segmentToNodeID[chestSegment] {
            lastSentCommand.removeValue(forKey: "\(chestNode)_imu")
            sendIMUCommand(segment: chestSegment, command: "START")
            imuStreamingSegments.insert(chestSegment)
        }
        guard !isIMUOverrideActive else { return }
        for segment in activeSegments where segment != chestSegment {
            if let nodeID = segmentToNodeID[segment] {
                lastSentCommand.removeValue(forKey: "\(nodeID)_imu")
            }
            sendIMUCommand(segment: segment, command: "START")
            imuStreamingSegments.insert(segment)
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
                let nowTime = CACurrentMediaTime()
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
                        self.lastSegmentPacketTime[segName] = nowTime
                    }
                }

                // First-packet status nudge: tell the operator that quaternion
                // data is flowing and they can press Calibrate. We DO NOT
                // auto-start the two-pose routine — the user controls when
                // calibration begins so they can dwell in pose 1.
                if self.calibrationState == .notCalibrated,
                   self.calibrationStatusTitle != "IMUs Connected" {
                    self.calibrationStatusTitle = "IMUs Connected"
                    self.calibrationStatusDetail = "Quaternion data received. Press Calibrate when you're in pose 1 (standing, arms down, face and chest aligned)."
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
            detail: "Stand still with both arms hanging down. Face and chest must point the same direction (don't twist your torso relative to your head)."
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

        // Chest (single-pose, gravity + headset-derived forward).
        // Forward is taken from the AVP headset at pose 1 — the user's torso
        // and head must point the same way then (see calibration
        // instructions). Skipping the chest-IMU axis solve removes the
        // projection-of-torso-pitch-into-apparent-yaw failure mode.
        if let chestPose1 = pose1IMUData[chestSegment] {
            let gravityLocal = detectLimbDownAxis(pose: chestPose1)
            let e_down = simd_normalize(rotateVector(gravityLocal, by: chestPose1))

            let avpFwd = calibrationHeadsetForwardAVP
            let enuFwd = simd_normalize(SIMD3<Float>(avpFwd.x, -avpFwd.z, 0))
            // Re-project against measured gravity so e_fwd ⟂ e_down even if
            // the chest IMU's gravity reading isn't perfectly aligned with
            // ENU Z.
            let fwdProj = simd_dot(enuFwd, e_down) * e_down
            let e_fwd = simd_normalize(enuFwd - fwdProj)
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
            updateCalibrationStatus(title: "Calibration Reset", detail: "Press Calibrate when ready to begin two-pose calibration.")
        } else {
            updateCalibrationStatus(title: "Not Calibrated", detail: "Press Calibrate to begin two-pose calibration.")
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
        } else if obstacle.name.hasPrefix("Obstacle") {
            lastCollisionTime = now
            recordObstacleContact(obstacle.name)
        } else {
            // Any other entity (e.g. stationary debug pillar) still counts
            // as a generic collision for the banner, but isn't tallied.
            lastCollisionTime = now
        }
    }

    private enum TriggerKind { case proximity, body }

    /// Skeleton-cylinder names follow `<skeletonID>_<segment>` (see
    /// HandTrackingManager.buildArmLimb / buildLegLimb), e.g.
    /// "center_rightForearm". Used to recognize a cylinder-vs-obstacle
    /// collision so it gets routed to the body-contact handler.
    private static let limbCylinderSegmentSuffixes: [String] = [
        "_leftUpperArm", "_leftForearm", "_rightUpperArm", "_rightForearm",
        "_leftThigh",    "_leftShank",   "_rightThigh",    "_rightShank"
    ]
    private func isLimbCylinder(_ entity: Entity) -> Bool {
        let n = entity.name
        for suffix in Self.limbCylinderSegmentSuffixes where n.hasSuffix(suffix) {
            return true
        }
        return false
    }

    /// Classifies a collision pair. Returns the obstacle entity along with
    /// which of our two headset-anchored triggers (or a skeleton cylinder)
    /// overlapped it. Returns nil if neither entity is one of ours
    /// (e.g. obstacle-vs-obstacle).
    private func classifyCollision(a: Entity, b: Entity) -> (obstacle: Entity, trigger: TriggerKind)? {
        if a.name == "bodyCollisionTrigger"      { return (b, .body) }
        if b.name == "bodyCollisionTrigger"      { return (a, .body) }
        if a.name == "headsetProximityTrigger"   { return (b, .proximity) }
        if b.name == "headsetProximityTrigger"   { return (a, .proximity) }
        // Skeleton cylinders count as direct body contact: any limb
        // touching an obstacle should fire COLLIDED + tally the obstacle.
        if isLimbCylinder(a)                     { return (b, .body) }
        if isLimbCylinder(b)                     { return (a, .body) }
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

        let midpoints = handManager.limbContactPoints()

        // Always-on candidates (eligible IMU limbs). The chest virtual point
        // is held separately so it can be gated per-obstacle by the rear
        // cone — this keeps the dodge-direction signal unambiguous: chest
        // = "behind, dodge either way", shoulder = "side, dodge the other
        // way".
        typealias HapticCandidate = (motorSeg: String, position: SIMD3<Float>, imuSeg: HandTrackingManager.IMUBodySegment?)
        var baseCandidates: [HapticCandidate] = []
        for (imuSeg, pos) in midpoints where motorEligibleLimbs.contains(imuSeg) {
            baseCandidates.append((imuSeg.rawValue, pos, imuSeg))
        }

        // Compute the chest virtual-point position + rear-cone basis (for
        // per-obstacle gating). World-space, derived from the headset
        // transform so it tracks where the user is actually facing.
        var chestCandidate: HapticCandidate? = nil
        var headPosFlat: SIMD3<Float> = .zero
        var backFlat: SIMD3<Float> = SIMD3<Float>(0, 0, 1)
        var rearConeCos: Float = -1
        if chestEligible, let head = lastHeadsetTransform {
            let headPos = SIMD3<Float>(head.columns.3.x, head.columns.3.y, head.columns.3.z)
            let backRaw = SIMD3<Float>(head.columns.2.x, head.columns.2.y, head.columns.2.z)
            // Project to horizontal so vertical placement is decoupled from
            // head pitch (looking up/down shouldn't move the chest point).
            var bf = SIMD3<Float>(backRaw.x, 0, backRaw.z)
            let mag = simd_length(bf)
            if mag > 1e-4 { bf /= mag } else { bf = SIMD3<Float>(0, 0, 1) }
            backFlat = bf
            headPosFlat = SIMD3<Float>(headPos.x, 0, headPos.z)
            let chestPos = headPos
                + backFlat * chestBackOffset
                + SIMD3<Float>(0, chestVerticalOffset, 0)
            chestCandidate = (chestSegment, chestPos, nil)
            let halfRad = max(rearConeHalfDegrees, 0) * .pi / 180
            rearConeCos = cos(halfRad)
        }

        // Always-on chest position marker. Visible whenever chest is eligible
        // and visualizers are on, regardless of whether chest is winning any
        // obstacle this frame.
        if let posMarker = chestPositionMarker {
            if let cc = chestCandidate, showProximityVisualizers {
                posMarker.position = cc.position
                if !posMarker.isEnabled { posMarker.isEnabled = true }
            } else if posMarker.isEnabled {
                posMarker.isEnabled = false
            }
        }

        // Rear-cone debug sector. Only built/positioned when chest is
        // active; ride along with `showRearConeVisual` toggle. Mesh
        // rebuilds whenever the half-angle or proximity radius change.
        updateRearConeVisual(active: chestCandidate != nil,
                             headPosFlat: headPosFlat,
                             backFlat: backFlat)

        // Pass 1: per-obstacle, every eligible candidate within
        // `limbSwitchMargin` of the closest candidate's distance gets
        // assigned the obstacle (and thus fires its motor + lights up its
        // visualizer in pass 2). This makes near-ties present on both
        // limbs simultaneously instead of going silent or flipping.
        var limbAssignments: [String: (dist: Float, closest: SIMD3<Float>)] = [:]
        let margin = max(limbSwitchMargin, 0)

        // Contact (COLLIDED / VICTORY) is handled event-driven by the
        // shoulder-width body trigger — see `registerBodyContact`. This loop
        // is now purely motor/visualizer feedback against the larger
        // proximity sphere.
        for obstacle in trackedObstacles {
            let box = obstacle.visualBounds(relativeTo: nil)
            let isVictory = obstacle.name.contains("Victory")

            // Victory entities never buzz; skip them for motor feedback.
            guard !isVictory else { continue }

            // Per-obstacle candidate set. Start from the always-on shoulders /
            // legs; include the chest virtual point only when the obstacle's
            // horizontal bearing is inside the rear cone (i.e. it's
            // genuinely behind the user, not to a side).
            var candidatesForThisObstacle = baseCandidates
            if let cc = chestCandidate {
                let obstacleCenter = obstacle.position(relativeTo: nil)
                var dir = SIMD3<Float>(obstacleCenter.x - headPosFlat.x, 0,
                                       obstacleCenter.z - headPosFlat.z)
                let dmag = simd_length(dir)
                let inRearCone: Bool
                if dmag < 1e-4 {
                    inRearCone = true
                } else {
                    dir /= dmag
                    inRearCone = simd_dot(dir, backFlat) >= rearConeCos
                }
                if inRearCone {
                    candidatesForThisObstacle.append(cc)
                }
            }

            // Score every candidate against this obstacle, then identify
            // the closest distance. Every candidate within `margin` of that
            // closest distance is considered tied and gets assigned.
            var perCand: [String: (dist: Float, closest: SIMD3<Float>)] = [:]
            var bestDist: Float = .infinity
            for cand in candidatesForThisObstacle {
                let closest = clampPointToAABB(cand.position, boxMin: box.min, boxMax: box.max)
                let d = simd_length(closest - cand.position)
                perCand[cand.motorSeg] = (d, closest)
                if d < bestDist { bestDist = d }
            }
            guard bestDist.isFinite else { continue }

            let tieCutoff = bestDist + margin
            for (motorSeg, entry) in perCand where entry.dist <= tieCutoff {
                // Per-limb, hold onto whichever obstacle is closest to it
                // across the frame (so the motor level reflects the worst
                // case, and the visualizer's connector points at it).
                if let existing = limbAssignments[motorSeg], existing.dist <= entry.dist { continue }
                limbAssignments[motorSeg] = (entry.dist, entry.closest)
            }
        }

        // Pass 2A: drive every always-on candidate (eligible IMU limbs).
        // Visualizers update for each via the IMU-keyed marker/connector.
        var candidateMotorSegs: Set<String> = Set(baseCandidates.map { $0.motorSeg })
        for cand in baseCandidates {
            if let assignment = limbAssignments[cand.motorSeg] {
                let level = quantizeDistance(assignment.dist)
                applyLevel(segment: cand.motorSeg, newLevel: level)
                if let imu = cand.imuSeg {
                    updateProximityVisualizer(imuSeg: imu, limbPos: cand.position, closestPoint: assignment.closest, level: level)
                }
            } else {
                applyLevel(segment: cand.motorSeg, newLevel: nil)
                if let imu = cand.imuSeg {
                    updateProximityVisualizer(imuSeg: imu, limbPos: cand.position, closestPoint: cand.position, level: nil)
                }
            }
        }

        // Pass 2B: drive the chest virtual candidate (motor + dedicated
        // closest-point/connector visualizer). Skipped entirely when chest
        // is disabled.
        if let cc = chestCandidate {
            candidateMotorSegs.insert(cc.motorSeg)
            if let assignment = limbAssignments[cc.motorSeg] {
                let level = quantizeDistance(assignment.dist)
                applyLevel(segment: cc.motorSeg, newLevel: level)
                updateProximityVisualizerEntities(
                    marker: chestClosestPointMarker,
                    connector: chestConnector,
                    limbPos: cc.position,
                    closestPoint: assignment.closest,
                    level: level
                )
            } else {
                applyLevel(segment: cc.motorSeg, newLevel: nil)
                updateProximityVisualizerEntities(
                    marker: chestClosestPointMarker,
                    connector: chestConnector,
                    limbPos: cc.position,
                    closestPoint: cc.position,
                    level: nil
                )
            }
        } else {
            // Chest disabled — make sure its visualizer pair is off.
            chestClosestPointMarker?.isEnabled = false
            chestConnector?.isEnabled = false
            applyLevel(segment: chestSegment, newLevel: nil)
        }

        // Pass 2C: every other IMU midpoint (forearms, legs, …) goes to OFF
        // along with its visualizer. Ineligible limbs that happen to share
        // a motor segment with a candidate (e.g. a future chest IMU stream)
        // are left alone here.
        for (imuSeg, limbPos) in midpoints where !candidateMotorSegs.contains(imuSeg.rawValue) {
            applyLevel(segment: imuSeg.rawValue, newLevel: nil)
            updateProximityVisualizer(imuSeg: imuSeg, limbPos: limbPos, closestPoint: limbPos, level: nil)
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
        updateProximityVisualizerEntities(
            marker: closestPointMarkers[imuSeg],
            connector: distanceConnectors[imuSeg],
            limbPos: limbPos,
            closestPoint: closestPoint,
            level: level
        )
    }

    /// Generalized variant that operates on raw entity refs. Used both by the
    /// per-limb path and by the chest virtual-point visualizer.
    private func updateProximityVisualizerEntities(
        marker: ModelEntity?,
        connector: ModelEntity?,
        limbPos: SIMD3<Float>,
        closestPoint: SIMD3<Float>,
        level: String?
    ) {
        guard let marker, let connector else { return }

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

    /// Builds (or rebuilds) the flat-sector mesh used to visualize the rear
    /// cone, then positions/orients it at chest level along the current
    /// horizontal back direction. Hidden when chest is disabled or the
    /// debug toggle is off. Mesh is rebuilt only when the half-angle or
    /// radius change, so steady-state cost is just a position + orientation
    /// write per frame.
    private func updateRearConeVisual(active: Bool,
                                      headPosFlat: SIMD3<Float>,
                                      backFlat: SIMD3<Float>) {
        guard let cone = rearConeVisual else { return }

        let visible = active && showRearConeVisual && showProximityVisualizers
        guard visible else {
            if cone.isEnabled { cone.isEnabled = false }
            return
        }

        let halfAngleDeg = max(rearConeHalfDegrees, 0.5)
        let radius = max(proximityTriggerRadius, 0.5)
        if abs(halfAngleDeg - lastBuiltConeHalfAngle) > 0.5
            || abs(radius - lastBuiltConeRadius) > 0.05
            || cone.model == nil {
            rebuildRearConeMesh(halfAngleDegrees: halfAngleDeg, radius: radius)
            lastBuiltConeHalfAngle = halfAngleDeg
            lastBuiltConeRadius = radius
        }

        // The mesh is built with apex at origin and axis along local +X (in
        // the XZ plane). At runtime we orient that local +X to `backFlat`
        // so the wedge opens horizontally behind the user, then drop the
        // entity to chest level so it sits in the same plane the cars do.
        guard let head = lastHeadsetTransform else { return }
        let headY = head.columns.3.y
        cone.position = SIMD3<Float>(headPosFlat.x,
                                     headY + chestVerticalOffset,
                                     headPosFlat.z)
        cone.orientation = shortestRotation(from: SIMD3<Float>(1, 0, 0), to: backFlat)
        if !cone.isEnabled { cone.isEnabled = true }
    }

    private func rebuildRearConeMesh(halfAngleDegrees: Float, radius: Float) {
        guard let cone = rearConeVisual else { return }

        let segments = 48
        let halfRad = halfAngleDegrees * .pi / 180
        var positions: [SIMD3<Float>] = []
        positions.reserveCapacity(segments + 2)
        positions.append(SIMD3<Float>(0, 0, 0))   // apex
        for i in 0...segments {
            let t = Float(i) / Float(segments)
            let a = -halfRad + (2 * halfRad) * t
            // Sweep around the apex axis (+X) in the XZ plane. Y stays 0
            // so the sector is flat; the entity is positioned at chest Y
            // by the caller.
            let x = cos(a) * radius
            let z = sin(a) * radius
            positions.append(SIMD3<Float>(x, 0, z))
        }
        var indices: [UInt32] = []
        indices.reserveCapacity(segments * 3)
        for i in 0..<segments {
            indices.append(0)
            indices.append(UInt32(i + 1))
            indices.append(UInt32(i + 2))
        }

        var desc = MeshDescriptor(name: "RearConeSector")
        desc.positions = MeshBuffers.Positions(positions)
        desc.primitives = .triangles(indices)
        do {
            let mesh = try MeshResource.generate(from: [desc])
            // Translucent green so it's visually distinct from the orange
            // chest sphere and the red/yellow/blue distance markers.
            let mat = UnlitMaterial(color: UIColor.systemGreen.withAlphaComponent(0.18))
            cone.model = ModelComponent(mesh: mesh, materials: [mat])
        } catch {
            // Mesh build failed; leave the entity without a model.
            cone.model = nil
        }
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
