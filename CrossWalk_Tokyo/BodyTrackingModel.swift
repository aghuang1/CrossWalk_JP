/*
 BodyTrackingModel.swift
 CrossWalk_Tokyo

 Central orchestrator for body limb tracking, IMU calibration, collision detection,
 and haptic motor control. Adapted from ExtendedTouch_AVP's EntityModel.swift.

 Removed: SceneReconstructionManager, PlaneDetectionManager dependencies.
 Modified: checkActivationByDistance uses obstacle entity proximity instead of mesh proximity.
*/

import ARKit
import RealityKit
import UIKit
import Combine
import Network

// MARK: - Node ID to Body Segment Mapping

// RPi relay address — all commands are sent here; RPi forwards to the correct node
let rpiIP = "192.168.1.7"

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
    // Pre-computed segment order for dead reckoning (parents first, then children)
    private static let orderedSegmentPairs: [(String, HandTrackingManager.IMUBodySegment)] = [
        ("leftUpperArm",  .leftUpperArm),  ("rightUpperArm", .rightUpperArm),
        ("leftThigh",     .leftThigh),     ("rightThigh",    .rightThigh),
        ("leftForearm",   .leftForearm),   ("rightForearm",  .rightForearm),
        ("leftShank",     .leftShank),     ("rightShank",    .rightShank),
    ]

    let session = ARKitSession()
    let worldTracking = WorldTrackingProvider()
    let handManager = HandTrackingManager()

    var errorMessage: String?
    private var contentEntity = Entity()
    private var hasSetupContentEntity = false
    private var hasAttachedToScene = false
    private var lastHeadsetTransform: simd_float4x4?

    // References to obstacle entities for distance-based activation
    var obstacleEntities: [Entity] = []

    // Combine subscriptions
    nonisolated(unsafe) private var orientationSubscription: AnyCancellable?
    nonisolated(unsafe) private var collisionBeganSubscription: (any Cancellable)?
    nonisolated(unsafe) private var collisionEndedSubscription: (any Cancellable)?
    nonisolated(unsafe) private var skeletonTrackingSubscription: (any Cancellable)?

    // MARK: - Collision State (per-motor collision tracking)

    private var motorCollisionCounts: [String: Int] = {
        var d: [String: Int] = [:]
        for seg in activeSegments { d[seg] = 0 }
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

    var areSkeletonsActive: Bool = false

    private var imuStreamingSegments: Set<String> = []

    private var activationFrameCounter: Int = 0
    private let activationRadius: Float = 1.5

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
    var shoulderLateralOffset: Float = 0.15
    var hipVerticalOffset: Float = -0.70
    var hipLateralOffset: Float = 0.10

    var skeletonRadius: Float = 1.0
    var skeletonAngles: [Float] = [0, -Float.pi/3, Float.pi/3]

    var upperArmLength: Float = 0.28
    var upperArmRadius: Float = 0.08
    var forearmLength: Float = 0.25
    var forearmRadius: Float = 0.08
    var thighLength: Float = 0.45
    var thighRadius: Float = 0.10
    var shankLength: Float = 0.17
    var shankRadius: Float = 0.10

    // MARK: - IMU Calibration (Two-Pose Gram-Schmidt)

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
    private var imuCalibrationOffsets: [String: simd_quatf] = [:]
    private var imuLimbDownAxis: [String: SIMD3<Float>] = [:]
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

    // Dead reckoning state
    private struct DeadReckoningState {
        var packetQuat: simd_quatf = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)
        var angularVelocity: SIMD3<Float> = .zero
        var packetTime: CFTimeInterval = 0
        var displayQuat: simd_quatf = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)
        var hasReceivedPacket: Bool = false
    }
    private var drState: [String: DeadReckoningState] = [:]
    private var lastDRFrameTime: CFTimeInterval = 0
    private let drCorrectionBlend: Float = 0.1

    // Chest yaw smoothing
    private var chestYawTarget: simd_quatf = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)
    private var chestYawDisplay: simd_quatf = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)
    private var hasChestYawTarget: Bool = false
    private let chestYawBlend: Float = 0.15

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

        handManager.setAllLimbsActive(false)

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

        // Update skeleton positions + dead reckoning every render frame (90Hz)
        skeletonTrackingSubscription = scene.subscribe(to: SceneEvents.Update.self) { [weak self] _ in
            guard let self = self else { return }

            let now = CACurrentMediaTime()
            let frameDt = self.lastDRFrameTime > 0 ? Float(now - self.lastDRFrameTime) : 0
            self.lastDRFrameTime = now

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

                self.activationFrameCounter += 1
                if self.isIMUCalibrated && self.activationFrameCounter % 10 == 0 {
                    self.checkActivationByDistance(headsetTransform: cameraTransform)
                }
            }

            // Chest yaw: SLERP toward target each frame
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

            // Dead reckoning: interpolate limb orientations at 90Hz
            guard frameDt > 0 && frameDt < 0.1 else { return }

            for (segmentName, segment) in Self.orderedSegmentPairs {
                guard var state = self.drState[segmentName], state.hasReceivedPacket else { continue }

                state.displayQuat = self.quatIntegrate(state.displayQuat, omega: state.angularVelocity, dt: frameDt)

                let elapsed = Float(now - state.packetTime)
                var target = state.packetQuat
                if elapsed > 0 && elapsed < 1.0 {
                    target = self.quatIntegrate(state.packetQuat, omega: state.angularVelocity, dt: elapsed)
                }

                if simd_dot(state.displayQuat.vector, target.vector) < 0 {
                    target = simd_quatf(ix: -target.imag.x, iy: -target.imag.y,
                                        iz: -target.imag.z, r: -target.real)
                }
                state.displayQuat = simd_slerp(state.displayQuat, target, self.drCorrectionBlend)

                self.drState[segmentName] = state
                self.handManager.updateIMUCylinderOrientation(segment: segment, orientation: state.displayQuat)
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

                for (nodeID, quat) in orientations {
                    if let segName = nodeIDToSegment[nodeID] {
                        self.lastIMUOrientations[segName] = quat
                    }
                }

                // Auto-start calibration when first IMU data arrives
                if !self.autoCalibrationScheduled && self.calibrationState == .notCalibrated && !orientations.isEmpty {
                    self.autoCalibrationScheduled = true
                    print("IMU data received! Starting calibration in 3 seconds...")

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

                // Dispatch calibrated orientations to active segments
                for (nodeID, rawQuat) in orientations {
                    guard let segmentName = nodeIDToSegment[nodeID] else { continue }

                    if segmentName == chestSegment {
                        guard self.isIMUCalibrated else { continue }
                        let calibrated = self.applyIMUCalibration(segment: segmentName, raw: rawQuat)
                        self.updateChestYaw(calibrated: calibrated)
                        continue
                    }

                    guard self.isIMUCalibrated else { continue }
                    let calibrated = self.applyIMUCalibration(segment: segmentName, raw: rawQuat)
                    let angVels = imuClient.angularVelocities
                    let pktTimes = imuClient.packetTimestamps
                    let omega = angVels[nodeID] ?? .zero
                    let pktTime = pktTimes[nodeID] ?? CACurrentMediaTime()

                    var state = self.drState[segmentName] ?? DeadReckoningState()
                    state.packetQuat = calibrated
                    state.angularVelocity = omega
                    state.packetTime = pktTime
                    if !state.hasReceivedPacket {
                        state.displayQuat = calibrated
                        state.hasReceivedPacket = true
                    }
                    self.drState[segmentName] = state
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
        resetIMUCalibration()

        print("IMU CALIBRATION (Two-Pose Gram-Schmidt)")

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

        if let deviceAnchor = worldTracking.queryDeviceAnchor(atTimestamp: CACurrentMediaTime()) {
            let headsetTransform = deviceAnchor.originFromAnchorTransform
            let headsetForward = SIMD3<Float>(
                -headsetTransform.columns.2.x, 0, -headsetTransform.columns.2.z
            )
            let headsetForwardNormalized = simd_normalize(headsetForward)
            calibrationHeadsetForwardAVP = headsetForwardNormalized
            calibrationHeadsetYaw = atan2(headsetForwardNormalized.x, -headsetForwardNormalized.z)
            calibrationHeadingQ = simd_quatf(angle: -calibrationHeadsetYaw, axis: SIMD3<Float>(0, 1, 0))
            handManager.calibrationHeadingQ = calibrationHeadingQ
        }

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

    private func computeCalibration() {
        var calibratedCount = 0

        let armSegmentList   = activeSegments.filter { !legSegments.contains($0) && $0 != chestSegment }
        let thighSegmentList = activeSegments.filter { thighSegments.contains($0) }
        let shankSegmentList = activeSegments.filter { shankSegments.contains($0) }

        // Phase 1: Arms
        for segment in armSegmentList {
            guard let pose1 = pose1IMUData[segment], let pose2 = pose2IMUData[segment] else { continue }

            let limbDownLocal = SIMD3<Float>(0, -1, 0)
            imuLimbDownAxis[segment] = limbDownLocal

            let limbDown1 = rotateVector(limbDownLocal, by: pose1)
            let e_down = simd_normalize(limbDown1)
            let limbDown2 = rotateVector(limbDownLocal, by: pose2)

            let proj = simd_dot(limbDown2, e_down) * e_down
            let swingRaw = limbDown2 - proj
            guard simd_length(swingRaw) > 0.01 else { continue }
            let e_swing = simd_normalize(swingRaw)
            let e_normal = simd_normalize(simd_cross(e_down, e_swing))

            let t_down = SIMD3<Float>(0, -1, 0)
            let t_fwd = SIMD3<Float>(0, 0, -1)
            let t_normal = simd_normalize(simd_cross(t_down, t_fwd))
            let yawQ = simd_quatf(angle: -calibrationHeadsetYaw, axis: SIMD3<Float>(0, 1, 0))
            let target_down   = rotateVector(t_down, by: yawQ)
            let target_fwd    = rotateVector(t_fwd, by: yawQ)
            let target_normal = rotateVector(t_normal, by: yawQ)

            let M_src = simd_float3x3(columns: (e_normal, e_down, e_swing))
            let M_tgt = simd_float3x3(columns: (target_normal, target_down, target_fwd))
            let R = M_tgt * M_src.transpose

            imuCalibrationOffsets[segment] = simd_quatf(R)
            calibratedCount += 1
        }

        // Phase 2: Thighs
        for segment in thighSegmentList {
            guard let pose1 = pose1IMUData[segment], let pose2 = pose2IMUData[segment] else { continue }

            let enuDown = SIMD3<Float>(0, 0, -1)
            let limbDownLocal = simd_normalize(rotateVector(enuDown, by: pose1.inverse))
            imuLimbDownAxis[segment] = limbDownLocal

            let limbDown1 = rotateVector(limbDownLocal, by: pose1)
            let e_down = simd_normalize(limbDown1)
            let limbDown2 = rotateVector(limbDownLocal, by: pose2)

            let proj = simd_dot(limbDown2, e_down) * e_down
            let swingRaw = limbDown2 - proj
            guard simd_length(swingRaw) > 0.01 else { continue }
            let e_swing = simd_normalize(swingRaw)
            let e_normal = simd_normalize(simd_cross(e_down, e_swing))

            let t_down = SIMD3<Float>(0, -1, 0)
            let t_fwd = SIMD3<Float>(0, 0, -1)
            let t_normal = simd_normalize(simd_cross(t_down, t_fwd))
            let yawQ = simd_quatf(angle: -calibrationHeadsetYaw, axis: SIMD3<Float>(0, 1, 0))
            let target_down   = rotateVector(t_down, by: yawQ)
            let target_fwd    = rotateVector(t_fwd, by: yawQ)
            let target_normal = rotateVector(t_normal, by: yawQ)

            let M_src = simd_float3x3(columns: (e_normal, e_down, e_swing))
            let M_tgt = simd_float3x3(columns: (target_normal, target_down, target_fwd))
            let R = M_tgt * M_src.transpose

            imuCalibrationOffsets[segment] = simd_quatf(R)
            calibratedCount += 1
        }

        // Phase 3: Shanks
        for segment in shankSegmentList {
            guard let pose1 = pose1IMUData[segment], let pose2 = pose2IMUData[segment] else { continue }

            let enuDown = SIMD3<Float>(0, 0, -1)
            let limbDownLocal = simd_normalize(rotateVector(enuDown, by: pose1.inverse))
            imuLimbDownAxis[segment] = limbDownLocal

            let limbDown1 = rotateVector(limbDownLocal, by: pose1)
            let e_down = simd_normalize(limbDown1)
            let limbDown2 = rotateVector(limbDownLocal, by: pose2)

            let proj = simd_dot(limbDown2, e_down) * e_down
            let swingRaw = limbDown2 - proj
            guard simd_length(swingRaw) > 0.01 else { continue }
            let e_swing = simd_normalize(swingRaw)
            let e_normal = simd_normalize(simd_cross(e_down, e_swing))

            let t_down = SIMD3<Float>(0, -1, 0)
            let t_fwd = SIMD3<Float>(0, 0, -1)
            let t_normal = simd_normalize(simd_cross(t_down, t_fwd))
            let yawQ = simd_quatf(angle: -calibrationHeadsetYaw, axis: SIMD3<Float>(0, 1, 0))
            let target_down   = rotateVector(t_down, by: yawQ)
            let target_fwd    = rotateVector(t_fwd, by: yawQ)
            let target_normal = rotateVector(t_normal, by: yawQ)

            let M_src = simd_float3x3(columns: (e_normal, e_down, e_swing))
            let M_tgt = simd_float3x3(columns: (target_normal, target_down, target_fwd))
            let R = M_tgt * M_src.transpose

            imuCalibrationOffsets[segment] = simd_quatf(R)
            calibratedCount += 1
        }

        // Phase 4: Chest
        if let chestPose1 = pose1IMUData[chestSegment] {
            let gravityLocal = detectLimbDownAxis(pose: chestPose1)
            let forwardLocal = detectChestForwardAxis(pose: chestPose1, headsetForward: calibrationHeadsetForwardAVP)
            chestForwardLocalAxis = forwardLocal

            let e_down = simd_normalize(rotateVector(gravityLocal, by: chestPose1))
            let fwdRaw = rotateVector(forwardLocal, by: chestPose1)
            let fwdProj = simd_dot(fwdRaw, e_down) * e_down
            let e_fwd = simd_normalize(fwdRaw - fwdProj)
            let e_normal = simd_normalize(simd_cross(e_down, e_fwd))

            let M_src = simd_float3x3(columns: (e_normal, e_down, e_fwd))

            let t_down = SIMD3<Float>(0, -1, 0)
            let t_fwd = SIMD3<Float>(0, 0, -1)
            let t_normal = simd_normalize(simd_cross(t_down, t_fwd))
            let yawQ = simd_quatf(angle: -calibrationHeadsetYaw, axis: SIMD3<Float>(0, 1, 0))
            let target_down   = rotateVector(t_down, by: yawQ)
            let target_fwd    = rotateVector(t_fwd, by: yawQ)
            let target_normal = rotateVector(t_normal, by: yawQ)

            let M_tgt = simd_float3x3(columns: (target_normal, target_down, target_fwd))
            let R = M_tgt * M_src.transpose

            imuCalibrationOffsets[chestSegment] = simd_quatf(R)
            calibratedCount += 1
        }

        guard calibratedCount > 0 else {
            updateCalibrationStatus(title: "Calibration Failed", detail: "Calibration solve failed. Please re-calibrate.")
            calibrationState = .notCalibrated
            return
        }

        calibrationState = .calibrated
        print("CALIBRATION COMPLETE! (\(calibratedCount)/\(activeSegments.count) devices)")

        // Stop all non-chest IMU streams post-calibration
        for segment in activeSegments where segment != chestSegment {
            sendIMUCommand(segment: segment, command: "STOP")
        }

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

        for seg in activeSegments where seg != chestSegment {
            imuStreamingSegments.remove(seg)
        }
        for seg in activeSegments {
            motorCollisionCounts[seg] = 0
            motorIsOn[seg] = false
        }
        for (_, timer) in motorOffDebounceTimers { timer.invalidate() }
        motorOffDebounceTimers.removeAll()

        handManager.setAllLimbsActive(true)
        areSkeletonsActive = true
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
        imuCalibrationOffsets.removeAll()
        imuLimbDownAxis.removeAll()
        pose1IMUData.removeAll()
        pose2IMUData.removeAll()
        drState.removeAll()
        lastDRFrameTime = 0
        hasChestYawTarget = false
        chestYawTarget = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)
        chestYawDisplay = simd_quatf(ix: 0, iy: 0, iz: 0, r: 1)

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
            motorCollisionCounts[seg] = 0
        }
        for (_, timer) in motorOffDebounceTimers { timer.invalidate() }
        motorOffDebounceTimers.removeAll()
        isMotorEnabled = false

        areSkeletonsActive = false

        if triggerAutoRecalibration {
            autoCalibrationScheduled = false
            updateCalibrationStatus(title: "Calibration Reset", detail: "Waiting for IMU data to auto-start calibration.")
        } else {
            updateCalibrationStatus(title: "Not Calibrated", detail: "Press Re-Calibrate to begin two-pose calibration.")
        }
    }

    private func applyIMUCalibration(segment: String, raw: simd_quatf) -> simd_quatf {
        guard isIMUCalibrated, let calibrationQ = imuCalibrationOffsets[segment] else {
            return raw
        }

        var correctedRaw = raw
        if let limbAxis = imuLimbDownAxis[segment] {
            let defaultDown = SIMD3<Float>(0, -1, 0)
            if simd_dot(limbAxis, defaultDown) < 0.99 {
                let preRotation = shortestRotation(from: defaultDown, to: limbAxis)
                correctedRaw = raw * preRotation
            }
        }

        let calibrated = calibrationQ * correctedRaw

        if segment != chestSegment {
            return buildLimbOrientation(calibrated: calibrated)
        }

        return calibrated
    }

    // MARK: - Chest Yaw Tracking

    private func updateChestYaw(calibrated: simd_quatf) {
        let forward = rotateVector(chestForwardLocalAxis, by: calibrated)
        let hLen = sqrt(forward.x * forward.x + forward.z * forward.z)
        guard hLen > 0.01 else { return }
        let currentChestYaw = atan2(forward.x, -forward.z)
        let deltaYaw = currentChestYaw - calibrationHeadsetYaw

        let target = simd_quatf(angle: -deltaYaw, axis: SIMD3<Float>(0, 1, 0))
        chestYawTarget = target

        if !hasChestYawTarget {
            chestYawDisplay = target
            hasChestYawTarget = true
            handManager.chestYawDelta = target
            handManager.chestYawDeltaInverse = target.inverse
        }
    }

    private func buildLimbOrientation(calibrated: simd_quatf) -> simd_quatf {
        let limbDir = simd_normalize(rotateVector(SIMD3<Float>(0, -1, 0), by: calibrated))

        let refForward = simd_act(handManager.absoluteHeading, SIMD3<Float>(0, 0, -1))

        let negY = limbDir
        let posY = -negY

        var zRaw = refForward - simd_dot(refForward, negY) * negY
        if simd_length(zRaw) < 0.01 {
            zRaw = simd_cross(SIMD3<Float>(0, 1, 0), negY)
        }
        let zAxis = simd_normalize(zRaw)
        let xAxis = simd_cross(posY, zAxis)

        let R = simd_float3x3(columns: (xAxis, posY, zAxis))
        return simd_quatf(R)
    }

    private func quatIntegrate(_ q: simd_quatf, omega: SIMD3<Float>, dt: Float) -> simd_quatf {
        let omegaQuat = simd_quatf(ix: omega.x, iy: omega.y, iz: omega.z, r: 0)
        let qdot = 0.5 * q * omegaQuat
        let integrated = simd_quatf(
            ix: q.imag.x + qdot.imag.x * dt,
            iy: q.imag.y + qdot.imag.y * dt,
            iz: q.imag.z + qdot.imag.z * dt,
            r:  q.real   + qdot.real   * dt
        )
        return integrated.normalized
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

    private func detectChestForwardAxis(pose: simd_quatf, headsetForward: SIMD3<Float>) -> SIMD3<Float> {
        let candidateAxes: [SIMD3<Float>] = [
            SIMD3<Float>( 1, 0, 0), SIMD3<Float>(-1, 0, 0),
            SIMD3<Float>( 0, 1, 0), SIMD3<Float>( 0,-1, 0),
            SIMD3<Float>( 0, 0, 1), SIMD3<Float>( 0, 0,-1),
        ]
        let enuForward = simd_normalize(SIMD3<Float>(headsetForward.x, -headsetForward.z, 0))

        var bestAxis = SIMD3<Float>(0, 1, 0)
        var bestDot: Float = -2.0
        for candidate in candidateAxes {
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

    // MARK: - Collision Handling

    /// Distance-based activation: checks if any obstacle entity is within activationRadius of the headset.
    private func checkActivationByDistance(headsetTransform: simd_float4x4) {
        let headPos = SIMD3<Float>(headsetTransform.columns.3.x, headsetTransform.columns.3.y, headsetTransform.columns.3.z)

        var nearbyObstacle = false
        for entity in obstacleEntities {
            let entityPos = entity.position(relativeTo: nil)
            let distance = simd_length(entityPos - headPos)
            if distance < activationRadius {
                nearbyObstacle = true
                break
            }
        }

        if nearbyObstacle && !areSkeletonsActive {
            areSkeletonsActive = true
            handManager.setAllLimbsActive(true)
            startAllLimbIMUs()
        } else if !nearbyObstacle && areSkeletonsActive {
            // Keep the virtual skeleton visible after calibration even when no obstacle is nearby.
            // We still force motors off for safety when outside activation radius.
            for seg in activeSegments where motorIsOn[seg] ?? false {
                sendMotorCommand(segment: seg, on: false)
                motorIsOn[seg] = false
            }
        }
    }

    private func handleCollisionBegan(_ event: CollisionEvents.Began) {
        let nameA = event.entityA.name
        let nameB = event.entityB.name
        let triggerName: String
        if nameA.hasPrefix("center_segment") || nameA.hasPrefix("left_segment") || nameA.hasPrefix("right_segment") {
            triggerName = nameA
        } else if nameB.hasPrefix("center_segment") || nameB.hasPrefix("left_segment") || nameB.hasPrefix("right_segment") {
            triggerName = nameB
        } else {
            return
        }

        let parts = triggerName.split(separator: "_", maxSplits: 2)
        guard parts.count >= 3 else { return }
        let skeletonID = String(parts[0])
        let segment = String(parts[2])

        guard let motorTarget = mapCollisionToMotor(skeletonID: skeletonID, segment: segment) else { return }

        motorCollisionCounts[motorTarget, default: 0] += 1

        if let imuSeg = HandTrackingManager.IMUBodySegment(rawValue: segment) {
            handManager.setSegmentCollisionIndicator(skeletonID: skeletonID, segment: imuSeg, isColliding: true)
        }

        motorStart(segment: motorTarget)
    }

    private func handleCollisionEnded(_ event: CollisionEvents.Ended) {
        let nameA = event.entityA.name
        let nameB = event.entityB.name
        let triggerName: String
        if nameA.hasPrefix("center_segment") || nameA.hasPrefix("left_segment") || nameA.hasPrefix("right_segment") {
            triggerName = nameA
        } else if nameB.hasPrefix("center_segment") || nameB.hasPrefix("left_segment") || nameB.hasPrefix("right_segment") {
            triggerName = nameB
        } else {
            return
        }

        let parts = triggerName.split(separator: "_", maxSplits: 2)
        guard parts.count >= 3 else { return }
        let skeletonID = String(parts[0])
        let segment = String(parts[2])

        guard let motorTarget = mapCollisionToMotor(skeletonID: skeletonID, segment: segment) else { return }

        motorCollisionCounts[motorTarget, default: 0] = max(0, (motorCollisionCounts[motorTarget] ?? 0) - 1)

        if motorCollisionCounts[motorTarget, default: 0] == 0 {
            if let imuSeg = HandTrackingManager.IMUBodySegment(rawValue: segment) {
                handManager.setSegmentCollisionIndicator(skeletonID: skeletonID, segment: imuSeg, isColliding: false)
            }
        }

        if motorCollisionCounts[motorTarget, default: 0] == 0 {
            motorScheduleStop(segment: motorTarget)
        }
    }

    // MARK: - Motor Mapping

    private func mapCollisionToMotor(skeletonID: String, segment: String) -> String? {
        switch skeletonID {
        case "center":
            return activeSegments.contains(segment) ? segment : nil
        case "left":
            if segment.contains("UpperArm") || segment.contains("Forearm") { return "leftForearm" }
            else if segment.contains("Thigh") { return "leftThigh" }
            else if segment.contains("Shank") { return "leftShank" }
        case "right":
            if segment.contains("UpperArm") || segment.contains("Forearm") { return "rightForearm" }
            else if segment.contains("Thigh") { return "rightThigh" }
            else if segment.contains("Shank") { return "rightShank" }
        default:
            return nil
        }
        return nil
    }

    // MARK: - Motor Control

    private func motorStart(segment: String) {
        motorOffDebounceTimers[segment]?.invalidate()
        motorOffDebounceTimers.removeValue(forKey: segment)
        guard isMotorEnabled else { return }
        guard !(motorIsOn[segment] ?? false) else { return }
        sendMotorCommand(segment: segment, on: true)
        motorIsOn[segment] = true
    }

    private func motorScheduleStop(segment: String) {
        motorOffDebounceTimers[segment]?.invalidate()
        motorOffDebounceTimers[segment] = Timer.scheduledTimer(
            withTimeInterval: motorOffDebounceDelay,
            repeats: false
        ) { [weak self] _ in
            Task { @MainActor in
                guard let self = self else { return }
                if self.motorCollisionCounts[segment, default: 0] == 0 {
                    self.sendMotorCommand(segment: segment, on: false)
                    self.motorIsOn[segment] = false
                }
            }
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

    // MARK: - Manual Override Toggles

    func toggleMotorEnabled() {
        isMotorEnabled.toggle()
        print("Motors \(isMotorEnabled ? "ENABLED" : "DISABLED")")

        if !isMotorEnabled {
            for seg in activeSegments where motorIsOn[seg] ?? false {
                sendMotorCommand(segment: seg, on: false)
                motorIsOn[seg] = false
                motorOffDebounceTimers[seg]?.invalidate()
                motorOffDebounceTimers.removeValue(forKey: seg)
            }
        }
    }

    func toggleIMUOverride() {
        isIMUOverrideActive.toggle()
        print("IMU override \(isIMUOverrideActive ? "ON (all STOPPED)" : "OFF (re-syncing)")")

        if isIMUOverrideActive {
            for seg in activeSegments where imuStreamingSegments.contains(seg) {
                if seg == chestSegment { continue }
                sendIMUCommand(segment: seg, command: "STOP")
                imuStreamingSegments.remove(seg)
            }
        } else {
            if areSkeletonsActive {
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
