import SwiftUI
import RealityKit
import RealityKitContent
import UIKit
import simd
import ARKit
import os
import Combine

// MARK: - Obstacle Types

/// One rectangular-prism obstacle spec. Size is in world meters
/// (width × height × depth); `yCenter` is the box-center vertical placement.
/// `xOffsetRange` is the allowable |X| offset from the corridor centerline:
/// curbs sit closer to center, the others can lean further toward the
/// corridor wall.
private struct ObstacleType {
    let name: String
    let size: SIMD3<Float>
    let yCenter: Float
    let color: UIColor
    let xOffsetRange: ClosedRange<Float>
}

/// Pool of obstacle types. All widths are 1.0 m; depths are slim so 6 of
/// these fit comfortably within the course. The course generator
/// guarantees at least one instance of each type per run (see
/// `setupObstacleCourse`).
private let obstacleTypes: [ObstacleType] = [
    ObstacleType(name: "Curb",     size: SIMD3<Float>(1.0, 0.10, 0.20), yCenter: 0.05, color: .systemGray,  xOffsetRange: 0.2...0.7),
    ObstacleType(name: "TrashCan", size: SIMD3<Float>(1.0, 0.60, 0.40), yCenter: 0.30, color: .systemGreen, xOffsetRange: 0.4...1.0),
    ObstacleType(name: "Signpost", size: SIMD3<Float>(1.0, 0.50, 0.10), yCenter: 1.50, color: .systemBlue,  xOffsetRange: 0.4...1.0),
    ObstacleType(name: "Wall",     size: SIMD3<Float>(1.0, 2.00, 0.15), yCenter: 1.00, color: .systemRed,   xOffsetRange: 0.4...1.0),
]

// MARK: - ImmersiveView
struct ImmersiveView: View {
    @Environment(AppModel.self) var appModel
    @Environment(BodyTrackingModel.self) var bodyModel

    
    // anchors & entities
    @State private var rootAnchorRef: AnchorEntity?
    @State private var headAnchor: AnchorEntity?

    // Separate head-anchored red "COLLIDED" text, toggled via isEnabled while
    // `bodyModel.lastCollisionTime` is within `collisionDisplayDuration`.
    @State private var headCollisionTextEntity: ModelEntity?
    // Green head-anchored "VICTORY!" text, toggled via isEnabled while
    // `bodyModel.lastVictoryTime` is within `victoryDisplayDuration`.
    @State private var headVictoryTextEntity: ModelEntity?
    // Head-anchored stats line under VICTORY! showing the finished run's
    // elapsed Time and cars-hit ratio. Mesh is regenerated each time a new
    // victory fires (detected via `lastRenderedVictoryTime`).
    @State private var headVictoryStatsEntity: ModelEntity?
    @State private var lastRenderedVictoryTime: CFTimeInterval = -.infinity

    // Head-anchored live elapsed-time readout. Visible while a run is
    // active; mesh regenerated per 10Hz tick. Hidden once the run ends
    // (the final time is reported by `headVictoryStatsEntity`).
    @State private var headRunTimerEntity: ModelEntity?

    private var isRunningInPreview: Bool {
        ProcessInfo.processInfo.environment["XCODE_RUNNING_FOR_PREVIEWS"] == "1"
    }

    @State private var isWorldTrackingRunning: Bool = false
    @State private var updateTick: Int = 0

    // Forward obstacle course. Re-built on every Start: the previous run's
    // entities are detached, then `obstacleCount` rectangular prisms are
    // randomly placed along -Z at type-specific heights, and a VictoryGoal
    // slab is positioned at the far end. Each obstacle's name embeds the
    // current `runIndex` so the per-name dedupe in
    // `bodyModel.obstaclesHitInstanceIDs` never collides across runs.
    @State private var obstacleEntities: [Entity] = []
    @State private var victoryGoalEntity: Entity?
    @State private var runIndex: Int = 0
    // Last runStartTime we observed; used to detect new-run transitions.
    @State private var observedRunStartTime: CFTimeInterval? = nil
    // Wall-clock time at which the obstacle course should materialize after
    // a Start press. nil when no spawn is pending. Lets blindfolded
    // subjects press Start, close their eyes, then have the course appear
    // after `bodyModel.courseStartGraceSec`.
    @State private var pendingSpawnAt: CFTimeInterval?

    // Refs to the immersive-environment entities so the 10Hz timer can
    // enable/disable them in lockstep with `bodyModel.showVREnvironment`.
    @State private var skydomeEntity: Entity?
    @State private var crossTokyoEntity: Entity?

    private let logger = Logger(subsystem: "flavinlab.CrossWalk-Tokyo", category: "WorldTracking")

    var body: some View {
        ZStack {
            RealityView { content, attachments in
                // Root world anchor
                let rootAnchor = AnchorEntity(world: matrix_identity_float4x4)
                content.add(rootAnchor)
                self.rootAnchorRef = rootAnchor

                // Head-locked anchor
                let head = AnchorEntity(.head)
                content.add(head)
                self.headAnchor = head

                // World-anchored calibration control panel. Spawned in front
                // of the user along the +45° elevation line in the y-z plane:
                // the panel sits `forwardDist` meters ahead and the same
                // distance above eye level, then tilted +45° about the X
                // axis so its face is normal to the user's upward gaze.
                // The panel is well above the 2 m boundary walls, so the
                // user walks under it during a run — visibility is best
                // from the start position; after a run, they look back up.
                // Tunables: forwardDist sets how far ahead/up the panel
                // sits (preserve the 1:1 ratio to keep the 45° elevation),
                // eyeHeight matches the wearer's standing eye height in
                // world coords.
                if let panelAttachment = attachments.entity(for: "calibrationPanel") {
                    let forwardDist: Float    = 1.0
                    let elevationDegrees: Float = 35
                    let eyeHeight: Float      = 1.6
                    let elevationRad = elevationDegrees * .pi / 180
                    // Height above eye derived from (forwardDist, elevation)
                    // so changing one tunable can't desync them. Tilt about
                    // X matches elevation so the panel face is normal to the
                    // user's upward-forward gaze.
                    let yUpFromEye = forwardDist * tan(elevationRad)
                    panelAttachment.position = SIMD3<Float>(0,
                                                            eyeHeight + yUpFromEye,
                                                            -forwardDist)
                    panelAttachment.orientation = simd_quatf(angle: elevationRad,
                                                             axis: SIMD3<Float>(1, 0, 0))
                    rootAnchor.addChild(panelAttachment)
                }

                // Head-anchored COLLIDED warning. Hidden by default; toggled on
                // whenever `bodyModel.lastCollisionTime` is within the display
                // window (see updateWorldTrackingAndEntities + RealityView
                // update: closure).
                let collisionMesh = MeshResource.generateText(
                    "COLLIDED",
                    extrusionDepth: 0.001,
                    font: .systemFont(ofSize: 0.06, weight: .bold),
                    containerFrame: .zero,
                    alignment: .center,
                    lineBreakMode: .byTruncatingTail
                )
                let collisionEntity = ModelEntity(
                    mesh: collisionMesh,
                    materials: [UnlitMaterial(color: .systemRed)]
                )
                collisionEntity.name = "HeadCollisionText"
                collisionEntity.position = SIMD3<Float>(0, 0.05, -0.7)
                collisionEntity.isEnabled = false
                head.addChild(collisionEntity)
                DispatchQueue.main.async {
                    self.headCollisionTextEntity = collisionEntity
                }

                // Head-anchored VICTORY! text, green, same placement as COLLIDED
                // but mutually exclusive visually (different event, different
                // timestamp). Hidden by default; toggled via isEnabled while
                // `bodyModel.lastVictoryTime` is within the display window.
                let victoryMesh = MeshResource.generateText(
                    "VICTORY!",
                    extrusionDepth: 0.001,
                    font: .systemFont(ofSize: 0.06, weight: .bold),
                    containerFrame: .zero,
                    alignment: .center,
                    lineBreakMode: .byTruncatingTail
                )
                let victoryEntity = ModelEntity(
                    mesh: victoryMesh,
                    materials: [UnlitMaterial(color: .systemGreen)]
                )
                victoryEntity.name = "HeadVictoryText"
                victoryEntity.position = SIMD3<Float>(0, 0.05, -0.7)
                victoryEntity.isEnabled = false
                head.addChild(victoryEntity)
                DispatchQueue.main.async {
                    self.headVictoryTextEntity = victoryEntity
                }

                // Head-anchored stats line (Time + Cars hit). Mesh is
                // regenerated per-victory in the RealityView update closure
                // once real values are available; start with a placeholder.
                let victoryStatsEntity = ModelEntity(
                    mesh: MeshResource.generateText(
                        " ",
                        extrusionDepth: 0.001,
                        font: .systemFont(ofSize: 0.03, weight: .medium),
                        containerFrame: .zero,
                        alignment: .center,
                        lineBreakMode: .byTruncatingTail
                    ),
                    materials: [UnlitMaterial(color: .white)]
                )
                victoryStatsEntity.name = "HeadVictoryStats"
                // Sit just below VICTORY! in head-anchor space.
                victoryStatsEntity.position = SIMD3<Float>(0, -0.02, -0.7)
                victoryStatsEntity.isEnabled = false
                head.addChild(victoryStatsEntity)
                DispatchQueue.main.async {
                    self.headVictoryStatsEntity = victoryStatsEntity
                }

                // Head-anchored live run timer. Sits above COLLIDED/VICTORY!
                // in head-anchor space so it remains readable while either
                // banner is shown. Mesh is rewritten every 10Hz tick while
                // `isRunActive`, and hidden otherwise.
                let runTimerEntity = ModelEntity(
                    mesh: MeshResource.generateText(
                        "0.00s",
                        extrusionDepth: 0.001,
                        font: .systemFont(ofSize: 0.04, weight: .bold),
                        containerFrame: .zero,
                        alignment: .center,
                        lineBreakMode: .byTruncatingTail
                    ),
                    materials: [UnlitMaterial(color: .white)]
                )
                runTimerEntity.name = "HeadRunTimer"
                runTimerEntity.position = SIMD3<Float>(0, 0.13, -0.7)
                runTimerEntity.isEnabled = false
                head.addChild(runTimerEntity)
                DispatchQueue.main.async {
                    self.headRunTimerEntity = runTimerEntity
                }

                // Add body tracking content entity to scene
                let bodyContentEntity = bodyModel.setupContentEntity()
                rootAnchor.addChild(bodyContentEntity)

                // Load scene entities asynchronously
                Task { @MainActor in
                    do {
                        // Large skydome — inverted sphere so there's always sky on the
                        // horizon when looking past the Tokyo crosswalk geometry.
                        let skydomeRadius: Float = 80.0
                        let skydome = ModelEntity(
                            mesh: .generateSphere(radius: skydomeRadius),
                            materials: [{
                                var m = SimpleMaterial(
                                    color: UIColor(red: 0.55, green: 0.75, blue: 0.95, alpha: 1.0),
                                    isMetallic: false
                                )
                                m.faceCulling = .front   // render the INSIDE of the sphere
                                return m
                            }()]
                        )
                        skydome.name = "Skydome"
                        skydome.position = SIMD3<Float>(0, 0, 0)
                        skydome.isEnabled = bodyModel.showVREnvironment
                        rootAnchor.addChild(skydome)
                        self.skydomeEntity = skydome

                        // Load Tokyo crossing environment (NO collision shapes - visual only).
                        // Yawed -90° around Y so the painted crosswalk runs perpendicular
                        // to the user's default forward facing — the user starts with
                        // their back to the lane direction along which cars approach.
                        let crossTokyo = try await Entity.load(named: "Crossing_Tokyo")
                        crossTokyo.orientation = simd_quatf(angle: -.pi / 2, axis: SIMD3<Float>(0, 1, 0))
                        crossTokyo.position.y = 5.95
                        crossTokyo.isEnabled = bodyModel.showVREnvironment
                        rootAnchor.addChild(crossTokyo)
                        self.crossTokyoEntity = crossTokyo

                        // Obstacle course entities are built lazily on each
                        // Start — `setupObstacleCourse(rootAnchor:)` tears
                        // down the previous run's obstacles and lays out a
                        // fresh randomized set.

                        // Directional light
                        let lightEntity = Entity()
                        var lightComponent = DirectionalLightComponent()
                        lightComponent.intensity = 1000
                        lightComponent.color = .white
                        lightEntity.components[DirectionalLightComponent.self] = lightComponent
                        lightEntity.orientation = simd_quatf(angle: -.pi/4, axis: SIMD3<Float>(1,0,0))
                        rootAnchor.addChild(lightEntity)

                    } catch {
                        print("Error loading entities: \(error)")
                    }
                }
            } update: { content, attachments in
                bodyModel.attachToSceneIfReady()
                // Reactively toggle head-anchored COLLIDED + VICTORY! texts.
                // Reading `lastCollisionTime` / `lastVictoryTime` establishes
                // @Observable dependencies, so this closure re-fires the
                // instant `updateMotorsByProximity` bumps either timestamp —
                // no 10Hz timer round-trip needed.
                let now = CACurrentMediaTime()
                if let collisionText = headCollisionTextEntity {
                    let isColliding = now - bodyModel.lastCollisionTime
                        < bodyModel.collisionDisplayDuration
                    if collisionText.isEnabled != isColliding {
                        collisionText.isEnabled = isColliding
                    }
                }
                if let victoryText = headVictoryTextEntity {
                    let isVictory = now - bodyModel.lastVictoryTime
                        < bodyModel.victoryDisplayDuration
                    if victoryText.isEnabled != isVictory {
                        victoryText.isEnabled = isVictory
                    }
                    // Stats line: rebuild mesh on each new victory (rising
                    // edge of `lastVictoryTime`) so it shows that run's final
                    // duration + hit ratio; toggle visibility in lockstep.
                    if let statsText = headVictoryStatsEntity {
                        if isVictory, bodyModel.lastVictoryTime > lastRenderedVictoryTime {
                            lastRenderedVictoryTime = bodyModel.lastVictoryTime
                            let duration = bodyModel.runDuration ?? 0
                            let touched = bodyModel.obstaclesHitInstanceIDs.count
                            let total   = bodyModel.totalObstaclesInCourse
                            let line = String(
                                format: "Time %.2fs   Touched %d / %d obstacles",
                                duration, touched, total
                            )
                            statsText.model?.mesh = MeshResource.generateText(
                                line,
                                extrusionDepth: 0.001,
                                font: .systemFont(ofSize: 0.03, weight: .medium),
                                containerFrame: .zero,
                                alignment: .center,
                                lineBreakMode: .byTruncatingTail
                            )
                        }
                        if statsText.isEnabled != isVictory {
                            statsText.isEnabled = isVictory
                        }
                    }
                }
            } attachments: {
                // SwiftUI panel projected into the immersive scene. Inherits
                // the surrounding view's environment, but `bodyModel` is
                // passed explicitly so the @Environment lookup resolves the
                // same instance the rest of the immersive view uses.
                Attachment(id: "calibrationPanel") {
                    CalibrationControlPanel()
                        .environment(bodyModel)
                        .frame(width: 720, height: 720)
                }
            }
            .ignoresSafeArea()

            // HUD: only shows transient COLLIDED during a run and a persistent
            // result panel (VICTORY! + time + hit ratio) once the run ends.
            VStack(spacing: 8) {
                // Read `updateTick` so SwiftUI re-evaluates this VStack on
                // every 10Hz timer tick. Without it, the `if now - last < 5`
                // check only runs when `lastCollisionTime` is *written* — so
                // COLLIDED would appear but never time out.
                let _ = updateTick
                // Transient COLLIDED banner during active contact. Visible
                // for `collisionDisplayDuration` after the most recent
                // write to `lastCollisionTime`.
                if CACurrentMediaTime() - bodyModel.lastCollisionTime < bodyModel.collisionDisplayDuration {
                    Text("COLLIDED")
                        .font(.title3).bold()
                        .foregroundStyle(.red)
                        .padding(.horizontal, 16)
                        .padding(.vertical, 8)
                        .background(.black.opacity(0.75), in: RoundedRectangle(cornerRadius: 10))
                }

                // Post-run result panel: VICTORY! + elapsed time + hit ratio.
                // Persists from the moment `runEndTime` is set until the
                // next `startRun()` call.
                if let duration = bodyModel.runDuration {
                    VStack(spacing: 4) {
                        Text("VICTORY!")
                            .font(.largeTitle).bold()
                            .foregroundStyle(.green)
                        Text(String(format: "Time: %.2fs", duration))
                            .font(.title3)
                            .foregroundStyle(.white)
                            .monospacedDigit()
                        Text(String(
                            format: "Touched %d / %d obstacles",
                            bodyModel.obstaclesHitInstanceIDs.count,
                            bodyModel.totalObstaclesInCourse
                        ))
                        .font(.body)
                        .foregroundStyle(.white)
                        .monospacedDigit()
                    }
                    .padding(.horizontal, 20)
                    .padding(.vertical, 12)
                    .background(.black.opacity(0.8), in: RoundedRectangle(cornerRadius: 14))
                }
            }
            .frame(maxWidth: .infinity, maxHeight: .infinity, alignment: .top)
            .padding(.top, 18)
        }
        .ignoresSafeArea()
        // 10Hz timer for run-transition detection and env toggle mirroring.
        .onReceive(Timer.publish(every: 0.1, on: .main, in: .common).autoconnect()) { _ in
            // Detect Start (or Restart). Defer the actual obstacle spawn by
            // `courseStartGraceSec` so the subject has time to close their
            // eyes; tear down the prior run's obstacles immediately so they
            // don't linger during the prep window.
            if observedRunStartTime != bodyModel.runStartTime {
                observedRunStartTime = bodyModel.runStartTime
                if bodyModel.runStartTime != nil {
                    for e in obstacleEntities { e.removeFromParent() }
                    obstacleEntities.removeAll()
                    victoryGoalEntity?.isEnabled = false
                    let grace = max(0.0, CFTimeInterval(bodyModel.courseStartGraceSec))
                    pendingSpawnAt = CACurrentMediaTime() + grace
                } else {
                    // runStartTime cleared without victory → user pressed
                    // Stop Run. Cancel any pending spawn, tear down the
                    // current course, and hide the victory goal so the
                    // scene returns to its pre-run state.
                    pendingSpawnAt = nil
                    for e in obstacleEntities { e.removeFromParent() }
                    obstacleEntities.removeAll()
                    victoryGoalEntity?.isEnabled = false
                }
            }

            // Course materializes when the prep window elapses. Reset
            // `runStartTime` at that moment so the on-victory stats and the
            // head-anchored elapsed timer measure navigation time only,
            // not navigation + prep.
            if let due = pendingSpawnAt,
               CACurrentMediaTime() >= due,
               let root = rootAnchorRef {
                runIndex += 1
                setupObstacleCourse(rootAnchor: root, runIndex: runIndex)
                let now = CACurrentMediaTime()
                bodyModel.runStartTime = now
                observedRunStartTime = now      // suppress re-trigger above
                pendingSpawnAt = nil
            }

            // Mirror the env toggle so the panel switch applies live without
            // re-entering the immersive space.
            let envOn = bodyModel.showVREnvironment
            if let sky = skydomeEntity, sky.isEnabled != envOn { sky.isEnabled = envOn }
            if let env = crossTokyoEntity, env.isEnabled != envOn { env.isEnabled = envOn }

            updateWorldTrackingAndEntities()
            updateTick &+= 1
        }
        .task {
            // Start ARKit session with body tracking providers
            #if !targetEnvironment(simulator)
            if !isRunningInPreview {
                do {
                    if bodyModel.dataProvidersAreSupported {
                        if bodyModel.isReadyToRun {
                            try await bodyModel.session.run([
                                bodyModel.handManager.handTracking,
                                bodyModel.worldTracking
                            ])
                            isWorldTrackingRunning = true
                            logger.debug("ARKitSession started with body tracking providers.")
                        }
                    } else {
                        bodyModel.errorMessage = "Data providers not supported."
                    }
                } catch {
                    isWorldTrackingRunning = false
                    bodyModel.errorMessage = "Failed to start session: \(error)"
                    logger.error("Failed to start session: \(error.localizedDescription)")
                }
            }
            #endif
        }
        .task {
            // Process hand tracking updates
            await bodyModel.processHandUpdates()
        }
        .onDisappear {
            headCollisionTextEntity = nil
            headVictoryTextEntity = nil
            headVictoryStatsEntity = nil
            headRunTimerEntity = nil
            // Drop scene-bound subscriptions BEFORE the scene is destroyed.
            // The next time the immersive space opens, attachToSceneIfReady
            // will re-subscribe against the new scene (and re-kick the IMU
            // streams) — without this, skeleton render and haptics silently
            // stop working after the user backgrounds + foregrounds the app.
            bodyModel.detachFromScene()
            #if !targetEnvironment(simulator)
            if !isRunningInPreview {
                bodyModel.session.stop()
            }
            #endif
            isWorldTrackingRunning = false
        }
    }

    // MARK: - Helpers

    /// Installs an `.obstacle` CollisionComponent on the *root* entity with a
    /// unit-space box of `localExtents`. World-space size = `localExtents *
    /// entity.scale`, so pass (1,1,1) for obstacles whose visible size comes
    /// from their `.scale`. Guarantees that CollisionEvents fire with the
    /// named root entity rather than an unnamed child mesh.
    private func installObstacleCollision(on entity: Entity, localExtents: SIMD3<Float>) {
        let shape = ShapeResource.generateBox(width: localExtents.x,
                                              height: localExtents.y,
                                              depth: localExtents.z)
        var collision = CollisionComponent(shapes: [shape])
        collision.filter = CollisionFilter(group: .obstacle, mask: .skeleton)
        entity.components.set(collision)
    }

    // MARK: - Obstacle Course Generation

    /// Tears down the previous run's obstacles + victory goal and lays out
    /// a fresh randomized course of `bodyModel.obstacleCount` rectangular-
    /// prism obstacles along -Z, then positions a green VictoryGoal slab
    /// past the last slot. Each obstacle's name embeds `runIndex` so the
    /// per-name dedupe set in `bodyModel.obstaclesHitInstanceIDs` never
    /// collides across runs.
    private func setupObstacleCourse(rootAnchor: Entity, runIndex: Int) {
        // 1. Clear previous run.
        for e in obstacleEntities { e.removeFromParent() }
        obstacleEntities.removeAll()

        // 2. Read course params off the model. `pathLen` is the user-to-goal
        //    distance (-Z). Obstacles are placed at fixed `spacing` apart
        //    starting `prefix` meters in front of the user, and the goal
        //    is auto-extended past the last slot if `pathLen` is too short.
        let count    = max(1, bodyModel.obstacleCount)
        let halfW    = max(0.5, bodyModel.corridorHalfWidth)
        let userPathLen = max(2.0, bodyModel.coursePathLength)
        // Random forward buffer so the first obstacle isn't always at the
        // same Z. Range matches the user's ask of 0.5–1.0 m.
        let prefix = Float.random(in: 0.5...1.0)
        let spacing = max(0.3, bodyModel.obstacleMinSpacing)
        let finalBuffer: Float = 1.0
        // Last obstacle Z = -prefix - spacing*(count - 1). Goal sits at
        // least `finalBuffer` past the last obstacle, or further if
        // `coursePathLength` is set larger.
        let lastSlotZ = -prefix - spacing * Float(max(0, count - 1))
        let pathLen   = max(userPathLen, -lastSlotZ + finalBuffer)
        let zJitterCap = max(0.0, min(0.20, spacing * 0.25))

        // 3. Build the type sequence. Guarantee at least one of each
        //    obstacle type when `count >= obstacleTypes.count`; fill any
        //    remaining slots with uniform-random picks; shuffle so the
        //    guaranteed-type ordering isn't predictable across runs.
        var typeSequence: [ObstacleType] = []
        if count >= obstacleTypes.count {
            typeSequence.append(contentsOf: obstacleTypes)
            for _ in 0..<(count - obstacleTypes.count) {
                typeSequence.append(obstacleTypes.randomElement()!)
            }
        } else {
            typeSequence.append(contentsOf: obstacleTypes.shuffled().prefix(count))
        }
        typeSequence.shuffle()

        // 4. Lay out one obstacle per Z slot. Slots run from -prefix back
        //    to -prefix - spacing*(count-1).
        for i in 0..<count {
            let zJitter = zJitterCap > 0 ? Float.random(in: -zJitterCap...zJitterCap) : 0
            let zSlot   = -prefix - spacing * Float(i) + zJitter

            let type = typeSequence[i]

            // X: pick a side, then offset by an amount drawn from this
            // type's `xOffsetRange` (curbs hug center, the others can lean
            // toward the corridor wall). Upper bound is clamped so the
            // obstacle's far edge stays inside the corridor.
            // Always +X when single-side testing is on, otherwise random.
            let sideSign: Float = bodyModel.spawnRightSideOnly
                ? 1.0
                : (Bool.random() ? 1.0 : -1.0)
            let halfObstacleW = type.size.x * 0.5
            let corridorMax   = max(halfObstacleW + 0.05, halfW - halfObstacleW)
            let lo = max(0.0, type.xOffsetRange.lowerBound)
            let hi = max(lo + 0.05, min(type.xOffsetRange.upperBound, corridorMax))
            let xOffsetMag = Float.random(in: lo...hi)
            let xOffset = sideSign * xOffsetMag

            let mesh = MeshResource.generateBox(size: type.size)
            let mat  = SimpleMaterial(color: type.color, isMetallic: false)
            let e    = ModelEntity(mesh: mesh, materials: [mat])
            e.name = "Obstacle_\(i)_\(type.name)_p\(runIndex)"
            e.position = SIMD3<Float>(xOffset, type.yCenter, zSlot)
            e.scale = SIMD3<Float>(repeating: 1)
            installObstacleCollision(on: e, localExtents: type.size)
            rootAnchor.addChild(e)
            obstacleEntities.append(e)
        }

        // 5. Spawn or re-position the victory goal at exactly `-pathLen`
        //    (so `coursePathLength` is literally the user-to-goal distance).
        //    Entity is reused across runs to avoid re-allocating its
        //    collision shape each time.
        let goalSize = SIMD3<Float>(2.0, 2.0, 0.10)
        if victoryGoalEntity == nil {
            let g = ModelEntity(
                mesh: .generateBox(size: goalSize),
                materials: [SimpleMaterial(color: .systemGreen, isMetallic: false)]
            )
            g.name = "VictoryGoal"
            installObstacleCollision(on: g, localExtents: goalSize)
            rootAnchor.addChild(g)
            victoryGoalEntity = g
        }
        victoryGoalEntity?.position = SIMD3<Float>(0, goalSize.y * 0.5, -pathLen)
        victoryGoalEntity?.isEnabled = true

        // 6. Publish total so on-victory stats show "touched N / TOTAL".
        //    Set BEFORE boundary walls are appended so the metric reflects
        //    only inner dodge-obstacles, not the guidance walls.
        bodyModel.setObstacleCourseTotal(obstacleEntities.count)

        // 7. Boundary walls running the full length of the corridor at ±halfW.
        //    Same `.obstacle` collision group as obstacles, so the proximity
        //    field drives motor haptics as a limb approaches a wall and the
        //    body trigger fires the COLLIDED banner on direct contact —
        //    giving subjects a tactile "stay centered" cue. Named
        //    `BoundaryWall_*` (not `Obstacle_*`) so registerBodyContact in
        //    BodyTrackingModel hits the generic-collision branch and does
        //    NOT inflate the touched/total tally.
        let wallHeight: Float    = 2.0
        let wallThickness: Float = 0.10
        let wallLength           = pathLen + 1.0
        let wallSize             = SIMD3<Float>(wallThickness, wallHeight, wallLength)
        let wallCenterZ          = -pathLen * 0.5
        let wallCenterY          = wallHeight * 0.5
        for sideSign in [Float(-1), Float(1)] {
            let sideName = sideSign < 0 ? "Left" : "Right"
            // Inner face flush with ±halfW; wall thickness extends outward.
            let xCenter = sideSign * (halfW + wallThickness * 0.5)
            let wall = ModelEntity(
                mesh: .generateBox(size: wallSize),
                materials: [SimpleMaterial(color: .systemPurple, isMetallic: false)]
            )
            wall.name = "BoundaryWall_\(sideName)_p\(runIndex)"
            wall.position = SIMD3<Float>(xCenter, wallCenterY, wallCenterZ)
            installObstacleCollision(on: wall, localExtents: wallSize)
            rootAnchor.addChild(wall)
            obstacleEntities.append(wall)
        }
    }

    // MARK: - Head-Anchored Banner Toggles

    /// 10 Hz fallback: also runs in the RealityView `update:` closure for
    /// instant show on the rising edge. This loop ensures the hide transition
    /// fires on time even when no observable state is churning.
    private func updateWorldTrackingAndEntities() {
        let now = CACurrentMediaTime()
        if let collisionText = headCollisionTextEntity {
            let isColliding = now - bodyModel.lastCollisionTime
                < bodyModel.collisionDisplayDuration
            if collisionText.isEnabled != isColliding {
                collisionText.isEnabled = isColliding
            }
        }
        if let victoryText = headVictoryTextEntity {
            let isVictory = now - bodyModel.lastVictoryTime
                < bodyModel.victoryDisplayDuration
            if victoryText.isEnabled != isVictory {
                victoryText.isEnabled = isVictory
            }
            // Mirror stats visibility here so the hide transition is never
            // missed even if the RealityView update closure idles.
            if let statsText = headVictoryStatsEntity,
               statsText.isEnabled != isVictory {
                statsText.isEnabled = isVictory
            }
        }
        // Live run timer: rewrite the mesh every tick while a run is in
        // progress; hide once VICTORY freezes runEndTime (the final time
        // is reported by `headVictoryStatsEntity`).
        if let timerText = headRunTimerEntity {
            // During the pre-spawn grace window the obstacle course hasn't
            // materialized yet — show a "Get Ready Nn" countdown so a
            // sighted operator sees the prep clock; the timer flips to
            // elapsed-time as soon as the course spawns.
            if let due = pendingSpawnAt {
                let secsLeft = max(0, Int(ceil(due - now)))
                timerText.model?.mesh = MeshResource.generateText(
                    "Get Ready  \(secsLeft)",
                    extrusionDepth: 0.001,
                    font: .systemFont(ofSize: 0.04, weight: .bold),
                    containerFrame: .zero,
                    alignment: .center,
                    lineBreakMode: .byTruncatingTail
                )
                if !timerText.isEnabled { timerText.isEnabled = true }
            } else if bodyModel.isRunActive, let start = bodyModel.runStartTime {
                let elapsed = now - start
                timerText.model?.mesh = MeshResource.generateText(
                    String(format: "%.2fs", elapsed),
                    extrusionDepth: 0.001,
                    font: .systemFont(ofSize: 0.04, weight: .bold),
                    containerFrame: .zero,
                    alignment: .center,
                    lineBreakMode: .byTruncatingTail
                )
                if !timerText.isEnabled { timerText.isEnabled = true }
            } else if timerText.isEnabled {
                timerText.isEnabled = false
            }
        }
    }
}

#Preview(immersionStyle: .full) {
    ImmersiveView()
        .environment(AppModel())
        .environment(BodyTrackingModel())
}
