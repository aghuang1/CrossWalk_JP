import SwiftUI
import RealityKit
import RealityKitContent
import UIKit
import simd
import ARKit
import os
import Combine

// MARK: - ImmersiveView
struct ImmersiveView: View {
    @Environment(AppModel.self) var appModel
    @Environment(BodyTrackingModel.self) var bodyModel
    @Environment(\.openWindow) private var openWindow

    
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

    @State private var carEntities: [Entity] = []
    // Parallel to `carEntities`: the visible toy-car child of each cube, kept
    // separately so we can scale the visual independently of the cube's hitbox.
    @State private var toyEntities: [Entity] = []
    // Native uniform-fit scale for the toy car at hitboxScale=1, computed once
    // at spawn from `min(hitboxSize / nativeExtents)`. The per-frame timer
    // applies (visualScale / hitboxScale) on top, then multiplies by this.
    @State private var baseToyFitScale: Float = 1.0
    // Cars-from-behind pool. Each slot is either inactive (`carSpawnTimes[i] == nil`)
    // or carries a launch with a fixed lateral X spawn offset and a heading
    // bearing θ (radians around Y, 0 = straight -Z). Position evolves as
    // `spawn + velocity * elapsed`, where velocity rotates -Z by θ. Once
    // the car has cleared the user, the slot is recycled and (if not
    // contacted) counted as an avoidance.
    @State private var carLateralOffsets: [Float] = []
    @State private var carBearings: [Float] = []
    @State private var carSpawnTimes: [CFTimeInterval?] = []
    @State private var carLaunchedCount: Int = 0
    @State private var nextSpawnAt: CFTimeInterval = 0
    // Last runStartTime we observed; used to detect new-run transitions.
    @State private var observedRunStartTime: CFTimeInterval? = nil

    private let logger = Logger(subsystem: "flavinlab.CrossWalk-Tokyo", category: "WorldTracking")

    var body: some View {
        ZStack {
            RealityView { content in
                // Root world anchor
                let rootAnchor = AnchorEntity(world: matrix_identity_float4x4)
                content.add(rootAnchor)
                self.rootAnchorRef = rootAnchor

                // Head-locked anchor
                let head = AnchorEntity(.head)
                content.add(head)
                self.headAnchor = head

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
                        rootAnchor.addChild(skydome)

                        // Load Tokyo crossing environment (NO collision shapes - visual only).
                        // Yawed -90° around Y so the painted crosswalk runs perpendicular
                        // to the user's default forward facing — the user starts with
                        // their back to the lane direction along which cars approach.
                        let crossTokyoEntity = try await Entity.load(named: "Crossing_Tokyo")
                        crossTokyoEntity.orientation = simd_quatf(angle: -.pi / 2, axis: SIMD3<Float>(0, 1, 0))
                        crossTokyoEntity.position.y = 5.95
                        rootAnchor.addChild(crossTokyoEntity)

                        // Cars-from-behind pool. Each car is a *cube* (invisible)
                        // that owns the collision + physics body, with a ToyCar.usdz
                        // model parented underneath as the visible mesh. Pool size
                        // bounds concurrent in-flight cars; the 10Hz timer block
                        // launches them one at a time at a random bearing from the
                        // configured set, with a random gap between launches.
                        let carCount = 6
                        // Realistic car hitbox in world meters (X = length along
                        // travel, Y = height, Z = width). The previous 7×1.8×5 m
                        // box was so wide that the body trigger sphere overlapped
                        // it from ~3.5 m away regardless of the sphere's radius —
                        // making `bodyCollisionRadius` effectively irrelevant.
                        let carHitboxSize = SIMD3<Float>(2.0, 1.5, 1.0)
                        for i in 0..<carCount {
                            // Invisible collision body sized to a real car.
                            let cube = try await Entity.load(named: "Cube")
                            cube.name = "CarCube_\(i)"
                            // Keep parent at unit scale so child orientations and
                            // hitbox dimensions are not stretched non-uniformly.
                            cube.scale = SIMD3<Float>(repeating: 1)
                            // Sit at world origin disabled until launched.
                            cube.position = SIMD3<Float>(0, 0.65, 0)
                            // Explicit CollisionComponent on the ROOT entity so
                            // CollisionEvents.{Began,Ended} fire with entityA/B ==
                            // `cube` itself (name "CarCube_i"). With cube.scale = 1,
                            // localExtents == world extents.
                            self.installObstacleCollision(on: cube, localExtents: carHitboxSize)
                            // Hide the cube via a fully transparent material;
                            // collision component remains active.
                            if let modelEntity = cube as? ModelEntity,
                               var mc = modelEntity.components[ModelComponent.self] as? ModelComponent {
                                mc.materials = [UnlitMaterial(color: .clear)]
                                modelEntity.components[ModelComponent.self] = mc
                            }

                            // Visible toy car — child of the cube so it inherits
                            // every per-frame position update automatically.
                            // Uniform fit inside the hitbox; cube parent is now at
                            // unit scale so no per-axis compensation is needed.
                            let toy = try await Entity.load(named: "ToyCar")
                            let nativeExtents = toy.visualBounds(relativeTo: nil).extents
                            let safeExtents = SIMD3<Float>(
                                max(nativeExtents.x, 1e-4),
                                max(nativeExtents.y, 1e-4),
                                max(nativeExtents.z, 1e-4)
                            )
                            let fitWorld = min(
                                carHitboxSize.x / safeExtents.x,
                                carHitboxSize.y / safeExtents.y,
                                carHitboxSize.z / safeExtents.z
                            )
                            toy.scale = SIMD3<Float>(repeating: fitWorld)
                            // Yaw 90° around Y so the model's long axis aligns
                            // with world X (direction of travel).
                            toy.orientation = simd_quatf(angle: .pi / 2,
                                                         axis: SIMD3<Float>(0, 1, 0))
                            cube.addChild(toy)
                            self.toyEntities.append(toy)
                            // All cars share one toy model + hitbox, so the fit
                            // scale is identical across iterations; latest write wins.
                            self.baseToyFitScale = fitWorld

                            // Hidden until the user presses Start — the timer
                            // block enables cars on each new run.
                            cube.isEnabled = false
                            rootAnchor.addChild(cube)
                            self.carEntities.append(cube)
                            self.carLateralOffsets.append(0)
                            self.carBearings.append(0)
                            self.carSpawnTimes.append(nil)
                        }

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
            } update: { content in
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
                            let avoided = bodyModel.carsAvoidedCount
                            let target = bodyModel.carsToWinTotal
                            let hits = bodyModel.carsHitInstanceIDs.count
                            let line = String(
                                format: "Time %.2fs   Avoided %d / %d   Hits %d",
                                duration, avoided, target, hits
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
                            format: "Avoided %d / %d   Hits %d",
                            bodyModel.carsAvoidedCount,
                            bodyModel.carsToWinTotal,
                            bodyModel.carsHitInstanceIDs.count
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
        // 10Hz timer for car motion and coordinate display
        .onReceive(Timer.publish(every: 0.1, on: .main, in: .common).autoconnect()) { _ in
            // Cars-from-behind. Each pool slot carries a fixed lateral X
            // spawn offset and a fixed bearing θ (radians around Y, 0 =
            // straight -Z). Spawn at (xOffset, ground, +R); velocity is
            // -Z rotated by θ around Y, so trajectories combine a lane
            // displacement and a slight angular drift.
            if observedRunStartTime != bodyModel.runStartTime {
                observedRunStartTime = bodyModel.runStartTime
                if bodyModel.runStartTime != nil {
                    // New run began: reset the pool and schedule first spawn.
                    for car in carEntities { car.isEnabled = false }
                    for idx in carSpawnTimes.indices { carSpawnTimes[idx] = nil }
                    for idx in carLateralOffsets.indices { carLateralOffsets[idx] = 0 }
                    for idx in carBearings.indices { carBearings[idx] = 0 }
                    carLaunchedCount = 0
                    nextSpawnAt = CACurrentMediaTime() + 0.5
                }
            }

            if !bodyModel.isRunActive {
                // Not started yet, or VICTORY reached — clear the pool so
                // stale cars don't linger or trigger contacts.
                for i in carEntities.indices {
                    if carEntities[i].isEnabled { carEntities[i].isEnabled = false }
                    carSpawnTimes[i] = nil
                }
            } else if !carEntities.isEmpty,
                      carLateralOffsets.count == carEntities.count,
                      carBearings.count == carEntities.count,
                      carSpawnTimes.count == carEntities.count {
                let now = CACurrentMediaTime()
                let speed: Float = max(bodyModel.carSpeed, 0.01)
                let R: Float = max(bodyModel.carSpawnDistance, 1.0)
                let totalToWin = bodyModel.carsToWinTotal

                // Spawn launcher.
                if carLaunchedCount < totalToWin,
                   now >= nextSpawnAt,
                   let slot = carSpawnTimes.firstIndex(where: { $0 == nil }) {
                    let latCap = max(bodyModel.maxLateralOffset, 0)
                    let bearCapDeg = max(bodyModel.bearingOffsetDegrees, 0)
                    let bearCapRad = bearCapDeg * .pi / 180
                    carLateralOffsets[slot] = latCap == 0 ? 0 : Float.random(in: -latCap...latCap)
                    carBearings[slot] = bearCapRad == 0 ? 0 : Float.random(in: -bearCapRad...bearCapRad)
                    carSpawnTimes[slot] = now
                    carLaunchedCount += 1
                    // Unique per-launch name so BodyTrackingModel's hit set
                    // dedupes correctly across pool recycling.
                    carEntities[slot].name = "CarCube_\(slot)_p\(carLaunchedCount)"
                    carEntities[slot].isEnabled = true
                    bodyModel.recordCarSpawn()
                    nextSpawnAt = now + Double.random(in: bodyModel.spawnIntervalRange)
                }

                // Per-car position update. Velocity = rotateAroundY(-Z, θ),
                // so vx = sin(θ)*speed (drift), vz = -cos(θ)*speed (forward).
                let pastOriginThreshold: Float = 4.0
                let hitboxScale = max(bodyModel.carHitboxScale, 0.01)
                let visualScale = max(bodyModel.carVisualScale, 0.01)
                for i in carEntities.indices {
                    guard let t0 = carSpawnTimes[i] else { continue }
                    let elapsed = Float(now - t0)
                    let theta = carBearings[i]
                    let s = sin(theta), c = cos(theta)
                    let xPos = carLateralOffsets[i] + speed * s * elapsed
                    let zPos = R - speed * c * elapsed
                    carEntities[i].position = SIMD3<Float>(xPos, 0.65, zPos)
                    // Yaw the cube so its +X (the toy's "front") aligns with
                    // velocity direction (sin θ, 0, -cos θ). A Y-rotation by
                    // `yaw` maps (1,0,0) → (cos yaw, 0, -sin yaw); equating
                    // gives yaw = π/2 - θ. (θ=0 ⇒ yaw=π/2, matches straight-Z.)
                    let yaw: Float = .pi / 2 - theta
                    carEntities[i].orientation = simd_quatf(angle: yaw, axis: SIMD3<Float>(0, 1, 0))
                    carEntities[i].scale = SIMD3<Float>(repeating: hitboxScale)
                    if i < toyEntities.count {
                        toyEntities[i].scale = SIMD3<Float>(
                            repeating: baseToyFitScale * visualScale / hitboxScale
                        )
                    }

                    // Recycle once the car is well past the origin (use the
                    // forward-distance projection so wide bearings still
                    // recycle on the same threshold).
                    let forwardDist = R - speed * c * elapsed
                    if forwardDist < -pastOriginThreshold {
                        let nameAtPass = carEntities[i].name
                        carSpawnTimes[i] = nil
                        carEntities[i].isEnabled = false
                        if !bodyModel.carsHitInstanceIDs.contains(nameAtPass) {
                            bodyModel.recordCarAvoided()
                        }
                        if bodyModel.carsAvoidedCount >= totalToWin {
                            bodyModel.recordVictory()
                        }
                    }
                }
            }

            updateWorldTrackingAndEntities()
            updateTick &+= 1
        }
        .task {
            // Open calibration panel alongside immersive space
            openWindow(id: "calibrationPanel")
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
            if bodyModel.isRunActive, let start = bodyModel.runStartTime {
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
