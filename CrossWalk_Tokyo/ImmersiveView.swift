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

    @State private var headTextEntity: ModelEntity?
    @State private var lastHeadText: String = ""

    // GUI state
    @State private var guiWorldPosition: SIMD3<Float> = .zero
    @State private var guiOrientationDegrees: SIMD3<Float> = .zero

    private var isRunningInPreview: Bool {
        ProcessInfo.processInfo.environment["XCODE_RUNNING_FOR_PREVIEWS"] == "1"
    }

    @State private var isWorldTrackingRunning: Bool = false
    @State private var updateTick: Int = 0

    // Cube wandering reference
    @State private var targetCubeEntity: Entity?
    @State private var carEntity: Entity?
    @State private var userStartPosition: SIMD3<Float>?

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

                // Head-anchored text
                let initialText = "Loading coordinate..."
                let initialMesh = MeshResource.generateText(
                    initialText,
                    extrusionDepth: 0.001,
                    font: .systemFont(ofSize: 0.03, weight: .semibold),
                    containerFrame: .zero,
                    alignment: .center,
                    lineBreakMode: .byTruncatingTail
                )
                let initialMaterial = UnlitMaterial(color: .white)
                let textEntity = ModelEntity(mesh: initialMesh, materials: [initialMaterial])
                textEntity.name = "HeadWorldText"
                textEntity.position = SIMD3<Float>(0, 0.05, -0.7)
                textEntity.scale = SIMD3<Float>(repeating: 1.0)
                head.addChild(textEntity)

                DispatchQueue.main.async {
                    self.headTextEntity = textEntity
                    self.lastHeadText = initialText
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

                        // Load Tokyo crossing environment (NO collision shapes - visual only)
                        let crossTokyoEntity = try await Entity.load(named: "Crossing_Tokyo")
                        crossTokyoEntity.position.y = 5.95
                        rootAnchor.addChild(crossTokyoEntity)

                        // Spawn obstacle cube with collision shapes
                        let spawnCube = try await Entity.load(named: "Cube")
                        spawnCube.name = "SpawnCube"
                        spawnCube.scale = SIMD3<Float>(1.5, 10.2, 1.5)
                        spawnCube.position = SIMD3<Float>(0, 0.85, -2.0)

                        // Generate collision shapes and assign to obstacle group
                        spawnCube.generateCollisionShapes(recursive: true)
                        self.applyObstacleCollisionGroup(to: spawnCube)

                        // SpawnCube moves, so it needs kinematic physics
                        spawnCube.components.set(PhysicsBodyComponent(
                            shapes: [.generateBox(width: 1.5, height: 10.2, depth: 1.5)],
                            mass: 0,
                            mode: .kinematic
                        ))

                        if let modelEntity = spawnCube as? ModelEntity,
                           var mc = modelEntity.components[ModelComponent.self] as? ModelComponent {
                            mc.materials = [SimpleMaterial(color: .blue, isMetallic: false)]
                            modelEntity.components[ModelComponent.self] = mc
                        }

                        rootAnchor.addChild(spawnCube)
                        self.targetCubeEntity = spawnCube

                        // Car: wide obstacle that drives R→L perpendicular to the user's starting forward.
                        let car = try await Entity.load(named: "Cube")
                        car.name = "CarCube"
                        car.scale = SIMD3<Float>(4.0, 1.8, 2.0)
                        car.position = SIMD3<Float>(8.0, 0.9, -2.0)
                        car.generateCollisionShapes(recursive: true)
                        self.applyObstacleCollisionGroup(to: car)
                        car.components.set(PhysicsBodyComponent(
                            shapes: [.generateBox(width: 4.0, height: 1.8, depth: 2.0)],
                            mass: 0,
                            mode: .kinematic
                        ))
                        if let modelEntity = car as? ModelEntity,
                           var mc = modelEntity.components[ModelComponent.self] as? ModelComponent {
                            mc.materials = [SimpleMaterial(color: .red, isMetallic: false)]
                            modelEntity.components[ModelComponent.self] = mc
                        }
                        rootAnchor.addChild(car)
                        self.carEntity = car

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
            }
            .ignoresSafeArea()

            // Face-fixed world-coordinate HUD
            VStack(spacing: 4) {
                Text("My World Coordinate")
                    .font(.caption).bold()
                Text(String(format: "x: %.3f  y: %.3f  z: %.3f", guiWorldPosition.x, guiWorldPosition.y, guiWorldPosition.z))
                    .font(.caption2)
                    .monospacedDigit()

                Text(String(format: "yaw: %.1f  pitch: %.1f  roll: %.1f", guiOrientationDegrees.x, guiOrientationDegrees.y, guiOrientationDegrees.z))
                    .font(.caption2)
                    .monospacedDigit()
            }
            .padding(.horizontal, 12)
            .padding(.vertical, 8)
            .foregroundStyle(.white)
            .background(.black.opacity(0.6), in: RoundedRectangle(cornerRadius: 10))
            .frame(maxWidth: .infinity, maxHeight: .infinity, alignment: .top)
            .padding(.top, 18)
        }
        .ignoresSafeArea()
        // 10Hz timer for cube wandering and coordinate display
        .onReceive(Timer.publish(every: 0.1, on: .main, in: .common).autoconnect()) { _ in
            // Cube wandering
            if let cube = targetCubeEntity {
                let t = Float(updateTick) * 0.1
                let wanderX = sin(t * 0.5) * 4.0 + cos(t * 0.3) * 2.0
                let wanderZ = -3.0 + cos(t * 0.4) * 4.0 + sin(t * 0.2) * 2.0
                cube.position = SIMD3<Float>(wanderX, 0.85, wanderZ)
            }

            // Car: drive from +X (user's right) to -X (user's left) at ~2 m/s.
            // Track range is 16m; cycle is 8s driving + 1s hidden pause then respawn.
            if let car = carEntity {
                let carSpeed: Float = 2.0
                let startX: Float = 8.0
                let endX: Float = -8.0
                let driveDistance = startX - endX
                let driveDuration = driveDistance / carSpeed   // 8s
                let cycleDuration: Float = driveDuration + 1.0 // 9s total
                let t = Float(updateTick) * 0.1
                let phase = t.truncatingRemainder(dividingBy: cycleDuration)
                let carZ: Float = -2.0
                if phase < driveDuration {
                    car.position = SIMD3<Float>(startX - carSpeed * phase, 0.9, carZ)
                } else {
                    // Briefly park off-screen before the next pass.
                    car.position = SIMD3<Float>(startX, 0.9, carZ)
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
            headTextEntity = nil
            #if !targetEnvironment(simulator)
            if !isRunningInPreview {
                bodyModel.session.stop()
            }
            #endif
            isWorldTrackingRunning = false
        }
    }

    // MARK: - Helpers

    /// Recursively applies .obstacle collision group to entity and its children
    private func applyObstacleCollisionGroup(to entity: Entity) {
        if var collision = entity.components[CollisionComponent.self] as? CollisionComponent {
            collision.filter = CollisionFilter(group: .obstacle, mask: .skeleton)
            entity.components.set(collision)
        }
        for child in entity.children {
            applyObstacleCollisionGroup(to: child)
        }
    }

    // MARK: - World Tracking Display

    private func updateWorldTrackingAndEntities() {
        guard let headEntity = headTextEntity else { return }

        #if !targetEnvironment(simulator)
        guard isWorldTrackingRunning, !isRunningInPreview else { return }
        guard let deviceAnchor = bodyModel.worldTracking.queryDeviceAnchor(atTimestamp: CACurrentMediaTime()) else { return }

        let transformMatrix = deviceAnchor.originFromAnchorTransform
        let p = SIMD3<Float>(transformMatrix.columns.3.x, transformMatrix.columns.3.y, transformMatrix.columns.3.z)

        let q = Transform(matrix: transformMatrix).rotation
        let qw = q.real
        let qx = q.imag.x
        let qy = q.imag.y
        let qz = q.imag.z

        let sinrCosp = 2 * (qw * qx + qy * qz)
        let cosrCosp = 1 - 2 * (qx * qx + qy * qy)
        let roll = atan2(sinrCosp, cosrCosp)

        let sinp = 2 * (qw * qy - qz * qx)
        let pitch = abs(sinp) >= 1 ? (sinp >= 0 ? Float.pi / 2 : -Float.pi / 2) : asin(sinp)

        let sinyCosp = 2 * (qw * qz + qx * qy)
        let cosyCosp = 1 - 2 * (qy * qy + qz * qz)
        let yaw = atan2(sinyCosp, cosyCosp)

        let rad2deg: Float = 180 / .pi
        let o = SIMD3<Float>(yaw * rad2deg, pitch * rad2deg, roll * rad2deg)

        guiWorldPosition = p
        guiOrientationDegrees = o

        let newText = String(
            format: "World Coordinate:\nX: %.3f  Y: %.3f  Z: %.3f\nYaw: %.1f  Pitch: %.1f  Roll: %.1f",
            p.x, p.y, p.z, o.x, o.y, o.z
        )

        if newText != lastHeadText {
            lastHeadText = newText
            let mesh = MeshResource.generateText(
                newText,
                extrusionDepth: 0.001,
                font: .systemFont(ofSize: 0.03, weight: .semibold),
                containerFrame: .zero,
                alignment: .center,
                lineBreakMode: .byTruncatingTail
            )
            headEntity.model = ModelComponent(mesh: mesh, materials: [UnlitMaterial(color: .white)])
        }
        #endif
    }
}

#Preview(immersionStyle: .full) {
    ImmersiveView()
        .environment(AppModel())
        .environment(BodyTrackingModel())
}
