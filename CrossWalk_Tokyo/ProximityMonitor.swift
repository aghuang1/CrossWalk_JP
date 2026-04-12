import ARKit
import RealityKit
import Combine
import simd
import QuartzCore

/// Which half of the headset trigger volume detected a collision
enum HeadsetZone {
    case upper
    case lower
}

/// Monitors proximity to obstacles using trigger volumes.
/// Split into UPPER and LOWER halves based on headset height.
/// Adapted for CrossWalk VR: detects collisions with specific obstacle entities
/// (SpawnCube, characters) instead of scene reconstruction meshes.
@MainActor
class ProximityMonitor: ObservableObject {

    // MARK: - Published Properties
    @Published var isUpperObstacleNearby: Bool = false
    @Published var isLowerObstacleNearby: Bool = false
    @Published var isObstacleNearby: Bool = false
    @Published var nearestObstacleDistance: Float? = nil

    // MARK: - Configuration
    var upperWidth: Float = 0.70
    var upperDepth: Float = 2.00
    var upperHeight: Float = 1.0
    var upperYOffset: Float = -0.2

    var lowerWidth: Float = 0.70
    var lowerDepth: Float = 2.00
    var lowerHeight: Float = 0.5
    var lowerYOffset: Float = -1.0

    private let checkInterval: TimeInterval = 0.1

    // MARK: - Private Properties
    private var upperTriggerEntity: Entity?
    private var lowerTriggerEntity: Entity?
    private var checkTimer: Timer?
    private weak var worldTracking: WorldTrackingProvider?
    private weak var contentEntity: Entity?
    private var isMonitoring: Bool = false

    nonisolated(unsafe) private var collisionBeganSubscription: (any Cancellable)?
    nonisolated(unsafe) private var collisionEndedSubscription: (any Cancellable)?

    // Track entity IDs currently colliding with each zone
    private var entitiesInUpperZone: Set<ObjectIdentifier> = []
    private var entitiesInLowerZone: Set<ObjectIdentifier> = []

    private var recentCollisions: Set<String> = []
    private let collisionDebounceTime: TimeInterval = 0.1

    // Debouncing for state changes
    private var pendingUpperStateChange: Bool?
    private var pendingLowerStateChange: Bool?
    private var upperStateChangeTimer: Timer?
    private var lowerStateChangeTimer: Timer?
    private let stateChangeDebounceStart: TimeInterval = 0.3
    private let stateChangeDebounceStop: TimeInterval = 1.0

    // Callbacks
    var onUpperZoneStateChanged: ((Bool) -> Void)?
    var onLowerZoneStateChanged: ((Bool) -> Void)?
    var onProximityStateChanged: ((Bool) -> Void)?

    // MARK: - Obstacle entity names to detect
    /// Entity names that should trigger proximity detection
    private let obstacleEntityNames: Set<String> = ["SpawnCube", "Cube_0", "Cube_1", "Cube_2", "Man1"]

    init() {
        print("ProximityMonitor: Initialized (upper/lower independent trigger volumes)")
    }

    // MARK: - Public Methods

    func startMonitoring(
        worldTracking: WorldTrackingProvider,
        contentEntity: Entity
    ) {
        guard !isMonitoring else { return }

        self.worldTracking = worldTracking
        self.contentEntity = contentEntity
        self.isMonitoring = true

        setupTriggerVolumes()
        setupCollisionSubscriptions()

        checkTimer = Timer.scheduledTimer(
            withTimeInterval: checkInterval,
            repeats: true
        ) { [weak self] _ in
            Task { @MainActor in
                await self?.updateTriggerPosition()
            }
        }
    }

    func stopMonitoring() {
        guard isMonitoring else { return }

        checkTimer?.invalidate()
        checkTimer = nil
        upperStateChangeTimer?.invalidate()
        upperStateChangeTimer = nil
        lowerStateChangeTimer?.invalidate()
        lowerStateChangeTimer = nil
        collisionBeganSubscription?.cancel()
        collisionEndedSubscription?.cancel()
        collisionBeganSubscription = nil
        collisionEndedSubscription = nil

        upperTriggerEntity?.removeFromParent()
        upperTriggerEntity = nil
        lowerTriggerEntity?.removeFromParent()
        lowerTriggerEntity = nil

        entitiesInUpperZone.removeAll()
        entitiesInLowerZone.removeAll()
        recentCollisions.removeAll()

        isMonitoring = false
    }

    func rebuildTriggerVolumes() {
        guard let upper = upperTriggerEntity, let lower = lowerTriggerEntity else { return }

        let upperShape = ShapeResource.generateBox(width: upperWidth, height: upperHeight, depth: upperDepth)
        upper.components.set(CollisionComponent(shapes: [upperShape], filter: CollisionFilter(group: .skeleton, mask: .obstacle)))

        let lowerShape = ShapeResource.generateBox(width: lowerWidth, height: lowerHeight, depth: lowerDepth)
        lower.components.set(CollisionComponent(shapes: [lowerShape], filter: CollisionFilter(group: .skeleton, mask: .obstacle)))

        entitiesInUpperZone.removeAll()
        entitiesInLowerZone.removeAll()

        let upperWasActive = isUpperObstacleNearby
        let lowerWasActive = isLowerObstacleNearby
        isUpperObstacleNearby = false
        isLowerObstacleNearby = false
        isObstacleNearby = false

        if upperWasActive { onUpperZoneStateChanged?(false) }
        if lowerWasActive { onLowerZoneStateChanged?(false) }
    }

    // MARK: - Private Methods

    /// Checks if an entity name matches a known obstacle
    private func isObstacleEntity(_ entity: Entity) -> Bool {
        if obstacleEntityNames.contains(entity.name) { return true }
        if entity.name.contains("Cube") { return true }
        if entity.name.contains("Man") { return true }
        return false
    }

    private func setupTriggerVolumes() {
        guard let contentEntity = contentEntity else { return }

        let upperShape = ShapeResource.generateBox(width: upperWidth, height: upperHeight, depth: upperDepth)
        let upperTrigger = TriggerVolume(shape: upperShape)
        upperTrigger.name = "ProximityTrigger_Upper"

        var upperCollision = upperTrigger.collision ?? CollisionComponent(shapes: [upperShape])
        upperCollision.filter = CollisionFilter(group: .skeleton, mask: .obstacle)
        upperTrigger.components.set(upperCollision)

        contentEntity.addChild(upperTrigger)
        self.upperTriggerEntity = upperTrigger

        let lowerShape = ShapeResource.generateBox(width: lowerWidth, height: lowerHeight, depth: lowerDepth)
        let lowerTrigger = TriggerVolume(shape: lowerShape)
        lowerTrigger.name = "ProximityTrigger_Lower"

        var lowerCollision = lowerTrigger.collision ?? CollisionComponent(shapes: [lowerShape])
        lowerCollision.filter = CollisionFilter(group: .skeleton, mask: .obstacle)
        lowerTrigger.components.set(lowerCollision)

        contentEntity.addChild(lowerTrigger)
        self.lowerTriggerEntity = lowerTrigger
    }

    private func setupCollisionSubscriptions() {
        guard let scene = contentEntity?.scene else { return }

        collisionBeganSubscription = scene.subscribe(to: CollisionEvents.Began.self) { [weak self] event in
            self?.handleCollisionBegan(event)
        }

        collisionEndedSubscription = scene.subscribe(to: CollisionEvents.Ended.self) { [weak self] event in
            self?.handleCollisionEnded(event)
        }
    }

    private func handleCollisionBegan(_ event: CollisionEvents.Began) {
        let entityA = event.entityA
        let entityB = event.entityB

        let triggerEntity: Entity?
        let obstacleEntity: Entity?

        if entityA.name.hasPrefix("ProximityTrigger_") && isObstacleEntity(entityB) {
            triggerEntity = entityA
            obstacleEntity = entityB
        } else if entityB.name.hasPrefix("ProximityTrigger_") && isObstacleEntity(entityA) {
            triggerEntity = entityB
            obstacleEntity = entityA
        } else {
            return
        }

        guard let trigger = triggerEntity, let obstacle = obstacleEntity else { return }

        let zone: HeadsetZone
        if trigger.name == "ProximityTrigger_Upper" {
            zone = .upper
        } else if trigger.name == "ProximityTrigger_Lower" {
            zone = .lower
        } else {
            return
        }

        let collisionKey = "\(trigger.name)-\(obstacle.name)"
        guard !recentCollisions.contains(collisionKey) else { return }
        recentCollisions.insert(collisionKey)

        Task { @MainActor in
            try? await Task.sleep(nanoseconds: UInt64(collisionDebounceTime * 1_000_000_000))
            self.recentCollisions.remove(collisionKey)
        }

        let entityID = ObjectIdentifier(obstacle)
        switch zone {
        case .upper:
            entitiesInUpperZone.insert(entityID)
        case .lower:
            entitiesInLowerZone.insert(entityID)
        }

        evaluateProximityState(zone: zone)
    }

    private func handleCollisionEnded(_ event: CollisionEvents.Ended) {
        let entityA = event.entityA
        let entityB = event.entityB

        let triggerEntity: Entity?
        let obstacleEntity: Entity?

        if entityA.name.hasPrefix("ProximityTrigger_") && isObstacleEntity(entityB) {
            triggerEntity = entityA
            obstacleEntity = entityB
        } else if entityB.name.hasPrefix("ProximityTrigger_") && isObstacleEntity(entityA) {
            triggerEntity = entityB
            obstacleEntity = entityA
        } else {
            return
        }

        guard let trigger = triggerEntity, let obstacle = obstacleEntity else { return }

        let zone: HeadsetZone
        if trigger.name == "ProximityTrigger_Upper" {
            zone = .upper
        } else if trigger.name == "ProximityTrigger_Lower" {
            zone = .lower
        } else {
            return
        }

        let collisionKey = "\(trigger.name)-\(obstacle.name)-end"
        guard !recentCollisions.contains(collisionKey) else { return }
        recentCollisions.insert(collisionKey)

        Task { @MainActor in
            try? await Task.sleep(nanoseconds: UInt64(collisionDebounceTime * 1_000_000_000))
            self.recentCollisions.remove(collisionKey)
        }

        let entityID = ObjectIdentifier(obstacle)
        switch zone {
        case .upper:
            _ = entitiesInUpperZone.remove(entityID)
        case .lower:
            _ = entitiesInLowerZone.remove(entityID)
        }

        evaluateProximityState(zone: zone)
    }

    private func evaluateProximityState(zone: HeadsetZone) {
        let entities: Set<ObjectIdentifier>
        let currentState: Bool

        switch zone {
        case .upper:
            entities = entitiesInUpperZone
            currentState = isUpperObstacleNearby
        case .lower:
            entities = entitiesInLowerZone
            currentState = isLowerObstacleNearby
        }

        let desiredState = !entities.isEmpty

        if desiredState != currentState {
            let debounceTime = desiredState ? stateChangeDebounceStart : stateChangeDebounceStop

            switch zone {
            case .upper:
                upperStateChangeTimer?.invalidate()
                pendingUpperStateChange = desiredState
                upperStateChangeTimer = Timer.scheduledTimer(withTimeInterval: debounceTime, repeats: false) { [weak self] _ in
                    guard let self = self else { return }
                    let finalState = !self.entitiesInUpperZone.isEmpty
                    if finalState != self.isUpperObstacleNearby {
                        self.isUpperObstacleNearby = finalState
                        self.isObstacleNearby = self.isUpperObstacleNearby || self.isLowerObstacleNearby
                        self.onUpperZoneStateChanged?(finalState)
                        self.onProximityStateChanged?(self.isObstacleNearby)
                    }
                    self.pendingUpperStateChange = nil
                }

            case .lower:
                lowerStateChangeTimer?.invalidate()
                pendingLowerStateChange = desiredState
                lowerStateChangeTimer = Timer.scheduledTimer(withTimeInterval: debounceTime, repeats: false) { [weak self] _ in
                    guard let self = self else { return }
                    let finalState = !self.entitiesInLowerZone.isEmpty
                    if finalState != self.isLowerObstacleNearby {
                        self.isLowerObstacleNearby = finalState
                        self.isObstacleNearby = self.isUpperObstacleNearby || self.isLowerObstacleNearby
                        self.onLowerZoneStateChanged?(finalState)
                        self.onProximityStateChanged?(self.isObstacleNearby)
                    }
                    self.pendingLowerStateChange = nil
                }
            }
        }
    }

    private func updateTriggerPosition() async {
        guard let worldTracking = worldTracking,
              let upperTrigger = upperTriggerEntity,
              let lowerTrigger = lowerTriggerEntity else {
            return
        }

        guard let deviceAnchor = worldTracking.queryDeviceAnchor(atTimestamp: CACurrentMediaTime()) else {
            return
        }

        let headsetTransform = Transform(matrix: deviceAnchor.originFromAnchorTransform)
        let headsetPosition = headsetTransform.translation

        let forward = SIMD3<Float>(
            -deviceAnchor.originFromAnchorTransform.columns.2.x,
            0,
            -deviceAnchor.originFromAnchorTransform.columns.2.z
        )
        let yaw = atan2(forward.x, -forward.z)
        let yawQuat = simd_quatf(angle: -yaw, axis: SIMD3<Float>(0, 1, 0))

        let upperCenterY = headsetPosition.y + upperYOffset
        let lowerCenterY = headsetPosition.y + lowerYOffset

        upperTrigger.position = SIMD3<Float>(headsetPosition.x, upperCenterY, headsetPosition.z)
        upperTrigger.orientation = yawQuat

        lowerTrigger.position = SIMD3<Float>(headsetPosition.x, lowerCenterY, headsetPosition.z)
        lowerTrigger.orientation = yawQuat
    }

    func getCurrentProximityStatus() async -> (isNearby: Bool, distance: Float?) {
        return (isObstacleNearby, nearestObstacleDistance)
    }
}
