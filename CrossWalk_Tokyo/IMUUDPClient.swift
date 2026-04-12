import Foundation
import Network
import Combine
import simd
import QuartzCore

/// Parsed IMU packet: node ID + unit quaternion + angular velocity
struct IMUPacket {
    let nodeID: UInt8
    let quat: simd_quatf
    let angularVelocity: SIMD3<Float>
}

/// Bidirectional UDP client for IMU communication with nRF5340 devices via RPi relay.
/// Receives 9-byte quaternion packets on the local port and sends motor/IMU commands to the RPi.
class IMUUDPClient: ObservableObject {

    private var connections: [String: NWConnection] = [:]
    private var listener: NWListener?
    private var connectionReady: [String: Bool] = [:]
    private var pendingMessages: [String: [Data]] = [:]
    private let networkQueue = DispatchQueue(label: "IMUUDPClient.NetworkQueue")
    let deviceConnections: [String]

    // Keyed by nodeID string (e.g. "1", "2", ...)
    @Published var orientations: [String: simd_quatf] = [:]
    @Published var angularVelocities: [String: SIMD3<Float>] = [:]
    @Published var packetTimestamps: [String: CFTimeInterval] = [:]

    /// Previous quaternion per node for temporal sign continuity
    private var previousQuats: [String: simd_quatf] = [:]

    init(deviceConnections: [String], remotePort: UInt16, localPort: UInt16) {
        self.deviceConnections = deviceConnections
        self.listener = nil
        self.connections = [:]
        for host in deviceConnections {
            setupConnection(to: host, port: remotePort)
        }
        setupListener(on: localPort)
    }

    private func setupConnection(to host: String, port: UInt16) {
        let nwHost: NWEndpoint.Host
        if let ipv6Address = IPv6Address(host) {
            nwHost = .ipv6(ipv6Address)
        } else if let ipv4Address = IPv4Address(host) {
            nwHost = .ipv4(ipv4Address)
        } else {
            print("[IMUUDPClient] Invalid IP address: \(host)")
            return
        }

        let endpoint = NWEndpoint.hostPort(host: nwHost, port: .init(rawValue: port)!)
        let connection = NWConnection(to: endpoint, using: .udp)
        connections[host] = connection
        connectionReady[host] = false
        pendingMessages[host] = []
        connection.stateUpdateHandler = { state in
            print("[IMUUDPClient] Connection to \(host) state: \(state)")
            switch state {
            case .ready:
                self.connectionReady[host] = true
                self.flushPendingMessages(for: host)
            case .cancelled, .failed(_), .waiting(_):
                self.connectionReady[host] = false
            default:
                break
            }
        }
        connection.start(queue: networkQueue)
        print("[IMUUDPClient] Outbound connection set up to \(host):\(port)")
    }

    private func setupListener(on port: UInt16) {
        do {
            let params = NWParameters.udp
            params.allowLocalEndpointReuse = true
            params.acceptLocalOnly = false

            if let udpOptions = params.defaultProtocolStack.internetProtocol as? NWProtocolIP.Options {
                udpOptions.version = .any
            }
            listener = try NWListener(using: params, on: NWEndpoint.Port(rawValue: port)!)

            listener?.newConnectionHandler = { [weak self] newConnection in
                print("[IMUUDPClient] New inbound connection from: \(newConnection.endpoint)")
                newConnection.start(queue: self?.networkQueue ?? .global())
                self?.receive(on: newConnection)
            }

            listener?.stateUpdateHandler = { state in
                switch state {
                case .ready:
                    print("[IMUUDPClient] Listener READY on port \(port)")
                case .failed(let err):
                    print("[IMUUDPClient] Listener FAILED: \(err)")
                case .cancelled:
                    print("[IMUUDPClient] Listener CANCELLED")
                case .waiting(let err):
                    print("[IMUUDPClient] Listener WAITING: \(err)")
                default:
                    print("[IMUUDPClient] Listener state: \(state)")
                }
            }

            listener?.start(queue: networkQueue)
        } catch {
            print("[IMUUDPClient] Failed to create listener on port \(port): \(error)")
        }
    }

    private func receive(on connection: NWConnection) {
        connection.receiveMessage { [weak self] (data, context, isComplete, error) in
            if let error = error {
                print("[IMUUDPClient] Receive error: \(error)")
            }

            guard let data = data, !data.isEmpty else {
                self?.receive(on: connection)
                return
            }

            let receiveTime = CACurrentMediaTime()

            // 9-byte AVP v5 format: node_id(1B) + qw,qx,qy,qz as int16 LE (8B)
            if data.count >= 9 {
                if let packet = self?.decodeAVPPacket(from: data) {
                    let key = "\(packet.nodeID)"
                    var q = packet.quat
                    if let prev = self?.previousQuats[key],
                       simd_dot(q.vector, prev.vector) < 0 {
                        q = simd_quatf(ix: -q.imag.x, iy: -q.imag.y, iz: -q.imag.z, r: -q.real)
                    }
                    self?.previousQuats[key] = q
                    DispatchQueue.main.async {
                        self?.orientations[key] = q
                        self?.angularVelocities[key] = .zero
                        self?.packetTimestamps[key] = receiveTime
                    }
                }
            } else {
                // Suppress logging for known RPi text responses
                if data.count < 9, let text = String(data: data, encoding: .utf8), text.allSatisfy({ $0.isLetter }) {
                    // Silent: RPi acknowledgment
                }
            }

            self?.receive(on: connection)
        }
    }

    func sendMessage(to host: String, message: String) {
        guard let connection = connections[host] else {
            print("[IMUUDPClient] sendMessage('\(message)') FAILED - no connection for \(host)")
            return
        }

        let data = message.data(using: .utf8)!
        networkQueue.async {
            if self.connectionReady[host] == true {
                self.sendData(data, on: connection, host: host, debugMessage: message)
            } else {
                self.pendingMessages[host, default: []].append(data)
                print("[IMUUDPClient] Queued '\(message)' for \(host) (connection not ready yet)")
            }
        }
    }

    private func flushPendingMessages(for host: String) {
        guard let connection = connections[host] else { return }
        let queued = pendingMessages[host] ?? []
        guard !queued.isEmpty else { return }

        pendingMessages[host] = []
        print("[IMUUDPClient] Flushing \(queued.count) queued message(s) to \(host)")
        for data in queued {
            let text = String(data: data, encoding: .utf8) ?? "<binary>"
            sendData(data, on: connection, host: host, debugMessage: text)
        }
    }

    private func sendData(_ data: Data, on connection: NWConnection, host: String, debugMessage: String) {
        connection.send(content: data, completion: .contentProcessed({ error in
            if let error = error {
                print("[IMUUDPClient] Send '\(debugMessage)' -> \(host) FAILED: \(error)")
            }
        }))
    }

    func decodeAVPPacket(from data: Data) -> IMUPacket? {
        guard data.count >= 9 else { return nil }

        let nodeID = data[0]
        let scale: Float = 32767.0

        func int16At(_ offset: Int) -> Float {
            let raw = data.withUnsafeBytes { ptr in
                Int16(littleEndian: ptr.loadUnaligned(fromByteOffset: offset, as: Int16.self))
            }
            return Float(raw) / scale
        }

        let qw = int16At(1)
        let qx = int16At(3)
        let qy = int16At(5)
        let qz = int16At(7)

        let quat = simd_quatf(ix: qx, iy: qy, iz: qz, r: qw)
        return IMUPacket(nodeID: nodeID, quat: quat, angularVelocity: .zero)
    }

    deinit {
        listener?.cancel()
        for (_, connection) in connections {
            connection.cancel()
        }
    }
}
