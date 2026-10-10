import Foundation

enum RobotError: LocalizedError {
    case unauthorized
    case server(Int)
    case transport(String)

    var errorDescription: String? {
        switch self {
        case .unauthorized: return "The robot rejected the access token."
        case .server(let code): return "The robot returned an unexpected status (\(code))."
        case .transport(let msg): return "Can't reach the robot: \(msg)"
        }
    }
}

/// Talks to the robot bridge (see API.md). One request at a time per call; short timeout so the UI never hangs.
actor RobotClient {
    private var baseURL: URL
    private var token: String
    private let session: URLSession
    private let decoder = JSONDecoder()
    private let encoder = JSONEncoder()

    init(baseURL: URL, token: String) {
        self.baseURL = baseURL
        self.token = token
        let cfg = URLSessionConfiguration.default
        cfg.timeoutIntervalForRequest = 3
        cfg.waitsForConnectivity = false
        self.session = URLSession(configuration: cfg)
        decoder.keyDecodingStrategy = .convertFromSnakeCase
        encoder.keyEncodingStrategy = .convertToSnakeCase
    }

    func configure(baseURL: URL, token: String) {
        self.baseURL = baseURL
        self.token = token
    }

    // MARK: reads
    func state() async throws -> RobotState { try await get(["v1", "state"]) }
    func map() async throws -> RobotMap { try await get(["v1", "map"]) }

    // MARK: commands
    func summon(at p: XY) async throws -> CommandResult { try await post(["v1", "summon"], body: p) }
    func returnToStart() async throws -> CommandResult { try await post(["v1", "return"], body: Empty()) }
    func learnRoom() async throws -> CommandResult { try await post(["v1", "learn"], body: Empty()) }
    /// A SOFT stop. It is not a safety device; the hardware E-stop is.
    func softStop() async throws -> CommandResult { try await post(["v1", "stop"], body: Empty()) }

    // MARK: plumbing
    private struct Empty: Encodable {}

    private func url(_ parts: [String]) -> URL {
        parts.reduce(baseURL) { $0.appendingPathComponent($1) }
    }

    private func send(_ parts: [String], method: String, body: Data?) async throws -> (Data, Int) {
        var req = URLRequest(url: url(parts))
        req.httpMethod = method
        req.setValue("Bearer \(token)", forHTTPHeaderField: "Authorization")
        if let body {
            req.httpBody = body
            req.setValue("application/json", forHTTPHeaderField: "Content-Type")
        }
        do {
            let (data, resp) = try await session.data(for: req)
            let code = (resp as? HTTPURLResponse)?.statusCode ?? 0
            if code == 401 { throw RobotError.unauthorized }
            return (data, code)
        } catch let e as RobotError {
            throw e
        } catch {
            throw RobotError.transport(error.localizedDescription)
        }
    }

    private func get<T: Decodable>(_ parts: [String]) async throws -> T {
        let (data, code) = try await send(parts, method: "GET", body: nil)
        guard code == 200 else { throw RobotError.server(code) }
        return try decoder.decode(T.self, from: data)
    }

    /// 200 = accepted, 409/422 = the robot understood but refused (for example "cord still plugged in"): both decode.
    private func post<B: Encodable>(_ parts: [String], body: B) async throws -> CommandResult {
        let (data, code) = try await send(parts, method: "POST", body: try encoder.encode(body))
        guard code == 200 || code == 409 || code == 422 else { throw RobotError.server(code) }
        return try decoder.decode(CommandResult.self, from: data)
    }
}
