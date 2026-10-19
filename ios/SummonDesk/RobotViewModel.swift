import SwiftUI
import UserNotifications

@MainActor
final class RobotViewModel: ObservableObject {
    enum Link: Equatable {
        case connecting
        case online
        case offline(String)
    }

    @Published var state: RobotState?
    @Published var map: RobotMap?
    @Published var link: Link = .connecting
    @Published var target: XY?          // where you tapped, in meters
    @Published var banner: String?      // last command result or error

    private var client: RobotClient
    private var pollTask: Task<Void, Never>?

    init() {
        let host = UserDefaults.standard.string(forKey: "robotHost") ?? "127.0.0.1:8787"
        let token = Keychain.load(account: "robotToken") ?? "dev-token"
        client = RobotClient(baseURL: Self.makeURL(host), token: token)
    }

    // MARK: settings
    var host: String { UserDefaults.standard.string(forKey: "robotHost") ?? "127.0.0.1:8787" }
    var token: String { Keychain.load(account: "robotToken") ?? "dev-token" }

    func saveSettings(host: String, token: String) {
        UserDefaults.standard.set(host, forKey: "robotHost")
        Keychain.save(token, account: "robotToken")
        let url = Self.makeURL(host)
        Task { await client.configure(baseURL: url, token: token) }
        link = .connecting
    }

    /// "192.168.1.50:8787" or "desk.local:8787" -> http://... (plain HTTP is allowed to local addresses only; see README)
    private static func makeURL(_ host: String) -> URL {
        let h = host.hasPrefix("http") ? host : "http://\(host)"
        return URL(string: h) ?? URL(string: "http://127.0.0.1:8787")!
    }

    // MARK: polling
    func start() {
        pollTask?.cancel()
        pollTask = Task { [weak self] in
            var tick = 0
            while !Task.isCancelled {
                guard let self else { return }
                await self.refresh(includeMap: tick % 10 == 0)
                tick += 1
                try? await Task.sleep(nanoseconds: 500_000_000)
            }
        }
    }

    func stop() { pollTask?.cancel() }

    private func refresh(includeMap: Bool) async {
        do {
            let s = try await client.state()
            let old = state
            state = s
            link = .online
            notifyIfNeeded(old: old, new: s)
            if includeMap || map == nil { map = try await client.map() }
        } catch {
            link = .offline(error.localizedDescription)
        }
    }

    // MARK: choosing where you are
    /// location is a point inside a view of the given size that shows the whole room.
    func setTarget(location: CGPoint, in size: CGSize) {
        guard let m = map, size.width > 0, size.height > 0 else { return }
        target = XY(x: Double(location.x / size.width) * m.widthM,
                    y: Double(location.y / size.height) * m.heightM)
    }

    // MARK: commands
    func summon() async {
        guard let t = target else { banner = "Tap the map where you are first."; return }
        await run { try await self.client.summon(at: t) }
    }
    func returnToStart() async { await run { try await self.client.returnToStart() } }
    func learnRoom() async { await run { try await self.client.learnRoom() } }
    func softStop() async { await run { try await self.client.softStop() } }

    private func run(_ op: @escaping () async throws -> CommandResult) async {
        do {
            let r = try await op()
            banner = r.accepted ? r.status : "Refused: \(r.status)"
        } catch {
            banner = error.localizedDescription
        }
        await refresh(includeMap: false)
    }

    // MARK: notifications (local only; see README for background alerts)
    func requestNotificationPermission() {
        UNUserNotificationCenter.current().requestAuthorization(options: [.alert, .sound]) { _, _ in }
    }

    private func notifyIfNeeded(old: RobotState?, new: RobotState) {
        guard let old else { return }
        var message: String?
        if !old.alert && new.alert {
            message = new.status                                    // e.g. "Cord is still plugged in: unplug it first"
        } else if new.severity == "bad" && old.status != new.status {
            message = new.status
        } else if old.battery.robotPct >= 25 && new.battery.robotPct < 25 {
            message = "Robot battery is below 25%."
        } else if old.battery.upsPct >= 20 && new.battery.upsPct < 20 {
            message = "UPS battery is below 20%."
        }
        guard let message else { return }
        let content = UNMutableNotificationContent()
        content.title = "Summon Desk"
        content.body = message
        content.sound = .default
        UNUserNotificationCenter.current().add(
            UNNotificationRequest(identifier: UUID().uuidString, content: content, trigger: nil))
    }
}
