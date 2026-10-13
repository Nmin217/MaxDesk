import SwiftUI

struct ContentView: View {
    @StateObject private var vm = RobotViewModel()
    @State private var showSettings = false

    var body: some View {
        NavigationStack {
            ScrollView {
                VStack(spacing: 16) {
                    ConnectionBanner(link: vm.link)
                    if let s = vm.state { StatusCard(state: s) }
                    MapCard(vm: vm)
                    CommandButtons(vm: vm)
                    if let b = vm.banner {
                        Text(b).font(.footnote).foregroundStyle(.secondary)
                    }
                }
                .padding()
            }
            .navigationTitle("Summon Desk")
            .toolbar {
                ToolbarItem(placement: .topBarTrailing) {
                    Button { showSettings = true } label: { Image(systemName: "gearshape") }
                }
            }
            .sheet(isPresented: $showSettings) { SettingsView(vm: vm) }
        }
        .task {
            vm.requestNotificationPermission()
            vm.start()
        }
    }
}

// MARK: - connection

struct ConnectionBanner: View {
    let link: RobotViewModel.Link
    var body: some View {
        switch link {
        case .online:
            EmptyView()
        case .connecting:
            HStack { ProgressView(); Text("Connecting to the robot…") }
                .font(.footnote).foregroundStyle(.secondary)
        case .offline(let message):
            Text(message)
                .font(.footnote)
                .foregroundStyle(.orange)
                .frame(maxWidth: .infinity, alignment: .leading)
        }
    }
}

// MARK: - status

private func color(for severity: String) -> Color {
    switch severity {
    case "bad": return .red
    case "warn": return .orange
    case "ok": return .green
    default: return .primary
    }
}

struct StatusCard: View {
    let state: RobotState
    var body: some View {
        VStack(alignment: .leading, spacing: 10) {
            Text(state.status)
                .font(.headline)
                .foregroundStyle(color(for: state.severity))
            if state.estop {
                Label("E-stop engaged: release it on the robot", systemImage: "exclamationmark.octagon.fill")
                    .foregroundStyle(.red)
            }
            BatteryRow(title: "Robot battery (12 V)", pct: state.battery.robotPct)
            BatteryRow(title: "UPS battery", pct: state.battery.upsPct)
            HStack {
                Label(state.cord.plugged ? "Cord plugged in" : "Cord unplugged",
                      systemImage: state.cord.plugged ? "powerplug.fill" : "powerplug")
                Spacer()
                if state.charging {
                    Label("Charging", systemImage: "bolt.fill").foregroundStyle(.green)
                }
            }
            .font(.subheadline)
            Text("Desk height \(state.deskHeightCm) cm · room explored \(state.exploredPct)%")
                .font(.footnote)
                .foregroundStyle(.secondary)
        }
        .padding()
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(Color(.secondarySystemBackground), in: RoundedRectangle(cornerRadius: 14))
    }
}

struct BatteryRow: View {
    let title: String
    let pct: Double
    var body: some View {
        VStack(alignment: .leading, spacing: 4) {
            HStack { Text(title); Spacer(); Text("\(Int(pct))%").monospacedDigit() }
                .font(.subheadline)
            ProgressView(value: pct, total: 100).tint(pct < 25 ? .red : .green)
        }
    }
}

// MARK: - map

struct MapCard: View {
    @ObservedObject var vm: RobotViewModel

    var body: some View {
        let aspect = vm.map.map { $0.widthM / $0.heightM } ?? 1.6
        VStack(alignment: .leading, spacing: 6) {
            Text("Tap the map where you are")
                .font(.subheadline)
                .foregroundStyle(.secondary)
            Canvas { ctx, size in draw(ctx, size) }
                .aspectRatio(aspect, contentMode: .fit)
                .background(Color(.secondarySystemBackground))
                .clipShape(RoundedRectangle(cornerRadius: 10))
                .overlay {
                    GeometryReader { geo in
                        Color.clear
                            .contentShape(Rectangle())
                            .gesture(SpatialTapGesture().onEnded { vm.setTarget(location: $0.location, in: geo.size) })
                    }
                }
        }
    }

    private func draw(_ ctx: GraphicsContext, _ size: CGSize) {
        guard let m = vm.map else { return }
        let cw = size.width / CGFloat(m.gw), ch = size.height / CGFloat(m.gh)
        for (y, row) in m.rows.enumerated() {
            for (x, c) in row.enumerated() {
                let fill: Color
                switch c {
                case "_": continue                              // seen and free
                case "#": fill = .gray                          // remembered obstacle
                case "+": fill = Color.orange.opacity(0.35)     // people are often here
                default: fill = Color.black.opacity(0.08)       // never seen
                }
                ctx.fill(Path(CGRect(x: CGFloat(x) * cw, y: CGFloat(y) * ch, width: cw + 0.5, height: ch + 0.5)),
                         with: .color(fill))
            }
        }
        let s = size.width / CGFloat(m.widthM)                  // pixels per meter
        if let st = vm.state {
            for p in st.people {
                let r: CGFloat = 0.15 * s
                ctx.fill(Path(ellipseIn: CGRect(x: CGFloat(p.x) * s - r, y: CGFloat(p.y) * s - r, width: 2 * r, height: 2 * r)),
                         with: .color(p.settled ? .orange : .pink))
            }
            var c = ctx                                         // the desk: 1.12 m x 0.72 m, drawn at its pose
            c.translateBy(x: CGFloat(st.pose.x) * s, y: CGFloat(st.pose.y) * s)
            c.rotate(by: .radians(st.pose.theta))
            let hw = 0.56 * s, hh = 0.36 * s
            c.fill(Path(CGRect(x: -hw, y: -hh, width: 2 * hw, height: 2 * hh)), with: .color(.brown))
            var arrow = Path()                                  // forward = the back of the monitor
            arrow.move(to: CGPoint(x: 0, y: -hh))
            arrow.addLine(to: CGPoint(x: -0.1 * s, y: -hh + 0.15 * s))
            arrow.addLine(to: CGPoint(x: 0.1 * s, y: -hh + 0.15 * s))
            arrow.closeSubpath()
            c.fill(arrow, with: .color(.white))
        }
        if let t = vm.target {                                  // where you tapped
            let r: CGFloat = 0.3 * s
            ctx.stroke(Path(ellipseIn: CGRect(x: CGFloat(t.x) * s - r, y: CGFloat(t.y) * s - r, width: 2 * r, height: 2 * r)),
                       with: .color(.blue), lineWidth: 3)
        }
    }
}

// MARK: - commands

struct CommandButtons: View {
    @ObservedObject var vm: RobotViewModel

    var body: some View {
        VStack(spacing: 10) {
            Button { Task { await vm.summon() } } label: {
                Label("Come to me", systemImage: "location.fill").frame(maxWidth: .infinity)
            }
            .buttonStyle(.borderedProminent)
            .controlSize(.large)

            HStack(spacing: 10) {
                Button { Task { await vm.returnToStart() } } label: {
                    Label("Return to start", systemImage: "house").frame(maxWidth: .infinity)
                }
                Button { Task { await vm.learnRoom() } } label: {
                    Label("Learn the room", systemImage: "map").frame(maxWidth: .infinity)
                }
            }
            .buttonStyle(.bordered)

            Button(role: .destructive) { Task { await vm.softStop() } } label: {
                Label("Stop (software)", systemImage: "hand.raised.fill").frame(maxWidth: .infinity)
            }
            .buttonStyle(.bordered)

            Text("This stop is software only. For safety use the red E-stop on the robot; it cannot be released from this app.")
                .font(.caption)
                .foregroundStyle(.secondary)
        }
    }
}
