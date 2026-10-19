import Foundation

// JSON from the robot bridge uses snake_case; RobotClient decodes with .convertFromSnakeCase,
// so robot_pct -> robotPct, desk_height_cm -> deskHeightCm, and so on. Units: meters and radians.

struct XY: Codable, Equatable { var x: Double; var y: Double }
struct Pose: Codable, Equatable { var x: Double; var y: Double; var theta: Double }
struct Battery: Codable, Equatable { var robotPct: Double; var upsPct: Double }
struct Cord: Codable, Equatable { var plugged: Bool; var interlockOn: Bool; var breakaways: Int }
struct Person: Codable, Equatable { var x: Double; var y: Double; var settled: Bool }

struct RobotState: Codable, Equatable {
    var time: Double
    var mode: String          // "user" | "home" | "tour"
    var status: String        // human-readable, show as-is
    var severity: String      // "ok" | "warn" | "bad" | "info"
    var active: Bool          // a trip is in progress
    var estop: Bool           // READ-ONLY. The real E-stop is hardware and can never be released from the app.
    var pose: Pose
    var user: XY
    var battery: Battery
    var charging: Bool
    var cord: Cord
    var dockType: String      // "manual" | "auto"
    var deskHeightCm: Int
    var alert: Bool          // true while the robot wants attention (e.g. "unplug me")
    var exploredPct: Int
    var rememberedCells: Int
    var people: [Person]
}

struct RobotMap: Codable, Equatable {
    var widthM: Double
    var heightM: Double
    var cellM: Double
    var gw: Int
    var gh: Int
    var rows: [String]        // '.' never seen, '_' free, '#' remembered obstacle, '+' busy with people
}

struct CommandResult: Codable {
    var accepted: Bool
    var status: String
    var severity: String?
}
