import SwiftUI

struct SettingsView: View {
    @ObservedObject var vm: RobotViewModel
    @Environment(\.dismiss) private var dismiss
    @State private var host = ""
    @State private var token = ""

    var body: some View {
        NavigationStack {
            Form {
                Section("Robot") {
                    TextField("Address, for example 192.168.1.50:8787", text: $host)
                        .textInputAutocapitalization(.never)
                        .autocorrectionDisabled()
                        .keyboardType(.URL)
                    SecureField("Access token", text: $token)
                }
                Section {
                    Text("Your iPhone must be on the same network as the robot. The token is stored in the iOS Keychain.")
                        .font(.footnote)
                        .foregroundStyle(.secondary)
                }
            }
            .navigationTitle("Settings")
            .toolbar {
                ToolbarItem(placement: .confirmationAction) {
                    Button("Save") {
                        vm.saveSettings(host: host, token: token)
                        dismiss()
                    }
                }
            }
        }
        .onAppear {
            host = vm.host
            token = vm.token
        }
    }
}
