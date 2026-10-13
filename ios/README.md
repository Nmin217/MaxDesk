# iPhone app

SwiftUI, iOS 17+. See ../README.md for the overall picture and ../docs/API.md for the contract with the robot.

1. In Xcode: File > New > Project > iOS App (SwiftUI), name it SummonDesk, deployment target iOS 17.
2. Delete the generated `SummonDeskApp.swift` and `ContentView.swift`, then drag in everything from `SummonDesk/`.
3. Add the keys in `Info-additions.txt` to the target's Info tab (local network permission text, Bonjour service,
   and "Allow Local Networking" for plain HTTP to local addresses only; never "Allow Arbitrary Loads").
4. Run the bridge on a Mac (`node ../bridge/bridge_sim.js`). In the iOS Simulator use address `127.0.0.1:8787` and token
   `dev-token`; on a real iPhone use the Mac's LAN address and allow Local Network access when asked.

NOT COMPILED. The structure and API calls are standard, but expect small compile fixes. The Stop button is a software
stop; the app can never release the hardware E-stop.
