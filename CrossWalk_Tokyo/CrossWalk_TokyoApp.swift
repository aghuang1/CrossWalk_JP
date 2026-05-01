//
//  CrossWalk_TokyoApp.swift
//  CrossWalk_Tokyo
//
//  Created by JL on 10/12/25.
//

import SwiftUI

@main
struct CrossWalk_TokyoApp: App {		

    @State private var appModel = AppModel()
    @State private var bodyModel = BodyTrackingModel()

    var body: some Scene {
        // Immersive space listed first + declared as the app's default scene role
        // (see Info.plist -> UISceneSessionRoleImmersiveSpaceApplication) so the
        // app launches directly into the Tokyo scene. This gives it the full
        // walking envelope Apple grants to immersive-first apps like Ping Pong
        // Club, instead of the conservative boundary applied to window apps that
        // only optionally open an immersive space.
        ImmersiveSpace(id: appModel.immersiveSpaceID) {
            ImmersiveView()
                .environment(appModel)
                .environment(bodyModel)
                .onAppear {
                    appModel.immersiveSpaceState = .open
                }
                .onDisappear {
                    appModel.immersiveSpaceState = .closed
                }
        }
        .immersionStyle(selection: .constant(.full), in: .full)

        WindowGroup {
            ContentView()
                .environment(appModel)
        }

        // Calibration control panel (separate window)
        WindowGroup(id: "calibrationPanel") {
            CalibrationControlPanel()
                .environment(bodyModel)
        }
        .windowStyle(.plain)
    }
}
