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
        WindowGroup {
            ContentView()
                .environment(appModel)
        }

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

        // Calibration control panel (separate window)
        WindowGroup(id: "calibrationPanel") {
            CalibrationControlPanel()
                .environment(bodyModel)
        }
        .windowStyle(.plain)
    }
}
