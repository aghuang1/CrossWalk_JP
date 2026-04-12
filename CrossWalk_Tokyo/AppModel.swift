//
//  AppModel.swift
//  CrossWalk_Tokyo
//
//  Created by JLiu on 10/12/25.
//

import SwiftUI

/// Maintains app-wide state (immersive space lifecycle).
/// ARKit session and world tracking are now managed by BodyTrackingModel.
/// ESP32 directional haptics have been replaced by ExtendedTouch per-limb motor system.
@MainActor
@Observable
class AppModel {
    let immersiveSpaceID = "ImmersiveSpace"
    enum ImmersiveSpaceState {
        case closed
        case inTransition
        case open
    }
    var immersiveSpaceState = ImmersiveSpaceState.closed
}
