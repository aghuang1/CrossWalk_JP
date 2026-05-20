/*
 CalibrationControlPanel.swift
 CrossWalk_Tokyo

 Calibration UI and skeleton parameter controls for the body tracking system.
 Adapted from ExtendedTouch_AVP's CubeMeshInteraction.swift control panel views.
*/

import SwiftUI
import QuartzCore

// MARK: - Control Panel Window

struct CalibrationControlPanel: View {
    @Environment(BodyTrackingModel.self) var model

    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 16) {
                CalibrationStatusPanel()
                    .environment(model)

                HStack(alignment: .top, spacing: 24) {
                    ToggleButton(
                        label: model.isCalibrating ? "Calibrating..." : "Calibrate",
                        isActive: !model.isCalibrating,
                        activeColor: .purple,
                        inactiveColor: .gray,
                        systemImage: "dial.and.needle.fill"
                    ) {
                        guard !model.isCalibrating else { return }
                        model.startCalibration()
                    }

                    // Start / Restart the car simulation run. Starts the timer
                    // and begins counting spawned cars + distinct contacts;
                    // pressing again mid-run or post-run resets all stats.
                    ToggleButton(
                        label: model.isRunActive ? "Restart Run" : "Start Run",
                        isActive: true,
                        activeColor: .blue,
                        inactiveColor: .blue,
                        systemImage: "play.fill"
                    ) {
                        model.startRun()
                    }

                    // Cancel an in-progress run without recording a victory.
                    // Only surfaced while `isRunActive` so post-run / pre-run
                    // states don't show a meaningless Stop button. Tearing
                    // down the course is handled by ImmersiveView's run-state
                    // observer reacting to `runStartTime` going nil.
                    if model.isRunActive {
                        ToggleButton(
                            label: "Stop Run",
                            isActive: true,
                            activeColor: .red,
                            inactiveColor: .red,
                            systemImage: "stop.fill"
                        ) {
                            model.stopRun()
                        }
                    }

                    // Show/hide the immersive Tokyo environment + skydome.
                    // Useful while iterating on body tracking / haptics so
                    // the city geometry doesn't crowd the debug view.
                    @Bindable var bindable = model
                    ToggleButton(
                        label: model.showVREnvironment ? "Env Visible" : "Env Hidden",
                        isActive: model.showVREnvironment,
                        activeColor: .green,
                        inactiveColor: .gray,
                        systemImage: "mountain.2.fill"
                    ) {
                        bindable.showVREnvironment.toggle()
                    }

                    Spacer()
                }

                if model.isCalibrated {
                    ControlOverlayPanel()
                        .environment(model)
                }
            }
            .padding(24)
        }
        .glassBackgroundEffect()
    }
}

// MARK: - Calibration Status Panel

struct CalibrationStatusPanel: View {
    @Environment(BodyTrackingModel.self) var model

    var body: some View {
        VStack(alignment: .leading, spacing: 8) {
            HStack(spacing: 10) {
                Image(systemName: model.isCalibrating ? "hourglass" : "checkmark.seal")
                    .font(.system(size: 18, weight: .semibold))
                    .foregroundStyle(.white)

                Text(model.calibrationStatusTitle)
                    .font(.system(size: 20, weight: .bold))
                    .foregroundStyle(.white)

                if let countdown = model.calibrationCountdownSeconds {
                    Text("\(countdown)s")
                        .font(.system(size: 18, weight: .bold, design: .rounded))
                        .foregroundStyle(.yellow)
                        .padding(.horizontal, 10)
                        .padding(.vertical, 4)
                        .background(
                            Capsule(style: .continuous)
                                .fill(.black.opacity(0.35))
                        )
                }
            }

            Text(model.calibrationStatusDetail)
                .font(.system(size: 15, weight: .medium))
                .foregroundStyle(.white.opacity(0.9))
                .lineLimit(nil)

            // Live IMU data indicator. Re-evaluates twice per second via
            // TimelineView so the freshness check (last packet within 1 s)
            // stays accurate even when no observable state is churning.
            // Confirms quaternion data is flowing before the operator
            // presses Calibrate.
            IMUDataIndicator()
                .environment(model)

            if model.calibrationCountdownSeconds != nil {
                ProgressView(value: model.calibrationProgressFraction)
                    .progressViewStyle(.linear)
                    .tint(.yellow)
                    .frame(maxWidth: 360)
            }
        }
        .padding(16)
        .frame(maxWidth: 720, alignment: .leading)
        .background(
            RoundedRectangle(cornerRadius: 14, style: .continuous)
                .fill(.black.opacity(0.65))
        )
    }
}

// MARK: - IMU Data Indicator

/// Small live readout that confirms quaternion packets are arriving from the
/// IMUs. Counts segments whose most recent packet is within
/// `livenessWindow` seconds. The dot is green while at least one segment is
/// live, red otherwise. Polls twice per second via `TimelineView` so the
/// stale transition fires on time even if the publisher idles.
struct IMUDataIndicator: View {
    @Environment(BodyTrackingModel.self) var model

    private let livenessWindow: CFTimeInterval = 1.0

    var body: some View {
        TimelineView(.periodic(from: .now, by: 0.5)) { _ in
            let now = CACurrentMediaTime()
            let liveCount = model.lastSegmentPacketTime.values.reduce(into: 0) { acc, t in
                if now - t < livenessWindow { acc += 1 }
            }
            let isLive = liveCount > 0
            HStack(spacing: 8) {
                Circle()
                    .fill(isLive ? .green : .red)
                    .frame(width: 10, height: 10)
                Text(isLive
                     ? "Receiving IMU data — \(liveCount) segment\(liveCount == 1 ? "" : "s")"
                     : "No IMU data")
                    .font(.system(size: 14, weight: .semibold, design: .monospaced))
                    .foregroundStyle(.white)
            }
        }
    }
}

// MARK: - Control Overlay Panel

struct ControlOverlayPanel: View {
    @Environment(BodyTrackingModel.self) var model

    var body: some View {
        @Bindable var model = model

        VStack(alignment: .leading, spacing: 12) {
            // Toggle buttons row
            HStack(spacing: 12) {
                ToggleButton(
                    label: model.isMotorEnabled ? "Motors Enabled" : "Enable Motors",
                    isActive: model.isMotorEnabled,
                    activeColor: .green,
                    inactiveColor: .red,
                    systemImage: model.isMotorEnabled ? "bolt.fill" : "bolt.slash.fill"
                ) {
                    model.toggleMotorEnabled()
                }

                ToggleButton(
                    label: model.isIMUOverrideActive ? "IMU Paused" : "IMU Streaming",
                    isActive: !model.isIMUOverrideActive,
                    activeColor: .blue,
                    inactiveColor: .orange,
                    systemImage: model.isIMUOverrideActive ? "antenna.radiowaves.left.and.right.slash" : "antenna.radiowaves.left.and.right"
                ) {
                    model.toggleIMUOverride()
                }
            }

            // Parameter controls
            ScrollView(.vertical, showsIndicators: true) {
                VStack(alignment: .leading, spacing: 10) {
                    SectionHeader("Skeleton Position")
                    StepperRow(label: "Shoulder Y", value: $model.shoulderVerticalOffset, step: 0.02)
                    StepperRow(label: "Shoulder X", value: $model.shoulderLateralOffset, step: 0.01)
                    StepperRow(label: "Hip Y", value: $model.hipVerticalOffset, step: 0.02)
                    StepperRow(label: "Hip X", value: $model.hipLateralOffset, step: 0.01)

                    Divider().background(.white.opacity(0.3))

                    SectionHeader("Skeleton Projection")
                    StepperRow(label: "Radius", value: $model.skeletonRadius, step: 0.05)

                    Divider().background(.white.opacity(0.3))

                    SectionHeader("Upper Arm")
                    StepperRow(label: "Length", value: $model.upperArmLength, step: 0.01)
                    StepperRow(label: "Radius", value: $model.upperArmRadius, step: 0.01)

                    SectionHeader("Forearm")
                    StepperRow(label: "Length", value: $model.forearmLength, step: 0.01)
                    StepperRow(label: "Radius", value: $model.forearmRadius, step: 0.01)

                    SectionHeader("Thigh")
                    StepperRow(label: "Length", value: $model.thighLength, step: 0.01)
                    StepperRow(label: "Radius", value: $model.thighRadius, step: 0.01)

                    SectionHeader("Shank")
                    StepperRow(label: "Length", value: $model.shankLength, step: 0.01)
                    StepperRow(label: "Radius", value: $model.shankRadius, step: 0.01)

                    Divider().background(.white.opacity(0.3))

                    SectionHeader("Proximity Trigger")
                    StepperRow(label: "Radius", value: $model.proximityTriggerRadius, step: 0.1)

                    SectionHeader("Body Collision Trigger")
                    StepperRow(label: "Radius", value: $model.bodyCollisionRadius, step: 0.01)

                    SectionHeader("Distance Buckets (m)")
                    StepperRow(label: "Close Max", value: $model.distCloseMax, step: 0.05)
                    StepperRow(label: "Med Max", value: $model.distMedMax, step: 0.05)
                    StepperRow(label: "Far Max", value: $model.distFarMax, step: 0.05)

                    SectionHeader("Haptic Tie Tolerance")
                    // Tie cutoff: per obstacle, every eligible limb within
                    // this many meters of the closest limb's distance fires
                    // (and gets its visualizer drawn). 0 = strict closest
                    // wins; ~0.05 m fires both shoulders for a frontal
                    // wall when both are about equally near.
                    StepperRow(label: "Tie Margin (m)", value: $model.limbSwitchMargin, step: 0.01)

                    SectionHeader("Chest Haptic (Back-Center)")
                    // Toggle the virtual back-centerline candidate that
                    // drives the "chest" motor (UDP node 3).
                    HStack(spacing: 12) {
                        ToggleButton(
                            label: model.chestEligible ? "Chest On" : "Chest Off",
                            isActive: model.chestEligible,
                            activeColor: .green,
                            inactiveColor: .gray,
                            systemImage: model.chestEligible ? "circle.fill" : "circle"
                        ) {
                            model.chestEligible.toggle()
                        }
                    }
                    StepperRow(label: "Back Offset (m)", value: $model.chestBackOffset, step: 0.02)
                    StepperRow(label: "Vert Offset (m)", value: $model.chestVerticalOffset, step: 0.02)
                    // Rear cone half-angle: chest only competes for obstacles
                    // whose horizontal bearing is within this many degrees
                    // of straight behind the user. Outside the cone, chest
                    // is silent and the shoulder takes the obstacle.
                    StepperRow(label: "Rear Cone (°)", value: $model.rearConeHalfDegrees, step: 5.0)
                    HStack(spacing: 12) {
                        ToggleButton(
                            label: model.showRearConeVisual ? "Cone Visible" : "Cone Hidden",
                            isActive: model.showRearConeVisual,
                            activeColor: .green,
                            inactiveColor: .gray,
                            systemImage: "eye"
                        ) {
                            model.showRearConeVisual.toggle()
                        }
                    }

                    SectionHeader("Obstacle Course")
                    // Course extends along -Z. Each Start re-randomizes the
                    // 6 (or `Obstacles`) obstacles drawn from 4 type specs;
                    // VictoryGoal sits at exactly `-Path Length`.
                    StepperRow(label: "Path Length (m)",   value: $model.coursePathLength,   step: 1.0)
                    StepperRow(label: "Corridor Half (m)", value: $model.corridorHalfWidth,  step: 0.25)
                    IntStepperRow(label: "Obstacles",      value: $model.obstacleCount,      step: 1)
                    StepperRow(label: "Min Spacing (m)",   value: $model.obstacleMinSpacing, step: 0.25)
                    // Seconds between Start press and obstacles materializing.
                    StepperRow(label: "Start Grace (s)",   value: $model.courseStartGraceSec, step: 0.5)
                    // Right-side-only spawning for single-side testing.
                    HStack(spacing: 12) {
                        ToggleButton(
                            label: model.spawnRightSideOnly ? "Right Side Only" : "Both Sides",
                            isActive: model.spawnRightSideOnly,
                            activeColor: .blue,
                            inactiveColor: .gray,
                            systemImage: "arrow.right.circle.fill"
                        ) {
                            model.spawnRightSideOnly.toggle()
                        }
                    }
                }
            }
            .frame(maxHeight: 500)
        }
        .padding(20)
        .background(
            RoundedRectangle(cornerRadius: 16, style: .continuous)
                .fill(.black.opacity(0.7))
        )
    }
}

// MARK: - Section Header

struct SectionHeader: View {
    let title: String
    init(_ title: String) { self.title = title }

    var body: some View {
        Text(title)
            .font(.system(size: 16, weight: .bold))
            .foregroundStyle(.white)
            .padding(.top, 4)
    }
}

// MARK: - Stepper Row

struct StepperRow: View {
    let label: String
    @Binding var value: Float
    var step: Float = 0.01

    var body: some View {
        HStack(spacing: 10) {
            Text(label)
                .font(.system(size: 14))
                .foregroundStyle(.white.opacity(0.9))
                .frame(width: 90, alignment: .leading)

            Button {
                value -= step
            } label: {
                Image(systemName: "minus.circle.fill")
                    .font(.system(size: 52))
                    .foregroundStyle(.white.opacity(0.85))
                    .frame(width: 72, height: 72)
                    .contentShape(Rectangle())
            }
            .buttonBorderShape(.circle)

            Text(String(format: "%.2f", value))
                .font(.system(size: 18, weight: .semibold, design: .monospaced))
                .foregroundStyle(.white)
                .frame(width: 60, alignment: .center)

            Button {
                value += step
            } label: {
                Image(systemName: "plus.circle.fill")
                    .font(.system(size: 52))
                    .foregroundStyle(.white.opacity(0.85))
                    .frame(width: 72, height: 72)
                    .contentShape(Rectangle())
            }
            .buttonBorderShape(.circle)
        }
    }
}

// MARK: - Int Stepper Row

struct IntStepperRow: View {
    let label: String
    @Binding var value: Int
    var step: Int = 1

    var body: some View {
        HStack(spacing: 10) {
            Text(label)
                .font(.system(size: 14))
                .foregroundStyle(.white.opacity(0.9))
                .frame(width: 90, alignment: .leading)

            Button {
                value = max(0, value - step)
            } label: {
                Image(systemName: "minus.circle.fill")
                    .font(.system(size: 52))
                    .foregroundStyle(.white.opacity(0.85))
                    .frame(width: 72, height: 72)
                    .contentShape(Rectangle())
            }
            .buttonBorderShape(.circle)

            Text("\(value)")
                .font(.system(size: 18, weight: .semibold, design: .monospaced))
                .foregroundStyle(.white)
                .frame(width: 60, alignment: .center)

            Button {
                value += step
            } label: {
                Image(systemName: "plus.circle.fill")
                    .font(.system(size: 52))
                    .foregroundStyle(.white.opacity(0.85))
                    .frame(width: 72, height: 72)
                    .contentShape(Rectangle())
            }
            .buttonBorderShape(.circle)
        }
    }
}

// MARK: - Reusable Toggle Button

struct ToggleButton: View {
    let label: String
    let isActive: Bool
    let activeColor: Color
    let inactiveColor: Color
    let systemImage: String
    let action: () -> Void

    var body: some View {
        Button(action: action) {
            HStack(spacing: 10) {
                Image(systemName: systemImage)
                    .font(.system(size: 20, weight: .semibold))
                Text(label)
                    .font(.system(size: 18, weight: .semibold))
            }
            .foregroundStyle(.white)
            .frame(minHeight: 56)
            .padding(.horizontal, 24)
            .background(
                RoundedRectangle(cornerRadius: 14, style: .continuous)
                    .fill(isActive ? activeColor.opacity(0.85) : inactiveColor.opacity(0.85))
            )
        }
        .buttonBorderShape(.roundedRectangle(radius: 14))
    }
}
