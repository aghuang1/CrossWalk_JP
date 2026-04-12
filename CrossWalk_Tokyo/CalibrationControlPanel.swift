/*
 CalibrationControlPanel.swift
 CrossWalk_Tokyo

 Calibration UI and skeleton parameter controls for the body tracking system.
 Adapted from ExtendedTouch_AVP's CubeMeshInteraction.swift control panel views.
*/

import SwiftUI

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
                        label: model.isCalibrating ? "Calibrating..." : "Re-Calibrate",
                        isActive: !model.isCalibrating,
                        activeColor: .purple,
                        inactiveColor: .gray,
                        systemImage: "dial.and.needle.fill"
                    ) {
                        guard !model.isCalibrating else { return }
                        model.startCalibration()
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
