package org.frc5411.lib.instrument.module;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public record Setpoint(ChassisSpeeds Speeds, SwerveModuleState[] States) {}