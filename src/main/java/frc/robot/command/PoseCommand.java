package frc.robot.command;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.tank.TankSubsystem;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.RobotConstants.PoseConstants.RotationPID.*;
import static frc.robot.RobotConstants.PoseConstants.TranslationPID.*;
import static frc.robot.RobotConstants.TankConstants.MAX_ANGULAR;
import static frc.robot.RobotConstants.TankConstants.MAX_SPEED;

public class PoseCommand extends Command {

    private final TankSubsystem mTankSubsystem;
    private final Pose2d targetLocation;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       
    private final PIDController translationPIDCtrl;
    private final PIDController rotationPIDCtrl;
  
    private static final double TRANSLATION_TOLERANCE_METERS = 0.05; // 5厘米
    private static final double ROTATION_TOLERANCE_DEGREES = 3.0;   // 3度

    public PoseCommand(Pose2d targetLocation, TankSubsystem mTankSubsystem) {
        this.targetLocation = targetLocation;
        this.mTankSubsystem = mTankSubsystem;

        translationPIDCtrl = new PIDController(TranKP.get(), TranKI.get(), TranKD.get());
        rotationPIDCtrl = new PIDController(RotKP.get(), RotKI.get(), RotKD.get());
        
        rotationPIDCtrl.enableContinuousInput(-180, 180);

        translationPIDCtrl.setTolerance(TRANSLATION_TOLERANCE_METERS);
        rotationPIDCtrl.setTolerance(ROTATION_TOLERANCE_DEGREES);

        addRequirements(mTankSubsystem);
    }

    @Override
    public void initialize() {
        if (RobotConstants.TUNING) {
            translationPIDCtrl.setPID(TranKP.get(), TranKI.get(), TranKD.get());
            rotationPIDCtrl.setPID(RotKP.get(), RotKI.get(), RotKD.get());
        }
        
        translationPIDCtrl.reset();
        rotationPIDCtrl.reset();
    }

    @Override
    public void execute() {
        Pose2d currentPose = mTankSubsystem.getRobotPose();
        
        Translation2d translationError = targetLocation.getTranslation().minus(currentPose.getTranslation());
        
        double distanceToTarget = translationError.getNorm();

        double forwardSpeed;
        Rotation2d targetRotation;

        if (distanceToTarget > TRANSLATION_TOLERANCE_METERS) {
            forwardSpeed = -translationPIDCtrl.calculate(distanceToTarget, 0);
            targetRotation = translationError.getAngle();
        } else {
            forwardSpeed = 0;
            targetRotation = targetLocation.getRotation();
        }

        double rotationSpeed = rotationPIDCtrl.calculate(
                currentPose.getRotation().getDegrees(),
                targetRotation.getDegrees()
        );

        forwardSpeed = MathUtil.clamp(forwardSpeed, -MAX_SPEED.in(MetersPerSecond), MAX_SPEED.in(MetersPerSecond));
        rotationSpeed = MathUtil.clamp(rotationSpeed, -MAX_ANGULAR.in(DegreesPerSecond), MAX_ANGULAR.in(DegreesPerSecond));
        mTankSubsystem.setArcadeSpeed(
                MetersPerSecond.of(forwardSpeed),
                DegreesPerSecond.of(rotationSpeed)
        );

        // log
        Logger.recordOutput("PoseCommand/TargetPose", targetLocation);
        Logger.recordOutput("PoseCommand/DistanceToTarget", distanceToTarget);
        Logger.recordOutput("PoseCommand/ForwardSpeed", forwardSpeed);
        Logger.recordOutput("PoseCommand/RotationSpeed", rotationSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        mTankSubsystem.setArcadeSpeed(MetersPerSecond.of(0), DegreesPerSecond.of(0));
    }

    @Override
    public boolean isFinished() {
        boolean translationFinished = Math.abs(targetLocation.getTranslation().minus(mTankSubsystem.getRobotPose().getTranslation()).getNorm())
                < TRANSLATION_TOLERANCE_METERS;
        
        boolean rotationFinished = Math.abs(targetLocation.getRotation().minus(mTankSubsystem.getRobotPose().getRotation()).getDegrees())
                < ROTATION_TOLERANCE_DEGREES;

        return translationFinished && rotationFinished;
    }
}