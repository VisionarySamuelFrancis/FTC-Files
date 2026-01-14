package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10)
            .forwardZeroPowerAcceleration(-37.05409533191949) //-29.350800939975258
            .lateralZeroPowerAcceleration( 38.67654576491922) // -33.8349634557578
            .translationalPIDFCoefficients(new PIDFCoefficients(0.05, 0, 0.02, 0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0, 0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025,0.0,0.00001,0.6,0.01))
            .centripetalScaling(0.0005)
            ;
    
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    //Drive Train
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(66.20194274421752)
            .yVelocity(99.15196011761043);

    //X: 64.82347383273869
    //Y: 54.846202940452756
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(0)
            .strafePodX(-8)

            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("odo")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants) // ← THIS WAS MISSING
                .build();
    }
}
