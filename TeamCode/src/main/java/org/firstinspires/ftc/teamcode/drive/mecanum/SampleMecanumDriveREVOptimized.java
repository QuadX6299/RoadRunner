package org.firstinspires.ftc.teamcode.drive.mecanum;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.getMotorVelocityF;

import android.support.annotation.NonNull;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

/*
 * Optimized mecanum drive implementation for REV ExHs. The time savings may significantly improve
 * trajectory following performance with moderate additional complexity.
 */
public class SampleMecanumDriveREVOptimized extends SampleMecanumDriveBase {
    private ExpansionHubEx hubLeft, hubRight;
    private ExpansionHubMotor fl, bl, br, fr;
    private List<ExpansionHubMotor> leftMotors,rightMotors,allMotors;
    private BNO055IMU imu;

    public SampleMecanumDriveREVOptimized(HardwareMap hardwareMap) {
        super();

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        // TODO: adjust the names of the following hardware devices to match your configuration
        // for simplicity, we assume that the desired IMU and drive motors are on the same hub
        // if your motors are split between hubs, **you will need to add another bulk read**
        hubLeft = hardwareMap.get(ExpansionHubEx.class, "hubLeft");
        hubRight = hardwareMap.get(ExpansionHubEx.class, "hubRight");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
        // upward (normal to the floor) using a command like the following:
        // BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        fl = hardwareMap.get(ExpansionHubMotor.class, "fl");
        bl = hardwareMap.get(ExpansionHubMotor.class, "bl");
        br = hardwareMap.get(ExpansionHubMotor.class, "br");
        fr = hardwareMap.get(ExpansionHubMotor.class, "fr");

        leftMotors = Arrays.asList(fl, bl);
        rightMotors = Arrays.asList(br,fr);
        allMotors = Arrays.asList(fl,bl,br,fr);

        for (ExpansionHubMotor motor : allMotors) {
            if (RUN_USING_ENCODER) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()

        // TODO: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
    }

    @Override
    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        PIDFCoefficients coefficients = fl.getPIDFCoefficients(runMode);
        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
    }

    @Override
    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        for (ExpansionHubMotor motor : allMotors) {
            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, getMotorVelocityF()
            ));
        }
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
//

        RevBulkData bulkDataLeft = hubLeft.getBulkInputData();
        RevBulkData bulkDataRight = hubRight.getBulkInputData();

        if (bulkDataLeft == null) {
            return Arrays.asList(0.0, 0.0);
        }

        if (bulkDataRight == null){
            return  Arrays.asList(0.0,0.0);
        }

        List<Double> wheelPositions = new ArrayList<>();
        for (ExpansionHubMotor motor : leftMotors) {
            wheelPositions.add(encoderTicksToInches(bulkDataLeft.getMotorCurrentPosition(motor)));
        }

        for (ExpansionHubMotor motor : rightMotors){
            wheelPositions.add(encoderTicksToInches(bulkDataRight.getMotorCurrentPosition(motor)));
        }

        return wheelPositions;
    }


    @Override
    public List<Double> getWheelVelocities() {
        RevBulkData bulkDataLeft = hubLeft.getBulkInputData();
        RevBulkData bulkDataRight = hubRight.getBulkInputData();

        if (bulkDataLeft == null) {
            return Arrays.asList(0.0, 0.0);
        }

        if (bulkDataRight == null){
            return  Arrays.asList(0.0,0.0);
        }


        List<Double> wheelVelocities = new ArrayList<>();
        for (ExpansionHubMotor motor : leftMotors) {
            wheelVelocities.add(encoderTicksToInches(bulkDataLeft.getMotorVelocity(motor)));
        }

        for (ExpansionHubMotor motor : rightMotors){
            wheelVelocities.add(encoderTicksToInches(bulkDataRight.getMotorVelocity(motor)));
        }

        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        fl.setPower(v);
        bl.setPower(v1);
        br.setPower(v2);
        fr.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }
}
