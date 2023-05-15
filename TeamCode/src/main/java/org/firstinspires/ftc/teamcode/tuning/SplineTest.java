package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.ActionOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

import java.util.Vector;

public final class SplineTest extends ActionOpMode {
    Pose2d initialPose = new Pose2d(-60, 36, 0);
    Vector2d first = new Vector2d(-36, 36);
    Vector2d second = new Vector2d(-36,12);
    Vector2d third = new Vector2d(-12,12);
    Vector2d fourth = new Vector2d(24,12);

    @Override
    public void runOpMode() throws InterruptedException {
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

            waitForStart();

            runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineTo(first, calculateTangent(initialPose,first))
                        .splineTo(second, calculateTangent(first,second))
                        .splineTo(third, calculateTangent(second,third))
                        .splineTo(fourth, calculateTangent(third,fourth))
                        .build());
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            runBlocking(
                    drive.actionBuilder(drive.pose)
                            .splineTo(new Vector2d(30, 15), Math.PI / 2)
                            .splineTo(new Vector2d(60, 0), Math.PI)
                            .build());
        } else {
            throw new AssertionError();
        }
    }
    public static double calculateTangent(Pose2d initialPosition, Pose2d finalPosition) {
        double xd = initialPosition.trans.x - finalPosition.trans.x;
        double yd = initialPosition.trans.y - finalPosition.trans.y;
        return Math.atan2(yd,xd) - Math.PI;
    }
    public static double calculateTangent(Vector2d initialPosition, Vector2d finalPosition) {
        double xd = initialPosition.x - finalPosition.x;
        double yd = initialPosition.y - finalPosition.y;
        return Math.atan2(yd,xd) - Math.PI;
    }
    public static double calculateTangent(Pose2d initialPosition, Vector2d finalPosition) {
        double xd = initialPosition.trans.x - finalPosition.x;
        double yd = initialPosition.trans.y - finalPosition.y;
        return Math.atan2(yd,xd) - Math.PI;
    }
    public static double calculateTangent(Vector2d initialPosition, Pose2d finalPosition) {
        double xd = initialPosition.x - finalPosition.trans.x;
        double yd = initialPosition.y - finalPosition.trans.y;
        return Math.atan2(yd,xd) - Math.PI;
    }
}
