package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp
public class LocalizationTest extends LinearOpMode {
	final FtcDashboard dash = FtcDashboard.getInstance();
	@Override
	public void runOpMode() {
		Canvas c = new Canvas();
		final DriveView view = new DriveView(hardwareMap);
		waitForStart();

		while (opModeIsActive()) {

			TelemetryPacket p = new TelemetryPacket();
			p.fieldOverlay().getOperations().addAll(c.getOperations());
			c.clear();

			view.setDrivePowers(
					new Twist2d(
							new Vector2d(
									-gamepad1.left_stick_y,
									(-gamepad1.left_stick_x)
							),
							-gamepad1.right_stick_x
					)
			);
			view.getMd().updatePose();
			Pose2d pose = view.getMd().pose;
			p.put("x", pose.trans.x);
			p.put("y", pose.trans.y);
			p.put("heading (deg)", Math.toDegrees(pose.rot.log()));
			c.setStroke("#3F51B5");
			MecanumDrive.drawRobot(c, pose);
			dash.sendTelemetryPacket(p);

		}
	}

}
