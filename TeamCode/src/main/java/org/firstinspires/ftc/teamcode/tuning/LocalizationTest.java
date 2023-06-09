package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.Vector;

@TeleOp
public class LocalizationTest extends LinearOpMode {
	final FtcDashboard dash = FtcDashboard.getInstance();
	@Override
	public void runOpMode() {
		Canvas c = new Canvas();
		final DriveView view = new DriveView(hardwareMap);
		waitForStart();


		Vector2d emitter_position = new Vector2d(0,0);
		Vector2d emitter_position1 = new Vector2d(0,-12);
		Vector2d emitter_position2 = new Vector2d(-12,0);


		while (opModeIsActive()) {

			TelemetryPacket p = new TelemetryPacket();
			p.fieldOverlay().getOperations().addAll(c.getOperations());
			c.clear();
			view.getMd().updatePose();
			Pose2d pose = view.getMd().pose;
			p.put("x", pose.trans.x);
			p.put("y", pose.trans.y);
			p.put("heading (deg)", Math.toDegrees(pose.rot.log()));
			c.setStroke("#3F51B5");
			MecanumDrive.drawRobot(c, pose);
			dash.sendTelemetryPacket(p);


			double g = 10;

			Vector2d extraForce = new Vector2d(0,0);

			Vector2d emitterToRobot = pose.trans.minus(emitter_position);
			double mag = emitterToRobot.norm();
			Vector2d unitEmitterToRobot = emitterToRobot.div(mag);
			double power = g / (mag * mag);
			Vector2d forceFieldRelative = unitEmitterToRobot.times(power);
			Vector2d forceRobotRelative = rotateBy(forceFieldRelative,pose.rot.log());
			extraForce = extraForce.plus(forceRobotRelative);

			emitterToRobot = pose.trans.minus(emitter_position1);
			mag = emitterToRobot.norm();
			unitEmitterToRobot = emitterToRobot.div(mag);
			power = g / (mag * mag);
			forceFieldRelative = unitEmitterToRobot.times(power);
			forceRobotRelative = rotateBy(forceFieldRelative,pose.rot.log());
			extraForce = extraForce.plus(forceRobotRelative);

			emitterToRobot = pose.trans.minus(emitter_position2);
			mag = emitterToRobot.norm();
			unitEmitterToRobot = emitterToRobot.div(mag);
			power = g / (mag * mag);
			forceFieldRelative = unitEmitterToRobot.times(power);
			forceRobotRelative = rotateBy(forceFieldRelative,pose.rot.log());
			extraForce = extraForce.plus(forceRobotRelative);






			view.setDrivePowers(
					new Twist2d(
							new Vector2d(
									-gamepad1.left_stick_y,
									(-gamepad1.left_stick_x)
							),
							-gamepad1.right_stick_x
					)
			);


		}
	}


	public static Vector2d rotateBy(Vector2d vec, double radians) {
		return new Vector2d(Math.cos(radians) * vec.x - Math.sin(radians) * vec.y,
							Math.sin(radians) * vec.x + Math.cos(radians) * vec.y);
	}

}
