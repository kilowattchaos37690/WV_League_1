package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * TeleOp diagnosis for Pinpoint Auto moves (especially turn unwind).
 *
 * Sticks do NOT drive. Buttons run the same blocking Auto routines used in
 * Sample Pinpoint Kids, with live + post-move telemetry on the Driver Station.
 *
 * gamepad1:
 *   A              reset pose to 0,0,0 (robot still)
 *   B              turnDegreesOdo(+90)  right/CW
 *   X              turnDegreesOdo(-90)  left/CCW
 *   Y              driveForwardOdo(+12)
 *   dpad up/down   driveForwardOdo(+6 / -6)
 *   dpad right/left strafeRightOdo(+6 / -6)
 *   right bumper   turnDegreesOdo(+90) at power 0.25 (gentle)
 *   left bumper    turnDegreesOdo(+90) at power 0.40 (stronger)
 *   back           stopDriveOdo
 *
 * Watch DS during the move (live) and after it ends (lastTurn snapshot):
 *   stop=passed_target → crossed commanded angle (good stop)
 *   stop=settle        → held inside heading tolerance
 *   stop=timeout       → never finished cleanly
 *   If prog goes past target and err flips large while still "driving" → unwind
 *
 * On DS: Start+A = gamepad1.
 */
@TeleOp(name = "Pinpoint Diag (Auto moves)", group = "Diag")
public class SamplePinpointDiag extends LinearOpMode {

  private static final double DIAG_POWER = 0.30;
  private static final double TURN_GENTLE = 0.25;
  private static final double TURN_STRONG = 0.40;

  @Override
  public void runOpMode() {
    RobotMyBlocks.bindFromLinearOpMode(this);
    RobotPinpointMyBlocks_diag.bindFromLinearOpMode(this);

    RobotMyBlocks.initRobot();
    RobotPinpointMyBlocks_diag.initPinpoint();
    RobotPinpointMyBlocks_diag.setPose(0.0, 0.0, 0.0);
    // Live DS telemetry every loop slows turns and can cause hunting — keep off
    // unless you need it. (reportLastTurn still prints after each move.)
    RobotPinpointMyBlocks_diag.setDebugPoseDrive(false);
    telemetry.setMsTransmissionInterval(100);

    telemetry.addData("Status", "Ready — Pinpoint Diag");
    telemetry.addData("B/X", "turn +90 / -90");
    telemetry.addData("RB/LB", "turn +90 gentle / strong");
    telemetry.addData("Y / dpad", "fwd 12 / fwd±6 / strafe±6");
    telemetry.addData("A", "reset pose 0,0,0");
    telemetry.addData("Tip", "A=reset before each turn; X was overshooting from slow heading unwrap");
    telemetry.addData("Expect X", "stop=passed_target, prog≈+90, absH≈+90 (not 270–360)");
    telemetry.update();
    waitForStart();

    ElapsedTime buttonLock = new ElapsedTime();

    while (opModeIsActive()) {
      // Edge-ish lock so a held button does not re-fire immediately after a move.
      boolean canFire = buttonLock.seconds() > 0.4;

      if (canFire && gamepad1.a) {
        RobotPinpointMyBlocks_diag.setPose(0.0, 0.0, 0.0);
        buttonLock.reset();
        telemetry.addData("Action", "pose reset 0,0,0");
      } else if (canFire && gamepad1.back) {
        RobotPinpointMyBlocks_diag.stopDriveOdo();
        RobotMyBlocks.stopDrive();
        buttonLock.reset();
        telemetry.addData("Action", "stop");
      } else if (canFire && gamepad1.b) {
        runLabeled("turnDegreesOdo(+90 @ " + DIAG_POWER + ")", () ->
            RobotPinpointMyBlocks_diag.turnDegreesOdo(90.0, DIAG_POWER));
        buttonLock.reset();
      } else if (canFire && gamepad1.x) {
        runLabeled("turnDegreesOdo(-90 @ " + DIAG_POWER + ")", () ->
            RobotPinpointMyBlocks_diag.turnDegreesOdo(-90.0, DIAG_POWER));
        buttonLock.reset();
      } else if (canFire && gamepad1.right_bumper) {
        runLabeled("turnDegreesOdo(+90 @ " + TURN_GENTLE + ")", () ->
            RobotPinpointMyBlocks_diag.turnDegreesOdo(90.0, TURN_GENTLE));
        buttonLock.reset();
      } else if (canFire && gamepad1.left_bumper) {
        runLabeled("turnDegreesOdo(+90 @ " + TURN_STRONG + ")", () ->
            RobotPinpointMyBlocks_diag.turnDegreesOdo(90.0, TURN_STRONG));
        buttonLock.reset();
      } else if (canFire && gamepad1.y) {
        runLabeled("driveForwardOdo(+12)", () ->
            RobotPinpointMyBlocks_diag.driveForwardOdo(12.0, DIAG_POWER));
        buttonLock.reset();
      } else if (canFire && gamepad1.dpad_up) {
        runLabeled("driveForwardOdo(+6)", () ->
            RobotPinpointMyBlocks_diag.driveForwardOdo(6.0, DIAG_POWER));
        buttonLock.reset();
      } else if (canFire && gamepad1.dpad_down) {
        runLabeled("driveForwardOdo(-6)", () ->
            RobotPinpointMyBlocks_diag.driveForwardOdo(-6.0, DIAG_POWER));
        buttonLock.reset();
      } else if (canFire && gamepad1.dpad_right) {
        runLabeled("strafeRightOdo(+6)", () ->
            RobotPinpointMyBlocks_diag.strafeRightOdo(6.0, DIAG_POWER));
        buttonLock.reset();
      } else if (canFire && gamepad1.dpad_left) {
        runLabeled("strafeRightOdo(-6)", () ->
            RobotPinpointMyBlocks_diag.strafeRightOdo(-6.0, DIAG_POWER));
        buttonLock.reset();
      }

      RobotPinpointMyBlocks_diag.reportPose();
      RobotPinpointMyBlocks_diag.reportLastTurn();
      telemetry.addData("Idle", "press a move button — sticks disabled");
      telemetry.update();
      sleep(50);
    }

    RobotPinpointMyBlocks_diag.setDebugPoseDrive(false);
    RobotPinpointMyBlocks_diag.stopDriveOdo();
    RobotMyBlocks.stopDrive();
  }

  private void runLabeled(String label, Runnable move) {
    telemetry.addData("Action", "RUNNING " + label);
    telemetry.update();
    move.run();
    telemetry.addData("Action", "DONE " + label);
    // reportLastTurn / reportPose already refreshed inside turnDegreesOdo;
    // other moves still need a refresh.
    RobotPinpointMyBlocks_diag.reportPose();
    RobotPinpointMyBlocks_diag.reportLastTurn();
    telemetry.update();
  }
}
