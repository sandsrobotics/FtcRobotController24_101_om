package org.firstinspires.ftc.teamcode.parts.intake;

import om.self.ezftc.utils.Vector3;

public class FlipbotSettings {

   /* The purpose of this class is to store values accessible to all classes in FlipBot,
      and which can remain from run to run
    */

   static boolean modeTeleOp = true;
   static Vector3 robotPosition = new Vector3();   //starts with all zeroes
   public static boolean isRedGood = true;
   public static boolean isYellowGood = true;
   public static boolean isBlueGood = false;
   public static boolean autonomousDebugMode = false;
   public static boolean isRangingEnabled = false;
   public static boolean firstRun = true;
//   static Vector3 controlGovernor = new Vector3(1,1,1);

   public static void setTeleOp () { modeTeleOp = true; }
   public static void setAuto () { modeTeleOp = false; }
   public static boolean isTeleOp() { return modeTeleOp; }
   public static boolean isAuto() { return !modeTeleOp; }
   public static boolean isRanging() {return  isRangingEnabled; }

//   public static void setControlGovernor(Vector3 multiplier) {
//      controlGovernor = new Vector3(clamp(multiplier.X), clamp(multiplier.Y), clamp(multiplier.Z));
//   }
//   public static void setControlGovernor(double multiplier) {
//      controlGovernor = new Vector3(clamp(multiplier));
//      //controlGovernor = new Vector3(clamp(multiplier, 0.25, 1));
//   }
//   public static Vector3 getControlGovernor() {
//      return controlGovernor;
//   }

   public static void storeRobotPosition(Vector3 currentPosition) {
      if (currentPosition.X == 0 && currentPosition.Y == 0 && currentPosition.Z == 0) return;
      robotPosition = currentPosition;
   }

   public static Vector3 getRobotPosition() {
      return robotPosition;
   }

//   private static double clamp(double pos) {
//      return Math.max(0, Math.min(pos, 1));
//   }
//   private static double clamp(double val, double min, double max) {
//      return Math.max(min, Math.min(val, max));
//   }

}