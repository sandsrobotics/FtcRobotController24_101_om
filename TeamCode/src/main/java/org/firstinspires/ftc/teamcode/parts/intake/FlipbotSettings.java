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

   public static void setTeleOp () { modeTeleOp = true; }
   public static void setAuto () { modeTeleOp = false; }
   public static boolean isTeleOp() { return modeTeleOp; }
   public static boolean isAuto() { return !modeTeleOp; }
   public static boolean isRanging() {return  isRangingEnabled; }

   public static void storeRobotPosition(Vector3 currentPosition) {
      if (currentPosition.X == 0 && currentPosition.Y == 0 && currentPosition.Z == 0) return;
      robotPosition = currentPosition;
   }

   public static Vector3 getRobotPosition() {
      return robotPosition;
   }

}