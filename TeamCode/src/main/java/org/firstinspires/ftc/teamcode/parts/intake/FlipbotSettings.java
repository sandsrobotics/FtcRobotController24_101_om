package org.firstinspires.ftc.teamcode.parts.intake;

import om.self.ezftc.utils.Vector3;

public class FlipbotSettings {

   /* the only purpose of this class is to store values accessible to all classes in FlipBot,
      and which can remain from run to tun
    */

   static boolean isAuto = false;
   static boolean isTeleOp = false;
   static Vector3 robotPosition = new Vector3();   //starts with all zeroes
   public static boolean isRedGood = true;
   public static boolean isYellowGood = true;
   public static boolean isBlueGood = false;
   public static boolean autonomousDebugMode = false;

   public static void setAuto () {
      isAuto = true;
      isTeleOp = false;
   }

   public static void setTeleOp () {
      isAuto = false;
      isTeleOp = true;
   }

   public static void storeRobotPosition(Vector3 currentPosition) {
      if (currentPosition.X == 0 && currentPosition.Y == 0 && currentPosition.Z == 0) return;
      robotPosition = currentPosition;
   }

   public static Vector3 getRobotPosition() {
      return robotPosition;
   }

}