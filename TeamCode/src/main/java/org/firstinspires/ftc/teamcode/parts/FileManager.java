package org.firstinspires.ftc.teamcode.parts;

import android.content.Context;
import android.util.Log;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;

/*************************************************************************************
 * Example of writing and reading from a file using FileManager.  You will need to write
 * so you can convert it to a Vector3 after you read it.  Here the single value is done
 * with Double.parseDouble.
 *
 *     public boolean updateRotFromFile(){
 *         String val = FileManager.readFromFile(fileName);
 *         try {
 *             positionSettings.startPos.R = Double.parseDouble(val);
 *             return true;
 *         }
 *         catch(Exception e){return false;}
 *     }
 *
 *     void writeRotationToFile(){
 *         FileManager.writeToFile(fileName, Double.toString(currentPosition.R));
 *     }
 **************************************************************************************/

class FileManager {

    static String readFromFile(String fileName) {
        Context context = AppUtil.getDefContext();
        String ret = null;

        try {
            InputStream inputStream = context.openFileInput(fileName);

            if (inputStream != null) {
                InputStreamReader inputStreamReader = new InputStreamReader(inputStream);
                BufferedReader bufferedReader = new BufferedReader(inputStreamReader);
                String receiveString;
                StringBuilder stringBuilder = new StringBuilder();

                while ((receiveString = bufferedReader.readLine()) != null) {
                    stringBuilder.append("\n").append(receiveString);
                }

                inputStream.close();
                ret = stringBuilder.toString();
            }
        } catch (FileNotFoundException e) {
            Log.e("login activity", "File not found: " + e.toString());
        } catch (IOException e) {
            Log.e("login activity", "Can not read file: " + e.toString());
        }

        return ret;
    }
    static void writeToFile(String fileName, String data) {
        Context context = AppUtil.getDefContext();

        try {
            OutputStreamWriter outputStreamWriter = new OutputStreamWriter(context.openFileOutput(fileName, Context.MODE_PRIVATE));
            outputStreamWriter.write(data);
            outputStreamWriter.close();
        }
        catch (IOException e) {
            Log.e("Exception", "File write failed: " + e.toString());
        }
    }
}
