package org.firstinspires.ftc.teamcode.Utils;
import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import java.io.File;

public class FileHandler {

    public static void deleteFile (String toFileName) {

        File myFileName = AppUtil.getInstance().getSettingsFile(toFileName);
        myFileName.delete();



    }
    public static boolean fileExists (String toFileName) {

        File myFileName = AppUtil.getInstance().getSettingsFile(toFileName);
        return myFileName.exists();


    }

    public static void writeValueToFile (double myNumber, String toFileName) {

        File myFileName = AppUtil.getInstance().getSettingsFile(toFileName);
        ReadWriteFile.writeFile(myFileName, String.valueOf(myNumber));


    }   // end of method writeToFile()


    public static double readValueFromFile (String fromFileName) {

        File myFileName = AppUtil.getInstance().getSettingsFile(fromFileName);

        double myNumber = Double.parseDouble(ReadWriteFile.readFile(myFileName).trim());

        return myNumber;       // provide the number to the Block calling this myBlock

    }  // end of method readFromFile()
    public static void writeStringToFile (String myString, String toFileName) {

        File myFileName = AppUtil.getInstance().getSettingsFile(toFileName);
        ReadWriteFile.writeFile(myFileName, myString);


    }   // end of method writeToFile()


    @NonNull
    public static String readSringFromFile (String fromFileName) {

        File myFileName = AppUtil.getInstance().getSettingsFile(fromFileName);

        return ReadWriteFile.readFile(myFileName).trim();       // provide the number to the Block calling this myBlock

    }  // end of method readFromFile()

}
