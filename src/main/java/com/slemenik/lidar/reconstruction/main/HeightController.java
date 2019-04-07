package com.slemenik.lidar.reconstruction.main;

import com.slemenik.lidar.reconstruction.jni.JniLibraryHelpers;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

public class HeightController {

    public List<double[]> points2Insert;

    public void readFile(String fileName) {

    }

    public void parseFolder(String folderPath, String lazFilename) {
        this.points2Insert = new ArrayList<>();
        File fileList[] = new File(folderPath).listFiles();
        for(File file : fileList) {
            System.out.println("File z imenom:" + file.getName());
            try {
                Scanner scanner = new Scanner(file);
                while (scanner.hasNextLine()) {
                    String line = scanner.nextLine();
                    String coordinates[] = line.split("\\s+");
                    double x = Double.parseDouble(coordinates[0]);
                    double y = Double.parseDouble(coordinates[1]);
                    double z = Double.parseDouble(coordinates[2]);
                    int bounds[] = Main.getLowerBoundsFromFilename(lazFilename);

                    if (bounds[0] <= x && bounds[1] <= y && bounds[0] + 1000 >= x && bounds[1] + 1000 >= y) {
                        this.points2Insert.add(new double[]{x, y, z});
                    }
                    if (points2Insert.size() > 16 * 1000000) {
                        return;
                    }
                }
                scanner.close();
            } catch (FileNotFoundException e) {
                e.printStackTrace();
            }
            System.out.println("trenutno tcck:" + points2Insert.size());
        }
    }

}
