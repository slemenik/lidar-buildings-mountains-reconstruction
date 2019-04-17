package com.slemenik.lidar.reconstruction.mountains;

import com.slemenik.lidar.reconstruction.main.Main;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

public class HeightController {

    public List<double[]> points2Insert = new ArrayList<>();

    public void readAscFile(String fileName) {

        try {
            Scanner scanner = new Scanner(new File(fileName));
            while (scanner.hasNextLine()) {
                String line = scanner.nextLine();
                String coordinates[] = line.split(";");
                double x = Double.parseDouble(coordinates[0]);
                double y = Double.parseDouble(coordinates[1]);
                double z = Double.parseDouble(coordinates[2]);
                this.points2Insert.add(new double[]{x, y, z});

            }
            scanner.close();
        } catch (Exception e) {
            e.printStackTrace();
        }

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
                    int bounds[] = Main.getBoundsFromFilename(lazFilename);

                    if (bounds[0] <= x && bounds[1] <= y && bounds[2] >= x && bounds[3] >= y) {
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
