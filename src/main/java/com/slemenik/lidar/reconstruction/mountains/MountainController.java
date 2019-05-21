package com.slemenik.lidar.reconstruction.mountains;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

import static com.slemenik.lidar.reconstruction.buildings.ShpController.getBoundsFromFilename;

public class MountainController {

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
                    int bounds[] = getBoundsFromFilename(lazFilename);

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

    private static boolean[][] initField(double minX, double maxX, double minY, double maxY, double pointSpace) {


        double a = maxX - minX;
        System.out.println(a);
        System.out.println(pointSpace);
        System.out.println(a/pointSpace);
//        System.out.println(maxY - minY);
        int dimX = (int) ((maxX - minX) / pointSpace);
        int dimY = (int) ((maxY - minY) / pointSpace);
//        System.out.println(dimX);
//        System.out.println(dimY);
        boolean[][] field = new boolean[dimX][dimY];
        return field;
    }

    private static int point2Index(double coordinate, double min, double pointSpace) {
        return (int) ((coordinate - min) / pointSpace);
    }

    private static double index2Point(int x, double min, double pointSpace) {
        return ((double) x) * pointSpace + min;
    }

    public List<double[]> fillHoles(double[][]  arr, double pointsSpace) {

        int i = 0;
        QuadTree<Integer> q = new QuadTree<>();
        double minX = Integer.MAX_VALUE;
        double maxX = 0;
        double minY = Integer.MAX_VALUE;
        double maxY = 0;
        for (double[] arrEl: arr) {
            arr[i][0] = 0;
//            System.out.println(String.format("%f %f %f", arrEl[0], arrEl[1], arrEl[2] ));
            q.place(arr[i][1],arr[i][2], i);


            maxX = Double.max(maxX, arr[i][1] );
            minX = Double.min(minX, arr[i][1] );
            maxY = Double.max(maxY, arr[i][2] );
            minY = Double.min(minY, arr[i][2] );
            i++;
        }



        boolean[][] field = initField(minX,maxX,minY,maxY,pointsSpace);
        System.out.println("minX " + minX);
        System.out.println("maxX " + maxX);
        System.out.println("minY " + minY);
        System.out.println("maxY " + maxY);
        System.out.println(field.length);
        System.out.println(field[0].length);

        int indexX = 0;
        int indexY = 0;
        double temp = 0;
        try {
            for (double[] arrEl : arr) {
                temp = arrEl[1];
                indexX = Integer.min(point2Index(arrEl[1], minX, pointsSpace),field.length-1) ;
                indexY = Integer.min(point2Index(arrEl[2], minY, pointsSpace), field[0].length-1);
                field[indexX][indexY] = true;
            }
        }catch (Exception e) {
            System.out.println("error");
            System.out.println(indexX);
            System.out.println(indexY);
            System.out.println(temp);
        }

        List<double[]> result = new ArrayList<>();
        for (int x = 0; x<field.length; x++) {
            double newX = index2Point(x, minX, pointsSpace);
            for (int y = 0; y<field[0].length; y++) {
                double newY = index2Point(y, minY, pointsSpace);
                if (field[x][y] == false) {
                    result.add(new double[]{0.0, newX, newY});//temp
                }

            }
        }

        return result;



//        PrintStream printStream;

//        PrintStream ps = new PrintStream(System.out);
//        q.print(ps);
//        ps.flush();

//        System.out.println(q.size());
//        List<QuadTree<Integer>.CoordHolder> a = q.findAll( 137782.030 , 2761.990, 137783.340, 2762.980);
//        System.out.println(a.size());
//        for (QuadTree<Integer>.CoordHolder item : a) {
//            System.out.println(item.x);
//            System.out.println(item.y);
//            System.out.println(item.o);
//            System.out.println(item.depth());
//            System.out.println(item.quad);
//            System.out.println("_____________________-");
//
//        }





//        if (LP != null) q.LIST_PROVIDER = LP;
//        MAX = 1;
//        DYNAMIC = 0;
//        BUCKET_EXP = 0;
//        q.LEAF_MAX_OBJECTS = MAX;
//        if (DYNAMIC > 0)
//            q.DYNAMIC_MAX_OBJECTS = true;
//        q.MAX_OBJ_TARGET_EXPONENT = BUCKET_EXP;
//
//        for(int i = 0; i < nItems; i++)
//        {
//            q.place(Math.round(Math.random()*1000) + (windowShift * i / (double)nItems),
//                    Math.round(Math.random()*1000), i);
//        }
//        return q;

//        
    }

}
