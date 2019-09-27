package com.slemenik.lidar.reconstruction.main;

import com.slemenik.lidar.reconstruction.jni.JniLibraryHelpers;
import delaunay_triangulation.Point_dt;

import javax.vecmath.Point3d;
import java.time.LocalTime;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.SortedSet;
import java.util.TreeSet;

public class HelperClass {

    public static double[][] toResultDoubleArray(List list){
        if (list.isEmpty()){
            return new double[][]{};
        } else if (list.get(0) instanceof double[]) {
            return (double[][]) list.toArray(new double[][]{});
        } else if (list.get(0) instanceof Point3d) {
           return ((List<Point3d>) list).stream().map(point3d -> new double[]{point3d.x, point3d.y, point3d.z}).toArray(double[][]::new);
        } else if (list.get(0) instanceof int[]) {
            return ((List<int[]>) list).stream().map(index -> new double[]{index[0], index[1], 0}).toArray(double[][]::new);
        } else {
            return null;
        }

    }

    public static double[][] toResultDoubleArray(TreeSet treeSet){
        if (treeSet.isEmpty()){
            return new double[][]{};
        } else {
            return ((TreeSet<Point3d>)treeSet).stream().map(point -> new double[]{point.x, point.y, point.z /*0/*temp*/}).toArray(double[][]::new);//todo paralel streams?
        }

    }

    public static double[][] toResultDoubleArray(SortedSet set){
        if (set.isEmpty()){
            return new double[][]{};
        } else {
            return ((SortedSet<Point3d>)set).stream().map(point -> new double[]{point.x, point.y, point.z /*0/*temp*/}).toArray(double[][]::new);//todo paralel streams?
        }

    }

    public static double[][] toResultDoubleArray(TreeSet treeSet, Main.Classification classification) {
        if (treeSet == null || treeSet.isEmpty()){
            return new double[][]{};
        } else {
            return ((SortedSet<Point3d>)treeSet).stream().map(point -> new double[]{point.x, point.y, point.z /*0/*temp*/, classification.getDoubleValue()}).toArray(double[][]::new);//todo paralel streams?
        }
    }

    public static double[][] toResultDoubleArray(List list,  Main.Classification classification){
        if (list == null || list.isEmpty()){
            return new double[][]{};
        } else if (list.get(0) instanceof double[]) {
            return (double[][]) list.toArray(new double[][]{});
        } else if (list.get(0) instanceof Point3d) {
            return ((List<Point3d>) list).stream().map(point3d -> new double[]{point3d.x, point3d.y, point3d.z, classification.getDoubleValue()}).toArray(double[][]::new);
        } else if (list.get(0) instanceof int[]) {
            return ((List<int[]>) list).stream().map(index -> new double[]{index[0], index[1], 0, classification.getDoubleValue()}).toArray(double[][]::new);
        } else {
            return null;
        }

    }

    public static double[][] toResultDoubleArray (Point_dt[] arr) {
        return Arrays.asList(arr).stream().map(x -> new double[]{x.x(), x.y(), x.z()}).toArray(double[][]::new);
    }

    public static void printLine(String splitter, Object... params){
        System.out.print("[" + LocalTime.now().withNano(0) + "] ");
        for (int i = 0; i<params.length; i++) {
            System.out.print(params[i]);
            if (i != params.length-1) {
                System.out.print(splitter);
            }
        }
        System.out.println();

    }

    public static double index2PointTemp(int x, double min, double pointSpace) {//temp spremeni v private
        return ((double) x) * pointSpace + min;
    }

    public static void createFieldPointFile(List<int[]> indexList) {
        List<double[]> points = new ArrayList<>();
        indexList.forEach(intArray -> {
            points.add(new double[]{intArray[0],intArray[1],0});
        });
        JniLibraryHelpers.writePointList(toResultDoubleArray(points), Main.INPUT_FILE_NAME, Main.OUTPUT_FILE_NAME +"fieldTestTransformedCoo");
    }

    /*creates points from field, z = 0, x and y are coordinates starting from 0 to field.length*/
    public static void createFieldPointFile(boolean[][] field) {
        List<double[]> points = new ArrayList<>();
        for (int i = 0; i< field.length;i++) {
            for (int j = 0; j< field[i].length;j++) {
                if (field[i][j]) {
                    points.add(new double[]{i,j,0});
                }
            }
        }
        JniLibraryHelpers.writePointList(toResultDoubleArray(points), Main.INPUT_FILE_NAME, Main.OUTPUT_FILE_NAME +"fieldTestZeroCoo");
    }

    /*creates points from field, z = value of field[i][j], x and y are coordinates starting from 0 to field.length*/
    public static void createFieldPointFile(double[][] field, boolean useZ) {
        List<double[]> points = new ArrayList<>();
        for (int i = 0; i< field.length;i++) {
            for (int j = 0; j< field[i].length;j++) {
                if (field[i][j] > 0) {
                    double z = useZ ? field[i][j] : 0;
                    points.add(new double[]{i,j,z});
                }
            }
        }
        JniLibraryHelpers.writePointList(toResultDoubleArray(points), Main.INPUT_FILE_NAME, Main.OUTPUT_FILE_NAME +"fieldTestHeightIndexCoo");
    }

    public static void createFieldPointFile(List<int[]> indexList, double minX, double minY, double pointsSpace) {
        List<double[]> points = new ArrayList<>();
        indexList.stream().forEach(intArray -> {
            double newX = index2PointTemp(intArray[0], minX, pointsSpace);
            double newY = index2PointTemp(intArray[1], minY, pointsSpace);
            points.add(new double[]{newX,newY,0});
        });
        JniLibraryHelpers.writePointList(toResultDoubleArray(points), Main.INPUT_FILE_NAME, Main.OUTPUT_FILE_NAME +"fieldTestTransformedCoo");
    }

    /*creates points from field, z = 0, x and y are actual coordinates of corespondant field*/
    public static void createFieldPointFile(boolean[][] field, double minX, double minY, double pointsSpace) {
        List<double[]> points = new ArrayList<>();
        for (int i = 0; i< field.length;i++) {
            for (int j = 0; j< field[i].length;j++) {
                if (field[i][j]) {
                    double newX = index2PointTemp(i, minX, pointsSpace);
                    double newY = index2PointTemp(j, minY, pointsSpace);
                    points.add(new double[]{newX,newY,0});
                }
            }
        }
        JniLibraryHelpers.writePointList(toResultDoubleArray(points), Main.INPUT_FILE_NAME, Main.OUTPUT_FILE_NAME +"fieldTestTransformedCoo");
    }

    // returns true if c is between a and b
    public static boolean isBetween(double c, double a, double b) {
        return b > a ? c > a && c < b : c > b && c < a;
    }

    public static String formatHeapSize(long v) {
        if (v < 1024) return v + " B";
        int z = (63 - Long.numberOfLeadingZeros(v)) / 10;
        return String.format("%.1f %sB", (double)v / (1L << (z*10)), " KMGTPE".charAt(z));
    }

    public static void memory() {
        // Get current size of heap in bytes
        long heapSize = Runtime.getRuntime().totalMemory();

        // Get amount of free memory within the heap in bytes. This size will increase // after garbage collection and decrease as new objects are created.
        long heapFreeSize = Runtime.getRuntime().freeMemory();

        long used = heapSize - heapFreeSize;
        printLine("", "totalMemory "+HelperClass.formatHeapSize(heapSize));
        printLine("","heapFreeSize "+HelperClass.formatHeapSize(heapFreeSize));
        printLine("","used "+HelperClass.formatHeapSize(used));

    }

    public static int getValueInsideBounds(int value, int arrayLength) {
        return Integer.max(Integer.min(value, arrayLength-1), 0); //value must be between 0 and arrayLength-1

    }

//    public static double[][] toResultDoubleArray(List<Point3d> list) {
//    }
}
