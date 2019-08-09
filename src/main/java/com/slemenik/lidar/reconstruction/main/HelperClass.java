package com.slemenik.lidar.reconstruction.main;

import com.slemenik.lidar.reconstruction.jni.JniLibraryHelpers;

import javax.vecmath.Point3d;
import java.util.ArrayList;
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

    public static void printLine(String splitter, Object... params){
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

    public static void createFieldPointFile(boolean[][] field) {
        List<double[]> points = new ArrayList<>();
        for (int i = 0; i< field.length;i++) {
            for (int j = 0; j< field[i].length;j++) {
                if (field[i][j]) {
                    points.add(new double[]{i,j,0});
                }
            }
        }
        JniLibraryHelpers.writePointList(toResultDoubleArray(points), Main.INPUT_FILE_NAME, Main.OUTPUT_FILE_NAME +"fieldTest");
    }

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
        JniLibraryHelpers.writePointList(toResultDoubleArray(points), Main.INPUT_FILE_NAME, Main.OUTPUT_FILE_NAME +"fieldTest");
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
        System.out.println("totalMemory "+HelperClass.formatHeapSize(heapSize));
        System.out.println("heapFreeSize "+HelperClass.formatHeapSize(heapFreeSize));
        System.out.println("used "+HelperClass.formatHeapSize(used));

    }

//    public static double[][] toResultDoubleArray(List<Point3d> list) {
//    }
}
