package com.slemenik.lidar.reconstruction.main;

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

    public static void printLine(Object... params){
        for (int i = 0; i<params.length; i++) {
            System.out.print(params[i]);
            if (i != params.length-1) {
                System.out.print(", ");
            }
        }
        System.out.println();

    }

    // returns true if c is between a and b
    public static boolean isBetween(double c, double a, double b) {
        return b > a ? c > a && c < b : c > b && c < a;
    }

//    public static double[][] toResultDoubleArray(List<Point3d> list) {
//    }
}
