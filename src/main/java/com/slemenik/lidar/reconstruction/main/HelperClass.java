package com.slemenik.lidar.reconstruction.main;

import javax.vecmath.Point3d;
import java.util.ArrayList;
import java.util.List;

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
