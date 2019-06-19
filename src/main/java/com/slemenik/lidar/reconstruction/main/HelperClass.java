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

//    public static double[][] toResultDoubleArray(List<Point3d> list) {
//    }
}
