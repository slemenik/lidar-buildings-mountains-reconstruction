package com.slemenik.lidar.reconstruction.main;

import com.slemenik.lidar.reconstruction.jni.JniLibraryHelpers;
import com.slemenik.lidar.reconstruction.mountains.HeightController;
import org.apache.commons.io.FilenameUtils;
import org.geotools.data.*;
import org.geotools.factory.CommonFactoryFinder;
import org.geotools.feature.FeatureCollection;
import org.geotools.feature.FeatureIterator;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.MultiPolygon;
import org.locationtech.jts.math.Vector2D;
import org.opengis.feature.Feature;
import org.opengis.feature.Property;
import org.opengis.feature.simple.SimpleFeature;
import org.opengis.feature.type.FeatureType;
import org.opengis.filter.Filter;
import org.opengis.filter.FilterFactory;
import org.opengis.geometry.BoundingBox;

import java.io.*;
import java.util.*;
import java.util.concurrent.TimeUnit;

import com.slemenik.lidar.reconstruction.mountains.triangulation.Triangulation;
import com.slemenik.lidar.reconstruction.buildings.BuildingController;


public class Main {

//    private static final String INPUT_FILE_NAME = ".\\data\\410_137_triglav.laz";
    public static final String DATA_FOLDER = "";
    public static final String INPUT_FILE_NAME = DATA_FOLDER + "462_100_grad.laz";
    public static final String OUTPUT_FILE_NAME = DATA_FOLDER + "out.laz";
    public static String TEMP_FILE_NAME = DATA_FOLDER + "temp.laz";
    public static final String DMR_FILE_NAME = DATA_FOLDER + "GK1_410_137.asc";

    public static final double DISTANCE_FROM_ORIGINAL_POINT_THRESHOLD = 0.8; //manjše je bolj natančno za detajle, ne prekrije celega
    public static final double CREATED_POINTS_SPACING = 0.2;//2.0;//0.2;
    public static final boolean WRITE_POINTS_INDIVIDUALLY = false;
    public static final boolean CONSIDER_EXISTING_POINTS = false; //rešetke
    public static final double BOUNDING_BOX_FACTOR = 1.0;// za koliko povečamo mejo boundingboxa temp laz file-a
    public static final boolean CREATE_TEMP_FILE = true;
    public static final boolean PROCESS_WHOLE_FILE = false;
    public static final int[] TEMP_BOUNDS = new int[]{462264, 100575, 462411, 100701};

    private static int count = 0;
    public static List<double[]> points2Insert = new ArrayList<>();



    public static void main(String[] args) {
        System.out.println("start");
        long startTime = System.nanoTime();


        BuildingController.write(TEMP_BOUNDS);
//        write(TEMP_BOUNDS);
        System.out.println("Konec racunanja.");

        //Triangulation.triangulate(DMR_FILE_NAME);

        HeightController hc = new HeightController();
        //hc.readAscFile(DMR_FILE_NAME);
        points2Insert = hc.points2Insert;
        try {
            //String a = getHTML("http://maps.googleapis.com/maps/api/streetview?size=600x400&location=12420+timber+heights,+Austin&key=AIzaSyCkUOdZ5y7hMm0yrcCQoCvLwzdM6M8s5qk");
            //System.out.println(a);
        }catch (Exception e) {
            System.out.println(e);
        }

        if (!points2Insert.isEmpty()) {
            System.out.println("zacetek pisanja... ");
            double[][] pointListDoubleArray = points2Insert.toArray(new double[][]{});
            int returnValue = JniLibraryHelpers.writePointList(pointListDoubleArray, INPUT_FILE_NAME, OUTPUT_FILE_NAME);
            System.out.println(returnValue);
        }

        System.out.println();
        long endTime = System.nanoTime();
        long duration = (endTime - startTime);
        long time = TimeUnit.SECONDS.convert(duration, TimeUnit.NANOSECONDS);
        System.out.println("Sekunde izvajanja: " + time);
        System.out.println("end");
    }

}
