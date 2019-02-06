package com.slemenik.lidar.reconstruction.main;

import com.github.mreutegg.laszip4j.LASPoint;
import com.github.mreutegg.laszip4j.LASReader;
import io.pdal.pipeline.LasWrite;

import java.io.File;


public class Main {

    public static void main(String[] args) {

        LASReader reader = new LASReader(new File("./data/lj grad.laz"));
        for (LASPoint p : reader.getPoints()) {
            // read something from point
            System.out.println(p.getX());
            System.out.println(p.getY());
            System.out.println(p.getZ());
            p.getClassification();
        }



    }
}
