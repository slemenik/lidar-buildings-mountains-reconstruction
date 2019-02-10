package com.slemenik.lidar.reconstruction.main;

import com.github.mreutegg.laszip4j.LASPoint;
import com.github.mreutegg.laszip4j.LASReader;
import org.geotools.data.DataStore;
import org.geotools.data.DataStoreFinder;
import org.geotools.data.FeatureSource;
import org.geotools.feature.FeatureCollection;
import org.geotools.feature.FeatureIterator;
import org.opengis.feature.Feature;
import org.opengis.feature.GeometryAttribute;

import java.io.File;
import java.util.HashMap;
import java.util.Map;


public class Main {

    public static void main(String[] args) {
    shp();


    }

    public static void las() {
        LASReader reader = new LASReader(new File("./data/lj grad.laz"));
        for (LASPoint p : reader.getPoints()) {
            // read something from point
            System.out.println(p.getX());
            System.out.println(p.getY());
            System.out.println(p.getZ());
            p.getClassification();
        }

        Iterable<LASPoint> a = reader.getPoints();
    }

    public static void shp() {
        File file = new File("./data/BU_STAVBE_P.shp");

        try {
            Map<String, String> connect = new HashMap();
            connect.put("url", file.toURI().toString());

            DataStore dataStore = DataStoreFinder.getDataStore(connect);
            String[] typeNames = dataStore.getTypeNames();
            String typeName = typeNames[0];

            System.out.println("Reading content " + typeName);

            FeatureSource featureSource = dataStore.getFeatureSource(typeName);
            FeatureCollection collection = featureSource.getFeatures();
            FeatureIterator iterator = collection.features();
//
//
            try {
                int i = 0;
                while (iterator.hasNext() && i++ < 20) {
                    Feature feature = iterator.next();
                    GeometryAttribute sourceGeometry = feature.getDefaultGeometryProperty();

                    System.out.println(feature.getBounds().getMaxY());
                }
            } finally {
                iterator.close();
            }

        } catch (Throwable e) {}
    }
}
