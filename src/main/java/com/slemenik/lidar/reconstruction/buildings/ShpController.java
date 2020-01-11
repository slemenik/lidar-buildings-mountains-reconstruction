package com.slemenik.lidar.reconstruction.buildings;

import com.slemenik.lidar.reconstruction.main.HelperClass;
import org.apache.commons.io.FilenameUtils;
import org.geotools.data.*;
import org.geotools.data.collection.ListFeatureCollection;
import org.geotools.data.shapefile.ShapefileDataStore;
import org.geotools.data.shapefile.ShapefileDataStoreFactory;
import org.geotools.data.simple.SimpleFeatureCollection;
import org.geotools.data.simple.SimpleFeatureSource;
import org.geotools.data.simple.SimpleFeatureStore;
import org.geotools.factory.CommonFactoryFinder;
import org.geotools.feature.FeatureCollection;
import org.geotools.feature.FeatureIterator;
import org.opengis.feature.simple.SimpleFeature;
import org.opengis.feature.simple.SimpleFeatureType;
import org.opengis.feature.type.FeatureType;
import org.opengis.filter.Filter;
import org.opengis.filter.FilterFactory;
import java.io.File;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;


public class ShpController {

//    static FeatureCollection oldFeatureCollectiontemp;
//    static FeatureSource oldFeatureSourcetemp;

    public static DataStore dataStore;

    public static FeatureSource getFeatureSource(String shpFileName) {
        File file = new File(shpFileName);
        FeatureSource featureSource = null;
        try {
            Map<String, String> connect = new HashMap();
            connect.put("url", file.toURI().toString());

            dataStore = DataStoreFinder.getDataStore(connect);
            String[] typeNames = dataStore.getTypeNames();
            String typeName = typeNames[0];

            System.out.println("Reading content " + typeName);

            featureSource = dataStore.getFeatureSource(typeName);

        } catch (Throwable e) {
            HelperClass.printLine(" ", "moja napaka, metoda ShpController.getFeatureSource(): ", e );
        }
        return featureSource;
    }

    public static FeatureIterator getFeatures (double bounds[], String shpFileName) {
        try {

            FeatureSource oldFeatureSource = getFeatureSource(shpFileName);
//            oldFeatureSourcetemp = oldFeatureSource;



            FeatureType schema = oldFeatureSource.getSchema();
            String name = schema.getGeometryDescriptor().getLocalName();//the_geom
            String typeName = schema.getName().getLocalPart(); //BU_STAVBE_P

            System.out.println(name);
            System.out.println();
            String srs = oldFeatureSource.getBounds().getCoordinateReferenceSystem().toString();

            FilterFactory ff = CommonFactoryFinder.getFilterFactory( null );

            Filter filter = ff.bbox(name, bounds[0], bounds[1], bounds[2], bounds[3], srs);

//
//            ff.property( "the_geom"), ff.literal( 12 )
//            Filter filter = CQL.toFilter(text.getText());

            Query query = new Query(typeName, filter, new String[] {name});
            FeatureCollection oldFeatureCollection = oldFeatureSource.getFeatures(query);
//            oldFeatureCollectiontemp = oldFeatureCollection;
//            System.out.println(oldFeatureSource.getFeatures(query).size());
//            System.out.println(oldFeatureSource.getFeatures().size());

//            List<SimpleFeature> features = new ArrayList<>(); //uncomment if write shp
//            System.out.println(collection.size());
//            FeatureIterator iterator = oldFeatureCollection.features();
            return oldFeatureCollection.features();



        } catch (Exception e){
            System.out.println(e);
            return null;
        }
    }

    public static double[] getBoundsFromFilename(String filename){
        String fileName = FilenameUtils.getBaseName(filename);
        int bboxX = 0;
        int bboxY = 0;
        for (String str : fileName.split("_")) {
            try {
                int num = Integer.parseInt(str);
                if (bboxX == 0) {
                    bboxX = num;
                } else if (bboxY == 0) {
                    bboxY = num;
                    break;
                }
            } catch (NumberFormatException e) {

            }
        }
        return new double[]{bboxX*1000, bboxY*1000, (bboxX*1000)+1000, (bboxY*1000)+1000};
    }

    public static void writeShpFile(FeatureSource oldFeatureSource, List<SimpleFeature> features, FeatureCollection oldFeatureCollection, String newFileName) {
        try {
            /*
             * Get an output file name and create the new shapefile
             */
            File newFile = new File(newFileName);

            ShapefileDataStoreFactory dataStoreFactory = new ShapefileDataStoreFactory();

            Map<String, Serializable> params = new HashMap<>();
            params.put("url", newFile.toURI().toURL());
            params.put("create spatial index", Boolean.TRUE);

            ShapefileDataStore newDataStore =
                    (ShapefileDataStore) dataStoreFactory.createNewDataStore(params);

            SimpleFeatureType TYPE = (SimpleFeatureType) oldFeatureSource.getSchema();

            /*
             * TYPE is used as a template to describe the file contents
             */
            newDataStore.createSchema(TYPE);

            /*
             * Write the features to the shapefile
             */
            Transaction transaction = new DefaultTransaction("create");

            String typeName = newDataStore.getTypeNames()[0];
            System.out.println("typeName: " + typeName);
            SimpleFeatureSource featureSource = newDataStore.getFeatureSource(typeName);
            SimpleFeatureType SHAPE_TYPE = featureSource.getSchema();
            /*
             * The Shapefile format has a couple limitations:
             * - "the_geom" is always first, and used for the geometry attribute name
             * - "the_geom" must be of type PointDTO, MultiPoint, MuiltiLineString, MultiPolygon
             * - Attribute names are limited in length
             * - Not all data types are supported (example Timestamp represented as Date)
             *
             * Each data store has different limitations so check the resulting SimpleFeatureType.
             */
            System.out.println("SHAPE:" + SHAPE_TYPE);

            if (featureSource instanceof SimpleFeatureStore) {
                SimpleFeatureStore featureStore = (SimpleFeatureStore) featureSource;
                /*
                 * SimpleFeatureStore has a method to add features from a
                 * SimpleFeatureCollection object, so we use the ListFeatureCollection
                 * class to wrap our list of features.
                 */
                SimpleFeatureCollection collection = new ListFeatureCollection(TYPE, features);
                featureStore.setTransaction(transaction);
                try {
                    featureStore.addFeatures(oldFeatureCollection/*collection*/);
                    transaction.commit();
                } catch (Exception problem) {
                    problem.printStackTrace();
                    transaction.rollback();
                } finally {
                    transaction.close();
                }
                System.exit(0); // success!
            } else {
                System.out.println(typeName + " does not support read/write access");
                System.exit(1);
            }
        } catch (Exception e) {System.out.println(e);}
    }


}
