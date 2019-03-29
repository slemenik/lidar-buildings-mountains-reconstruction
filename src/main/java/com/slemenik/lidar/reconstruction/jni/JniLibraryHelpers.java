package com.slemenik.lidar.reconstruction.jni;

public class JniLibraryHelpers {

    //External tools:
    //program: C:\Program Files\Java\jdk-11.0.1\bin\javac.exe
    //arguments: -h data .\src\main\java\com\slemenik\lidar\reconstruction\jni\JniLibraryHelpers.java
    //working directory: C:\Users\Matej\IdeaProjects\lidar-buildings-mountains-reconstruction

      {
//        System.loadLibrary("cygLasReadWriteApi");
        System.load("C:\\Users\\Matej\\source\\repos\\Project2\\x64\\Debug\\Project2.dll");
//        System.loadLibrary("Project2");
    }

    private native void writeJNIPoint(double x, double y, double z);
    private native int writeJNIPointList(double[][] pointsArray, String inputFileName, String outputFileName);

    public void printDouble(double d) {
        System.out.println(d);
    }

    public void printString(String s) {
        System.out.println(s);
    }

    public static Integer writePoint(double x, double y, double z) {
        return writePointList(new double[][]{{x,y,z}});
    }

    public static Integer writePointList() {
        double[][] list =  {
                {1.0,11.0,111.0},
                {2.0,22.0,222.0},
                {3.0,33.0,333.0},
                {4.0,44.0,444.0},
                {5.0,55.0,555.0}
        };
        return writePointList(list);
    }

    public static Integer writePointList(double[][] list) {
        JniLibraryHelpers myInstance = new JniLibraryHelpers();
        System.out.println("klic funkcije writeJNIPointList()");

//        System.out.println(" seznam tock je: ");
//        for (int i = 0; i<list.length;i++) {
//            System.out.println(new Coordinate(list[i][0],  list[i][1], list[i][2]));
//        }
        return myInstance.writeJNIPointList(list,"france.laz","out.laz");
    }

}
