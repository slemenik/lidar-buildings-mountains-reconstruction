package com.slemenik.lidar.reconstruction.main;

public class DTO {

    public static class LasHeader {

        public double minX, maxX, minY, maxY, minZ, maxZ;

        public LasHeader(double[] lasHeaderParams) {
            this.minX = lasHeaderParams[0];
            this.maxX = lasHeaderParams[1];
            this.minY = lasHeaderParams[2];
            this.maxY = lasHeaderParams[3];
            this.minZ = lasHeaderParams[4];
            this.maxZ = lasHeaderParams[5];
        }

    }
}
