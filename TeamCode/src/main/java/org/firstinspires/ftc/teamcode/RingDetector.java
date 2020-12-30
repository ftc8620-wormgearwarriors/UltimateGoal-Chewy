package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;

public class RingDetector {

    // member variables
    Bitmap m_ringBitmap = null;
    int m_cropBoxLeft = 300;
    int m_cropBoxRight = 420;
    int m_cropBoxTop = 210;
    int m_cropBoxBottom = 290;

    // constructor
    public RingDetector(Bitmap ringImage) {

        // deep copy to make new bitmap owned by ring detector class
        m_ringBitmap = Bitmap.createBitmap(ringImage);
    }

    // setter for crop box if needs changed
    public void setCropBox(int left, int right, int top, int bottom) {
        m_cropBoxLeft = left;
        m_cropBoxRight = right;
        m_cropBoxTop = top;
        m_cropBoxBottom = bottom;
    }

    // function to return the number of rings based on the bitmap
    public int getNumberOfRings(Bitmap croppedRingImage, int nPixelThreshold,
                                double dRatioThreshold1, double dRatioThreshold2) {

        // Getting size of cropped bit map
        int cropImageHeight = croppedRingImage.getHeight();
        int cropImageWidth = croppedRingImage.getWidth();

        //creating varables to store pixels count
        int nAboveThresholdRed = 1;
        int nAboveThresholdBlue = 1;
        int nAboveThresholdGreen = 1;
        int nR = 0;
        int nG = 0;
        int nB = 0;

        // creating variables to store ratios\
        double dRBRatio = 0.0;
        double dGBRatio = 0.0;
        int nPixel = 0;
        int nRings = 0;

        //looping through all pixels and counting the number of pixels with r, g, or, b above
        // thresholds
        for (int i = 0; i < cropImageHeight; i++) {
            for (int j = 0; j < cropImageWidth; j++) {

                //get pixel value
                nPixel = croppedRingImage.getPixel(j, i);
                nR = Color.red(nPixel);
                nG = Color.green(nPixel);
                nB = Color.blue(nPixel);

                //check r, g, and b against thresholds
                if (nR > nPixelThreshold) {
                    nAboveThresholdRed++;
                }
                if (nB > nPixelThreshold) {
                    nAboveThresholdBlue++;
                }
                if (nG > nPixelThreshold) {
                    nAboveThresholdGreen++;
                }

            }
        }

        //compute ratio r and g to b
        dGBRatio = (double)nAboveThresholdGreen / (double)nAboveThresholdBlue;
        dRBRatio = (double)nAboveThresholdRed / (double)nAboveThresholdBlue;

        //check ratios vs thresholds to decide on number of rings
        //if either ratio is above threshold 2, we will set to 4 rings
        //if either ratio is below threshold 1, we will set to 0 rings
        if ((dGBRatio > dRatioThreshold2) || (dRBRatio > dRatioThreshold2)) {
            nRings = 4;
        }
        else if ((dGBRatio < dRatioThreshold1) || (dRBRatio < dRatioThreshold1)) {
            nRings = 0;
        }
        else {
            nRings = 1;
        }

        // default to return 0 rings
        return nRings;
    }

    // function to return the number of rings based on the bitmap
    public double getRBRatio(Bitmap croppedRingImage, int nPixelThreshold,
                                double dRatioThreshold1, double dRatioThreshold2) {

        // Getting size of cropped bit map
        int cropImageHeight = croppedRingImage.getHeight();
        int cropImageWidth = croppedRingImage.getWidth();

        //creating varables to store pixels count
        int nAboveThresholdRed = 1;
        int nAboveThresholdBlue = 1;
        int nR = 0;
        int nB = 0;

        // creating variables to store ratios\
        double dRBRatio = 0.0;
        int nPixel = 0;

        //looping through all pixels and counting the number of pixels with r, g, or, b above
        // thresholds
        for (int i = 0; i < cropImageHeight; i++) {
            for (int j = 0; j < cropImageWidth; j++) {

                //get pixel value
                nPixel = croppedRingImage.getPixel(j, i);
                nR = Color.red(nPixel);
                nB = Color.blue(nPixel);

                //check r, g, and b against thresholds
                if (nR > nPixelThreshold) {
                    nAboveThresholdRed++;
                }
                if (nB > nPixelThreshold) {
                    nAboveThresholdBlue++;
                }
            }
        }

        //compute ratio r and g to b
        dRBRatio = (double)nAboveThresholdRed / (double)nAboveThresholdBlue;

        // default to return 0 rings
        return dRBRatio;
    }

    // function to return the number of rings based on the bitmap
    public double getGBRatio(Bitmap croppedRingImage, int nPixelThreshold,
                                double dRatioThreshold1, double dRatioThreshold2) {

        // Getting size of cropped bit map
        int cropImageHeight = croppedRingImage.getHeight();
        int cropImageWidth = croppedRingImage.getWidth();

        //creating varables to store pixels count
        int nAboveThresholdBlue = 1;
        int nAboveThresholdGreen = 1;
        int nG = 0;
        int nB = 0;

        // creating variables to store ratios\
        double dGBRatio = 0.0;
        int nPixel = 0;

        //looping through all pixels and counting the number of pixels with r, g, or, b above
        // thresholds
        for (int i = 0; i < cropImageHeight; i++) {
            for (int j = 0; j < cropImageWidth; j++) {

                //get pixel value
                nPixel = croppedRingImage.getPixel(j, i);
                nG = Color.green(nPixel);
                nB = Color.blue(nPixel);

                //check r, g, and b against thresholds
                if (nB > nPixelThreshold) {
                    nAboveThresholdBlue++;
                }
                if (nG > nPixelThreshold) {
                    nAboveThresholdGreen++;
                }

            }
        }

        //compute ratio r and g to b
        dGBRatio = (double)nAboveThresholdGreen / (double)nAboveThresholdBlue;

        // default to return 0 rings
        return dGBRatio;
    }

    public Bitmap createCroppedRingImage() {
        int cropImageHeight = m_cropBoxBottom - m_cropBoxTop;
        int cropImageWidth = m_cropBoxRight - m_cropBoxLeft;
        Bitmap cropBitmap = Bitmap.createBitmap(m_ringBitmap, m_cropBoxLeft, m_cropBoxTop, cropImageWidth, cropImageHeight);
        return cropBitmap;
    }

    // TODO need to fix this to write to correct location so image can be viewed when running
    // on robot
    private void writeBMP(Bitmap bitmapToWrite, String bitmapFullPath) {
        try (FileOutputStream out = new FileOutputStream(bitmapFullPath)) {
            bitmapToWrite.compress(Bitmap.CompressFormat.PNG, 100, out);
            // PNG is a lossless format, the compression factor (100) is ignored
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}



