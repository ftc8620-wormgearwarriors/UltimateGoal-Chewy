package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;

// #################################################################################################
// Start Coach TEST 2021.01.27
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;
// end Coach TEST 2021.01.27
// #################################################################################################


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

    VuforiaLocalizer m_Vuforia = null;  // Should be moved up to member variable section, but keeping in the coach section for now.

    // constructor -  alternate version that is passed a handle to vuforia
    public RingDetector(VuforiaLocalizer vuforia ) {
        m_Vuforia = vuforia;
    }


    // setter for crop box if needs changed
    public void setCropBox(int left, int right, int top, int bottom) {
        m_cropBoxLeft = left;
        m_cropBoxRight = right;
        m_cropBoxTop = top;
        m_cropBoxBottom = bottom;
    }

    // function to return the number of rings based on the bitmap
    public int getNumberOfRings() {

        int nRings = 0;

        // first we need to grab an image from Vuforia and store in our member variable in this class
        // grabs image from the camera
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        VuforiaLocalizer.CloseableFrame closeableFrame = null;
        m_Vuforia.setFrameQueueCapacity(1);
        Image rgbImage = null;
        while (rgbImage == null) {
            try {
                closeableFrame = m_Vuforia.getFrameQueue().take();
                long numImages = closeableFrame.getNumImages();

                for (int i = 0; i < numImages; i++) {
                    if (closeableFrame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                        rgbImage = closeableFrame.getImage(i);
                        if (rgbImage != null) {
                            break;
                        }
                    }
                }
            } catch (InterruptedException exc) {

            } finally {
                if (closeableFrame != null) closeableFrame.close();
            }
        }  // finishes grabbing image

        // if we succesfully grabbed an image convert it from RGB to bmp format and store in our member bitmap
        if (rgbImage != null) {
            // copy the bitmap from the Vuforia frame to our own member variable
            m_ringBitmap = Bitmap.createBitmap(rgbImage.getWidth(), rgbImage.getHeight(), Bitmap.Config.RGB_565);
            m_ringBitmap.copyPixelsFromBuffer(rgbImage.getPixels());


            // create the cropped image and display it
            Bitmap bitmapCroppedRingImage = createCroppedRingImage();
            //saveCroppedBitmap(bitmapCroppedRingImage);

            // Getting size of cropped bit map
            int cropImageHeight = bitmapCroppedRingImage.getHeight();
            int cropImageWidth = bitmapCroppedRingImage.getWidth();

            double dPercent = 0.8;
            int nPixThresh1 = 1000;
            int nPixThresh2 = 5000;
            int nYellowPixels = getMoreRedThanBlue(bitmapCroppedRingImage, dPercent);
            if (nYellowPixels > nPixThresh2) {
                nRings = 4;
            } else if (nYellowPixels < nPixThresh1) {
                nRings = 0;
            } else {
                nRings = 1;
            }
        }
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

    // function to return the number of rings based on the bitmap
    public int getMoreRedThanBlue(Bitmap croppedRingImage, double dPercent) {

        // Getting size of cropped bit map
        int cropImageHeight = croppedRingImage.getHeight();
        int cropImageWidth = croppedRingImage.getWidth();

        //creating variables to store pixels count
        int nYellowPixels = 0;
        int nR = 0;
        int nB = 0;
        int nPixel = 0;

        //looping through all pixels and counting the number of pixels with r, g, or, b above
        // thresholds
        for (int i = 0; i < cropImageHeight; i++) {
            for (int j = 0; j < cropImageWidth; j++) {

                //get pixel values
                nPixel = croppedRingImage.getPixel(j, i);
                nR = Color.red(nPixel);
                nB = Color.blue(nPixel);

                //check b is less then the given percent below r
                double dCompareRValue = (double)nR * dPercent;
                if ((double)nB  < dCompareRValue ) {
                    nYellowPixels++;
                }
            }
        }

        // default to return 0 rings
        return nYellowPixels;
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



