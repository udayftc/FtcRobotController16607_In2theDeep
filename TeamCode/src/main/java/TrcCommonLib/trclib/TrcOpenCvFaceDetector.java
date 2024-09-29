

package TrcCommonLib.trclib;

import org.opencv.core.Mat;
import org.opencv.core.MatOfRect;
import org.opencv.core.Rect;
import org.opencv.objdetect.CascadeClassifier;

/**
 * This class implements an OpenCV face detector using the provided classifier.
 */
public abstract class TrcOpenCvFaceDetector extends TrcOpenCvDetector
{
    /**
     * This class encapsulates info of the detected object. It extends TrcOpenCvDetector.DetectedObject that requires
     * it to provide a method to return the detected object rect and area.
     */
    public static class DetectedObject extends TrcOpenCvDetector.DetectedObject<Rect>
    {
        /**
         * Constructor: Creates an instance of the object.
         *
         * @param label specifies the object label.
         * @param rect specifies the rect of the detected object.
         */
        public DetectedObject(String label, Rect rect)
        {
            super(label, rect);
        }   //DetectedObject

        /**
         * This method returns the rect of the detected object.
         *
         * @return rect of the detected object.
         */
        @Override
        public Rect getRect()
        {
            return object;
        }   //getRect

        /**
         * This method returns the area of the detected object.
         *
         * @return area of the detected object.
         */
        @Override
        public double getArea()
        {
            return object.area();
        }   //getArea

        /**
         * This method returns the pose of the detected object relative to the camera.
         *
         * @return pose of the detected object relative to camera.
         */
        @Override
        public TrcPose3D getObjectPose()
        {
            // Face detection does not provide detected object pose, let caller use homography to calculate it.
            return null;
        }   //getObjectPose

        /**
         * This method returns the real world width of the detected object.
         *
         * @return real world width of the detected object.
         */
        @Override
        public Double getObjectWidth()
        {
            // OpenCvFace detection does not provide detected object width, let caller use homography to calculate it.
            return null;
        }   //getObjectWidth

        /**
         * This method returns the real world depth of the detected object.
         *
         * @return real world depth of the detected object.
         */
        @Override
        public Double getObjectDepth()
        {
            // OpenCvFace detection does not provide detected object depth, let caller use homography to calculate it.
            return null;
        }   //getObjectDepth

    }   //class DetectedObject

    private final CascadeClassifier faceDetector;
    private final MatOfRect detectedFaceBuffer;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param classifierPath specifies the file path for the classifier.
     * @param numImageBuffers specifies the number of image buffers to allocate.
     */
    public TrcOpenCvFaceDetector(String instanceName, String classifierPath, int numImageBuffers)
    {
        super(instanceName, numImageBuffers, null, null);
        faceDetector = new CascadeClassifier(classifierPath);
        if (faceDetector.empty())
        {
            throw new RuntimeException("Failed to load Cascade Classifier <" + classifierPath + ">");
        }
        //
        // Preallocate detectedFaceBuffer.
        //
        detectedFaceBuffer = new MatOfRect();
    }   //TrcOpenCvFaceDetector

    //
    // Implements the TrcVisionTask.VisionProcesor interface.
    //

    /**
     * This method is called to process an image frame to detect objects in the acquired frame.
     *
     * @param image specifies the image to be processed.
     * @return detected objects, null if none detected.
     */
    @Override
    public DetectedObject[] processFrame(Mat image)
    {
        DetectedObject[] targets = null;

        faceDetector.detectMultiScale(image, detectedFaceBuffer);
        if (!detectedFaceBuffer.empty())
        {
            Rect[] faceRects = detectedFaceBuffer.toArray();
            targets = new DetectedObject[faceRects.length];
            for (int i = 0; i < targets.length; i++)
            {
                targets[i] = new DetectedObject(instanceName, faceRects[i]);
            }
        }

        return targets;
    }   //processFrame

}   //class TrcOpenCvFaceDetector
