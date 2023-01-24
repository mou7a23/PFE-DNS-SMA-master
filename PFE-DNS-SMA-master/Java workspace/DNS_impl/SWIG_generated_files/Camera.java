/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 4.0.2
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package SWIG_generated_files;

public class Camera extends Device {
  private transient long swigCPtr;

  protected Camera(long cPtr, boolean cMemoryOwn) {
    super(wrapperJNI.Camera_SWIGUpcast(cPtr), cMemoryOwn);
    swigCPtr = cPtr;
  }

  protected static long getCPtr(Camera obj) {
    return (obj == null) ? 0 : obj.swigCPtr;
  }

  @SuppressWarnings("deprecation")
  protected void finalize() {
    delete();
  }

  public synchronized void delete() {
    if (swigCPtr != 0) {
      if (swigCMemOwn) {
        swigCMemOwn = false;
        wrapperJNI.delete_Camera(swigCPtr);
      }
      swigCPtr = 0;
    }
    super.delete();
  }

  public static int imageGetRed(int[] image, int width, int x, int y){
    return ((image[y * width + x] & 0xff0000) >> 16) & 0xff;
  }

  public static int imageGetGreen(int[] image, int width, int x, int y){
    return ((image[y * width + x] & 0x00ff00) >> 8) & 0xff;
  }

  public static int imageGetBlue(int[] image, int width, int x, int y){
    return image[y * width + x] & 0x0000ff;
  }

  public static int imageGetGray(int[] image, int width, int x, int y){
    int pixel = (image[y * width + x] & 0xffffff);
    return (
            ((pixel >> 16) & 0xff) +
            ((pixel >>  8) & 0xff) +
            (pixel         & 0xff)
           )/3;
  }

  public static int imageGetGrey(int[] image, int width, int x, int y){
    return imageGetGray(image, width, x, y);
  }

  public static int pixelGetRed(int pixel) {
    return ((pixel & 0xff0000) >> 16) & 0xff;
  }

  public static int pixelGetGreen(int pixel) {
    return ((pixel & 0x00ff00) >> 8) & 0xff;
  }

  public static int pixelGetBlue(int pixel) {
    return pixel & 0x0000ff;
  }

  public static int pixelGetGray(int pixel) {
    return (
            ((pixel >> 16) & 0xff) +
            ((pixel >>  8) & 0xff) +
            (pixel         & 0xff)
           )/3;
  }

  public static int pixelGetGrey(int pixel) {
    return pixelGetGray(pixel);
  }

  public CameraRecognitionObject[] getRecognitionObjects() {
    int numberOfObjects = wrapperJNI.Camera_getRecognitionNumberOfObjects(swigCPtr, this);
    CameraRecognitionObject ret[] = new CameraRecognitionObject[numberOfObjects];
    for (int i = 0; i < numberOfObjects; ++i)
      ret[i] = this.getRecognitionObject(i);
    return ret;
  }

  public Camera(String name) {
    this(wrapperJNI.new_Camera(name), true);
  }

  public void enable(int samplingPeriod) {
    wrapperJNI.Camera_enable(swigCPtr, this, samplingPeriod);
  }

  public void disable() {
    wrapperJNI.Camera_disable(swigCPtr, this);
  }

  public int getSamplingPeriod() {
    return wrapperJNI.Camera_getSamplingPeriod(swigCPtr, this);
  }

  public int[] getImage() {
  return wrapperJNI.Camera_getImage(swigCPtr, this);
}

  public int getWidth() {
    return wrapperJNI.Camera_getWidth(swigCPtr, this);
  }

  public int getHeight() {
    return wrapperJNI.Camera_getHeight(swigCPtr, this);
  }

  public double getFov() {
    return wrapperJNI.Camera_getFov(swigCPtr, this);
  }

  public double getMaxFov() {
    return wrapperJNI.Camera_getMaxFov(swigCPtr, this);
  }

  public double getMinFov() {
    return wrapperJNI.Camera_getMinFov(swigCPtr, this);
  }

  public void setFov(double fov) {
    wrapperJNI.Camera_setFov(swigCPtr, this, fov);
  }

  public double getExposure() {
    return wrapperJNI.Camera_getExposure(swigCPtr, this);
  }

  public void setExposure(double exposure) {
    wrapperJNI.Camera_setExposure(swigCPtr, this, exposure);
  }

  public double getFocalLength() {
    return wrapperJNI.Camera_getFocalLength(swigCPtr, this);
  }

  public double getFocalDistance() {
    return wrapperJNI.Camera_getFocalDistance(swigCPtr, this);
  }

  public double getMaxFocalDistance() {
    return wrapperJNI.Camera_getMaxFocalDistance(swigCPtr, this);
  }

  public double getMinFocalDistance() {
    return wrapperJNI.Camera_getMinFocalDistance(swigCPtr, this);
  }

  public void setFocalDistance(double focalDistance) {
    wrapperJNI.Camera_setFocalDistance(swigCPtr, this, focalDistance);
  }

  public double getNear() {
    return wrapperJNI.Camera_getNear(swigCPtr, this);
  }

  public int saveImage(String filename, int quality) {
    return wrapperJNI.Camera_saveImage(swigCPtr, this, filename, quality);
  }

  public boolean hasRecognition() {
    return wrapperJNI.Camera_hasRecognition(swigCPtr, this);
  }

  public void recognitionEnable(int samplingPeriod) {
    wrapperJNI.Camera_recognitionEnable(swigCPtr, this, samplingPeriod);
  }

  public void recognitionDisable() {
    wrapperJNI.Camera_recognitionDisable(swigCPtr, this);
  }

  public int getRecognitionSamplingPeriod() {
    return wrapperJNI.Camera_getRecognitionSamplingPeriod(swigCPtr, this);
  }

  public int getRecognitionNumberOfObjects() {
    return wrapperJNI.Camera_getRecognitionNumberOfObjects(swigCPtr, this);
  }

  private CameraRecognitionObject getRecognitionObjectsPrivate() {
    long cPtr = wrapperJNI.Camera_getRecognitionObjectsPrivate(swigCPtr, this);
    return (cPtr == 0) ? null : new CameraRecognitionObject(cPtr, false);
  }

  public boolean hasRecognitionSegmentation() {
    return wrapperJNI.Camera_hasRecognitionSegmentation(swigCPtr, this);
  }

  public void enableRecognitionSegmentation() {
    wrapperJNI.Camera_enableRecognitionSegmentation(swigCPtr, this);
  }

  public void disableRecognitionSegmentation() {
    wrapperJNI.Camera_disableRecognitionSegmentation(swigCPtr, this);
  }

  public boolean isRecognitionSegmentationEnabled() {
    return wrapperJNI.Camera_isRecognitionSegmentationEnabled(swigCPtr, this);
  }

  public int[] getRecognitionSegmentationImage() {
  return wrapperJNI.Camera_getRecognitionSegmentationImage(swigCPtr, this);
}

  public int saveRecognitionSegmentationImage(String filename, int quality) {
    return wrapperJNI.Camera_saveRecognitionSegmentationImage(swigCPtr, this, filename, quality);
  }

  private CameraRecognitionObject getRecognitionObject(int index) {
    return new CameraRecognitionObject(wrapperJNI.Camera_getRecognitionObject(swigCPtr, this, index), true);
  }

}
