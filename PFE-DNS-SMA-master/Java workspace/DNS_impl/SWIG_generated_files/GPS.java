/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 4.0.2
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package SWIG_generated_files;

public class GPS extends Device {
  private transient long swigCPtr;

  protected GPS(long cPtr, boolean cMemoryOwn) {
    super(wrapperJNI.GPS_SWIGUpcast(cPtr), cMemoryOwn);
    swigCPtr = cPtr;
  }

  protected static long getCPtr(GPS obj) {
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
        wrapperJNI.delete_GPS(swigCPtr);
      }
      swigCPtr = 0;
    }
    super.delete();
  }

  public GPS(String name) {
    this(wrapperJNI.new_GPS(name), true);
  }

  public void enable(int samplingPeriod) {
    wrapperJNI.GPS_enable(swigCPtr, this, samplingPeriod);
  }

  public void disable() {
    wrapperJNI.GPS_disable(swigCPtr, this);
  }

  public int getSamplingPeriod() {
    return wrapperJNI.GPS_getSamplingPeriod(swigCPtr, this);
  }

  public double[] getValues() {
    return wrapperJNI.GPS_getValues(swigCPtr, this);
  }

  public double getSpeed() {
    return wrapperJNI.GPS_getSpeed(swigCPtr, this);
  }

  public double[] getSpeedVector() {
    return wrapperJNI.GPS_getSpeedVector(swigCPtr, this);
  }

  public int getCoordinateSystem() {
    return wrapperJNI.GPS_getCoordinateSystem(swigCPtr, this);
  }

  public static String convertToDegreesMinutesSeconds(double decimalDegree) {
    return wrapperJNI.GPS_convertToDegreesMinutesSeconds(decimalDegree);
  }

  // CoordinateSystem 
  public final static int LOCAL = 0;
  public final static int WGS84 = LOCAL + 1;

}
