/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 4.0.2
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package SWIG_generated_files;

public class TouchSensor extends Device {
  private transient long swigCPtr;

  protected TouchSensor(long cPtr, boolean cMemoryOwn) {
    super(wrapperJNI.TouchSensor_SWIGUpcast(cPtr), cMemoryOwn);
    swigCPtr = cPtr;
  }

  protected static long getCPtr(TouchSensor obj) {
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
        wrapperJNI.delete_TouchSensor(swigCPtr);
      }
      swigCPtr = 0;
    }
    super.delete();
  }

  public TouchSensor(String name) {
    this(wrapperJNI.new_TouchSensor(name), true);
  }

  public void enable(int samplingPeriod) {
    wrapperJNI.TouchSensor_enable(swigCPtr, this, samplingPeriod);
  }

  public void disable() {
    wrapperJNI.TouchSensor_disable(swigCPtr, this);
  }

  public int getSamplingPeriod() {
    return wrapperJNI.TouchSensor_getSamplingPeriod(swigCPtr, this);
  }

  public double getValue() {
    return wrapperJNI.TouchSensor_getValue(swigCPtr, this);
  }

  public double[] getValues() {
    return wrapperJNI.TouchSensor_getValues(swigCPtr, this);
  }

  private int getLookupTableSize() {
    return wrapperJNI.TouchSensor_getLookupTableSize(swigCPtr, this);
  }

  public double[] getLookupTable() {
    return wrapperJNI.TouchSensor_getLookupTable(swigCPtr, this);
  }

  public int getType() {
    return wrapperJNI.TouchSensor_getType(swigCPtr, this);
  }

  // Type 
  public final static int BUMPER = 0;
  public final static int FORCE = BUMPER + 1;
  public final static int FORCE3D = FORCE + 1;

}
