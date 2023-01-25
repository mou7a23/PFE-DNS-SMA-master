/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 4.0.2
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package SWIG_generated_files;

public class Mouse {
  private transient long swigCPtr;
  protected transient boolean swigCMemOwn;

  protected Mouse(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  protected static long getCPtr(Mouse obj) {
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
        wrapperJNI.delete_Mouse(swigCPtr);
      }
      swigCPtr = 0;
    }
  }

  public Mouse() {
    this(wrapperJNI.new_Mouse(), true);
  }

  public void enable(int samplingPeriod) {
    wrapperJNI.Mouse_enable(swigCPtr, this, samplingPeriod);
  }

  public void disable() {
    wrapperJNI.Mouse_disable(swigCPtr, this);
  }

  public int getSamplingPeriod() {
    return wrapperJNI.Mouse_getSamplingPeriod(swigCPtr, this);
  }

  public void enable3dPosition() {
    wrapperJNI.Mouse_enable3dPosition(swigCPtr, this);
  }

  public void disable3dPosition() {
    wrapperJNI.Mouse_disable3dPosition(swigCPtr, this);
  }

  public boolean is3dPositionEnabled() {
    return wrapperJNI.Mouse_is3dPositionEnabled(swigCPtr, this);
  }

  public MouseState getState() {
    return new MouseState(wrapperJNI.Mouse_getState(swigCPtr, this), true);
  }

}