/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 4.0.2
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package SWIG_generated_files;

public class LED extends Device {
  private transient long swigCPtr;

  protected LED(long cPtr, boolean cMemoryOwn) {
    super(wrapperJNI.LED_SWIGUpcast(cPtr), cMemoryOwn);
    swigCPtr = cPtr;
  }

  protected static long getCPtr(LED obj) {
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
        wrapperJNI.delete_LED(swigCPtr);
      }
      swigCPtr = 0;
    }
    super.delete();
  }

  public LED(String name) {
    this(wrapperJNI.new_LED(name), true);
  }

  public void set(int value) {
    wrapperJNI.LED_set(swigCPtr, this, value);
  }

  public int get() {
    return wrapperJNI.LED_get(swigCPtr, this);
  }

}