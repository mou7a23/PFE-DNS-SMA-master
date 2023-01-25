/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 4.0.2
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package SWIG_generated_files;

public class Keyboard {
  private transient long swigCPtr;
  protected transient boolean swigCMemOwn;

  protected Keyboard(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  protected static long getCPtr(Keyboard obj) {
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
        wrapperJNI.delete_Keyboard(swigCPtr);
      }
      swigCPtr = 0;
    }
  }

  public Keyboard() {
    this(wrapperJNI.new_Keyboard(), true);
  }

  public void enable(int samplingPeriod) {
    wrapperJNI.Keyboard_enable(swigCPtr, this, samplingPeriod);
  }

  public void disable() {
    wrapperJNI.Keyboard_disable(swigCPtr, this);
  }

  public int getSamplingPeriod() {
    return wrapperJNI.Keyboard_getSamplingPeriod(swigCPtr, this);
  }

  public int getKey() {
    return wrapperJNI.Keyboard_getKey(swigCPtr, this);
  }

  public final static int END = 312;
  public final static int HOME = END + 1;
  public final static int LEFT = HOME + 1;
  public final static int UP = LEFT + 1;
  public final static int RIGHT = UP + 1;
  public final static int DOWN = RIGHT + 1;
  public final static int PAGEUP = 366;
  public final static int PAGEDOWN = PAGEUP + 1;
  public final static int NUMPAD_HOME = 375;
  public final static int NUMPAD_LEFT = NUMPAD_HOME + 1;
  public final static int NUMPAD_UP = NUMPAD_LEFT + 1;
  public final static int NUMPAD_RIGHT = NUMPAD_UP + 1;
  public final static int NUMPAD_DOWN = NUMPAD_RIGHT + 1;
  public final static int NUMPAD_END = 382;
  public final static int KEY = 0x0000ffff;
  public final static int SHIFT = 0x00010000;
  public final static int CONTROL = 0x00020000;
  public final static int ALT = 0x00040000;

}