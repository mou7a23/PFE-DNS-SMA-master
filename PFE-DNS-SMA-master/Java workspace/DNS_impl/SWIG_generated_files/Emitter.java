/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 4.0.2
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package SWIG_generated_files;

public class Emitter extends Device {
  private transient long swigCPtr;

  protected Emitter(long cPtr, boolean cMemoryOwn) {
    super(wrapperJNI.Emitter_SWIGUpcast(cPtr), cMemoryOwn);
    swigCPtr = cPtr;
  }

  protected static long getCPtr(Emitter obj) {
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
        wrapperJNI.delete_Emitter(swigCPtr);
      }
      swigCPtr = 0;
    }
    super.delete();
  }

   public int send(byte[] data){
     return send(data,data.length);
   }

  public Emitter(String name) {
    this(wrapperJNI.new_Emitter(name), true);
  }

  public int send(byte[] data, int size) {
    return wrapperJNI.Emitter_send(swigCPtr, this, data, size);
  }

  public int getBufferSize() {
    return wrapperJNI.Emitter_getBufferSize(swigCPtr, this);
  }

  public void setChannel(int channel) {
    wrapperJNI.Emitter_setChannel(swigCPtr, this, channel);
  }

  public int getChannel() {
    return wrapperJNI.Emitter_getChannel(swigCPtr, this);
  }

  public double getRange() {
    return wrapperJNI.Emitter_getRange(swigCPtr, this);
  }

  public void setRange(double range) {
    wrapperJNI.Emitter_setRange(swigCPtr, this, range);
  }

  public final static int CHANNEL_BROADCAST = -1;

}
