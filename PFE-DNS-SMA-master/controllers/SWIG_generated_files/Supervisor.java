/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 4.0.2
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package SWIG_generated_files;

public class Supervisor extends Robot {
  private transient long swigCPtr;

  protected Supervisor(long cPtr, boolean cMemoryOwn) {
    super(wrapperJNI.Supervisor_SWIGUpcast(cPtr), cMemoryOwn);
    swigCPtr = cPtr;
  }

  protected static long getCPtr(Supervisor obj) {
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
        wrapperJNI.delete_Supervisor(swigCPtr);
      }
      swigCPtr = 0;
    }
    super.delete();
  }

  public Node getRoot() {
    long cPtr = wrapperJNI.Supervisor_getRootPrivate(swigCPtr, this);
    return Node.findNode(cPtr);
  }

  public Node getSelf() {
    long cPtr = wrapperJNI.Supervisor_getSelfPrivate(swigCPtr, this);
    return Node.findNode(cPtr);
  }

  public Node getFromDef(String name) {
    long cPtr = wrapperJNI.Supervisor_getFromDefPrivate(swigCPtr, this, name);
    return Node.findNode(cPtr);
  }

  public Node getFromId(int id) {
    long cPtr = wrapperJNI.Supervisor_getFromIdPrivate(swigCPtr, this, id);
    return Node.findNode(cPtr);
  }

  public Node getFromDevice(Device device) {
    return getFromDeviceTag(device.getTag());
  }

  private Node getFromDeviceTag(int tag) {
    long cPtr = wrapperJNI.Supervisor_getFromDeviceTagPrivate(swigCPtr, this, tag);
    return Node.findNode(cPtr);
  }

  public Node getSelected() {
    long cPtr = wrapperJNI.Supervisor_getSelectedPrivate(swigCPtr, this);
    return Node.findNode(cPtr);
  }

  public Supervisor() {
    this(wrapperJNI.new_Supervisor(), true);
  }

  public void simulationQuit(int status) {
    wrapperJNI.Supervisor_simulationQuit(swigCPtr, this, status);
  }

  public void simulationReset() {
    wrapperJNI.Supervisor_simulationReset(swigCPtr, this);
  }

  public void simulationResetPhysics() {
    wrapperJNI.Supervisor_simulationResetPhysics(swigCPtr, this);
  }

  public int simulationGetMode() {
    return wrapperJNI.Supervisor_simulationGetMode(swigCPtr, this);
  }

  public void simulationSetMode(int mode) {
    wrapperJNI.Supervisor_simulationSetMode(swigCPtr, this, mode);
  }

  public void worldLoad(String file) {
    wrapperJNI.Supervisor_worldLoad(swigCPtr, this, file);
  }

  public void worldReload() {
    wrapperJNI.Supervisor_worldReload(swigCPtr, this);
  }

  public boolean worldSave() {
    return wrapperJNI.Supervisor_worldSave__SWIG_0(swigCPtr, this);
  }

  public boolean worldSave(String file) {
    return wrapperJNI.Supervisor_worldSave__SWIG_1(swigCPtr, this, file);
  }

  public void exportImage(String file, int quality) {
    wrapperJNI.Supervisor_exportImage(swigCPtr, this, file, quality);
  }

  public boolean animationStartRecording(String file) {
    return wrapperJNI.Supervisor_animationStartRecording(swigCPtr, this, file);
  }

  public boolean animationStopRecording() {
    return wrapperJNI.Supervisor_animationStopRecording(swigCPtr, this);
  }

  public void movieStartRecording(String file, int width, int height, int codec, int quality, int acceleration, boolean caption) {
    wrapperJNI.Supervisor_movieStartRecording(swigCPtr, this, file, width, height, codec, quality, acceleration, caption);
  }

  public void movieStopRecording() {
    wrapperJNI.Supervisor_movieStopRecording(swigCPtr, this);
  }

  public boolean movieIsReady() {
    return wrapperJNI.Supervisor_movieIsReady(swigCPtr, this);
  }

  public boolean movieFailed() {
    return wrapperJNI.Supervisor_movieFailed(swigCPtr, this);
  }

  public void setLabel(int id, String label, double xpos, double ypos, double size, int color, double transparency, String font) {
    wrapperJNI.Supervisor_setLabel__SWIG_0(swigCPtr, this, id, label, xpos, ypos, size, color, transparency, font);
  }

  public void setLabel(int id, String label, double xpos, double ypos, double size, int color, double transparency) {
    wrapperJNI.Supervisor_setLabel__SWIG_1(swigCPtr, this, id, label, xpos, ypos, size, color, transparency);
  }

  private Node getRootPrivate() {
    long cPtr = wrapperJNI.Supervisor_getRootPrivate(swigCPtr, this);
    return (cPtr == 0) ? null : new Node(cPtr, false);
  }

  private Node getSelfPrivate() {
    long cPtr = wrapperJNI.Supervisor_getSelfPrivate(swigCPtr, this);
    return (cPtr == 0) ? null : new Node(cPtr, false);
  }

  private Node getFromDefPrivate(String name) {
    long cPtr = wrapperJNI.Supervisor_getFromDefPrivate(swigCPtr, this, name);
    return (cPtr == 0) ? null : new Node(cPtr, false);
  }

  private Node getFromIdPrivate(int id) {
    long cPtr = wrapperJNI.Supervisor_getFromIdPrivate(swigCPtr, this, id);
    return (cPtr == 0) ? null : new Node(cPtr, false);
  }

  private Node getFromDevicePrivate(Device device) {
    long cPtr = wrapperJNI.Supervisor_getFromDevicePrivate(swigCPtr, this, Device.getCPtr(device), device);
    return (cPtr == 0) ? null : new Node(cPtr, false);
  }

  private Node getFromDeviceTagPrivate(int tag) {
    long cPtr = wrapperJNI.Supervisor_getFromDeviceTagPrivate(swigCPtr, this, tag);
    return (cPtr == 0) ? null : new Node(cPtr, false);
  }

  private Node getSelectedPrivate() {
    long cPtr = wrapperJNI.Supervisor_getSelectedPrivate(swigCPtr, this);
    return (cPtr == 0) ? null : new Node(cPtr, false);
  }

  public boolean virtualRealityHeadsetIsUsed() {
    return wrapperJNI.Supervisor_virtualRealityHeadsetIsUsed(swigCPtr, this);
  }

  public double[] virtualRealityHeadsetGetPosition() {
    return wrapperJNI.Supervisor_virtualRealityHeadsetGetPosition(swigCPtr, this);
  }

  public double[] virtualRealityHeadsetGetOrientation() {
    return wrapperJNI.Supervisor_virtualRealityHeadsetGetOrientation(swigCPtr, this);
  }

  public void simulationRevert() {
    wrapperJNI.Supervisor_simulationRevert(swigCPtr, this);
  }

  public void loadWorld(String file) {
    wrapperJNI.Supervisor_loadWorld(swigCPtr, this, file);
  }

  public boolean saveWorld() {
    return wrapperJNI.Supervisor_saveWorld__SWIG_0(swigCPtr, this);
  }

  public boolean saveWorld(String file) {
    return wrapperJNI.Supervisor_saveWorld__SWIG_1(swigCPtr, this, file);
  }

  public void simulationPhysicsReset() {
    wrapperJNI.Supervisor_simulationPhysicsReset(swigCPtr, this);
  }

  public void startMovie(String file, int width, int height, int codec, int quality, int acceleration, boolean caption) {
    wrapperJNI.Supervisor_startMovie(swigCPtr, this, file, width, height, codec, quality, acceleration, caption);
  }

  public void stopMovie() {
    wrapperJNI.Supervisor_stopMovie(swigCPtr, this);
  }

  public int getMovieStatus() {
    return wrapperJNI.Supervisor_getMovieStatus(swigCPtr, this);
  }

  public int movieGetStatus() {
    return wrapperJNI.Supervisor_movieGetStatus(swigCPtr, this);
  }

  // SimulationMode 
  public final static int SIMULATION_MODE_PAUSE = 0;
  public final static int SIMULATION_MODE_REAL_TIME = SIMULATION_MODE_PAUSE + 1;
  public final static int SIMULATION_MODE_FAST = SIMULATION_MODE_REAL_TIME + 1;

  public final static int MOVIE_READY = 0;
  public final static int MOVIE_RECORDING = MOVIE_READY + 1;
  public final static int MOVIE_SAVING = MOVIE_RECORDING + 1;
  public final static int MOVIE_WRITE_ERROR = MOVIE_SAVING + 1;
  public final static int MOVIE_ENCODING_ERROR = MOVIE_WRITE_ERROR + 1;
  public final static int MOVIE_SIMULATION_ERROR = MOVIE_ENCODING_ERROR + 1;

}