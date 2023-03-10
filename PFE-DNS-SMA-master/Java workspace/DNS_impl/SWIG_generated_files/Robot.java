/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 4.0.2
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */

package SWIG_generated_files;

public class Robot {
  private transient long swigCPtr;
  protected transient boolean swigCMemOwn;

  protected Robot(long cPtr, boolean cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = cPtr;
  }

  protected static long getCPtr(Robot obj) {
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
        wrapperJNI.delete_Robot(swigCPtr);
      }
      swigCPtr = 0;
    }
  }


  static private Device[] devices = null;
  private Joystick joystick = new Joystick();
  private Keyboard keyboard = new Keyboard();
  private Mouse mouse = new Mouse();

  protected Accelerometer createAccelerometer(String name) {
    return new Accelerometer(name);
  }

  public Accelerometer getAccelerometer(String name) {
    int tag = getDeviceTagFromName(name);
    if (!Device.hasType(tag, Node.ACCELEROMETER))
      return null;
    return (Accelerometer)getOrCreateDevice(tag);
  }

  protected Altimeter createAltimeter(String name) {
    return new Altimeter(name);
  }

  public Altimeter getAltimeter(String name) {
    int tag = getDeviceTagFromName(name);
    if (!Device.hasType(tag, Node.ALTIMETER))
      return null;
    return (Altimeter)getOrCreateDevice(tag);
  }

  protected Brake createBrake(String name) {
    return new Brake(name);
  }

  public Brake getBrake(String name) {
    int tag = getDeviceTagFromName(name);
    if (!Device.hasType(tag, Node.BRAKE))
      return null;
    return (Brake)getOrCreateDevice(tag);
  }

  protected Camera createCamera(String name) {
    return new Camera(name);
  }

  public Camera getCamera(String name) {
    int tag = getDeviceTagFromName(name);
    if (!Device.hasType(tag, Node.CAMERA))
      return null;
    return (Camera)getOrCreateDevice(tag);
  }

  protected Compass createCompass(String name) {
    return new Compass(name);
  }

  public Compass getCompass(String name) {
    int tag = getDeviceTagFromName(name);
    if (!Device.hasType(tag, Node.COMPASS))
      return null;
    return (Compass)getOrCreateDevice(tag);
  }

  protected Connector createConnector(String name) {
    return new Connector(name);
  }

  public Connector getConnector(String name) {
    int tag = getDeviceTagFromName(name);
    if (!Device.hasType(tag, Node.CONNECTOR))
      return null;
    return (Connector)getOrCreateDevice(tag);
  }

  protected Display createDisplay(String name) {
    return new Display(name);
  }

  public Display getDisplay(String name) {
    int tag = getDeviceTagFromName(name);
    if (!Device.hasType(tag, Node.DISPLAY))
      return null;
    return (Display)getOrCreateDevice(tag);
  }

  protected DistanceSensor createDistanceSensor(String name) {
    return new DistanceSensor(name);
  }

  public DistanceSensor getDistanceSensor(String name) {
    int tag = getDeviceTagFromName(name);
    if (!Device.hasType(tag, Node.DISTANCE_SENSOR))
      return null;
    return (DistanceSensor)getOrCreateDevice(tag);
  }

  protected Emitter createEmitter(String name) {
    return new Emitter(name);
  }

  public Emitter getEmitter(String name) {
    int tag = getDeviceTagFromName(name);
    if (!Device.hasType(tag, Node.EMITTER))
      return null;
    return (Emitter)getOrCreateDevice(tag);
  }

  protected GPS createGPS(String name) {
    return new GPS(name);
  }

  public GPS getGPS(String name) {
    int tag = getDeviceTagFromName(name);
    if (!Device.hasType(tag, Node.GPS))
      return null;
    return (GPS)getOrCreateDevice(tag);
  }

  protected Gyro createGyro(String name) {
    return new Gyro(name);
  }

  public Gyro getGyro(String name) {
    int tag = getDeviceTagFromName(name);
    if (!Device.hasType(tag, Node.GYRO))
      return null;
    return (Gyro)getOrCreateDevice(tag);
  }

  protected InertialUnit createInertialUnit(String name) {
    return new InertialUnit(name);
  }

  public InertialUnit getInertialUnit(String name) {
    int tag = getDeviceTagFromName(name);
    if (!Device.hasType(tag, Node.INERTIAL_UNIT))
      return null;
    return (InertialUnit)getOrCreateDevice(tag);
  }

  public Joystick getJoystick() {
    return joystick;
  }

  public Keyboard getKeyboard() {
    return keyboard;
  }

  protected LED createLED(String name) {
    return new LED(name);
  }

  public LED getLED(String name) {
    int tag = getDeviceTagFromName(name);
    if (!Device.hasType(tag, Node.LED))
      return null;
    return (LED)getOrCreateDevice(tag);
  }

  protected Lidar createLidar(String name) {
    return new Lidar(name);
  }

  public Lidar getLidar(String name) {
    int tag = getDeviceTagFromName(name);
    if (!Device.hasType(tag, Node.LIDAR))
      return null;
    return (Lidar)getOrCreateDevice(tag);
  }

  protected LightSensor createLightSensor(String name) {
    return new LightSensor(name);
  }

  public LightSensor getLightSensor(String name) {
    int tag = getDeviceTagFromName(name);
    if (!Device.hasType(tag, Node.LIGHT_SENSOR))
      return null;
    return (LightSensor)getOrCreateDevice(tag);
  }

  protected Motor createMotor(String name) {
    return new Motor(name);
  }

  public Motor getMotor(String name) {
    int tag = getDeviceTagFromName(name);
    if (!Device.hasType(tag, Node.LINEAR_MOTOR) && !Device.hasType(tag, Node.ROTATIONAL_MOTOR))
      return null;
    return (Motor)getOrCreateDevice(tag);
  }

  public Mouse getMouse() {
    return mouse;
  }

  protected Pen createPen(String name) {
    return new Pen(name);
  }

  public Pen getPen(String name) {
    int tag = getDeviceTagFromName(name);
    if (!Device.hasType(tag, Node.PEN))
      return null;
    return (Pen)getOrCreateDevice(tag);
  }

  protected PositionSensor createPositionSensor(String name) {
    return new PositionSensor(name);
  }

  public PositionSensor getPositionSensor(String name) {
    int tag = getDeviceTagFromName(name);
    if (!Device.hasType(tag, Node.POSITION_SENSOR))
      return null;
    return (PositionSensor)getOrCreateDevice(tag);
  }

  protected Radar createRadar(String name) {
    return new Radar(name);
  }

  public Radar getRadar(String name) {
    int tag = getDeviceTagFromName(name);
    if (!Device.hasType(tag, Node.RADAR))
      return null;
    return (Radar)getOrCreateDevice(tag);
  }

  protected RangeFinder createRangeFinder(String name) {
    return new RangeFinder(name);
  }

  public RangeFinder getRangeFinder(String name) {
    int tag = getDeviceTagFromName(name);
    if (!Device.hasType(tag, Node.RANGE_FINDER))
      return null;
    return (RangeFinder)getOrCreateDevice(tag);
  }

  protected Receiver createReceiver(String name) {
    return new Receiver(name);
  }

  public Receiver getReceiver(String name) {
    int tag = getDeviceTagFromName(name);
    if (!Device.hasType(tag, Node.RECEIVER))
      return null;
    return (Receiver)getOrCreateDevice(tag);
  }

  protected Skin createSkin(String name) {
    return new Skin(name);
  }

  public Skin getSkin(String name) {
    int tag = getDeviceTagFromName(name);
    if (!Device.hasType(tag, Node.SKIN))
      return null;
    return (Skin)getOrCreateDevice(tag);
  }

  protected Speaker createSpeaker(String name) {
    return new Speaker(name);
  }

  public Speaker getSpeaker(String name) {
    int tag = getDeviceTagFromName(name);
    if (!Device.hasType(tag, Node.SPEAKER))
      return null;
    return (Speaker)getOrCreateDevice(tag);
  }

  protected TouchSensor createTouchSensor(String name) {
    return new TouchSensor(name);
  }

  public TouchSensor getTouchSensor(String name) {
    int tag = getDeviceTagFromName(name);
    if (!Device.hasType(tag, Node.TOUCH_SENSOR))
      return null;
    return (TouchSensor)getOrCreateDevice(tag);
  }

  public Device getDeviceByIndex(int index) {
    return getOrCreateDevice(getDeviceTagFromIndex(index));
  }

  static public Device getDevice(int tag) {
    if (tag == 0 || devices == null || tag >= devices.length)
      return null;
    return devices[tag];
  }

  private Device getOrCreateDevice(int tag) {
    if (tag == 0)
      return null;

    int count = getNumberOfDevices();
    // if new devices have been added, then count is greater than devices.length
    // deleted devices are not removed from the C API list and don't affect the number of devices
    if (devices != null && devices.length == count + 1 && tag < devices.length)
        return devices[tag];

    // (re-)initialize devices list
    if (tag > count)
        return null;
    devices = new Device[count + 1];
    for (int i = 0; i < count; i++) {
      int otherTag = getDeviceTagFromIndex(i);
      String name = getDeviceNameFromTag(otherTag);
      switch(getDeviceTypeFromTag(otherTag)) {
        case Node.ACCELEROMETER:    devices[otherTag] = createAccelerometer(name); break;
        case Node.ALTIMETER:        devices[otherTag] = createAltimeter(name); break;
        case Node.BRAKE:            devices[otherTag] = createBrake(name); break;
        case Node.CAMERA:           devices[otherTag] = createCamera(name); break;
        case Node.COMPASS:          devices[otherTag] = createCompass(name); break;
        case Node.CONNECTOR:        devices[otherTag] = createConnector(name); break;
        case Node.DISPLAY:          devices[otherTag] = createDisplay(name); break;
        case Node.DISTANCE_SENSOR:  devices[otherTag] = createDistanceSensor(name); break;
        case Node.EMITTER:          devices[otherTag] = createEmitter(name); break;
        case Node.GPS:              devices[otherTag] = createGPS(name); break;
        case Node.GYRO:             devices[otherTag] = createGyro(name); break;
        case Node.INERTIAL_UNIT:    devices[otherTag] = createInertialUnit(name); break;
        case Node.LED:              devices[otherTag] = createLED(name); break;
        case Node.LIDAR:            devices[otherTag] = createLidar(name); break;
        case Node.LIGHT_SENSOR:     devices[otherTag] = createLightSensor(name); break;
        case Node.LINEAR_MOTOR:
        case Node.ROTATIONAL_MOTOR: devices[otherTag] = createMotor(name); break;
        case Node.PEN:              devices[otherTag] = createPen(name); break;
        case Node.POSITION_SENSOR:  devices[otherTag] = createPositionSensor(name); break;
        case Node.RADAR:            devices[otherTag] = createRadar(name); break;
        case Node.RANGE_FINDER:     devices[otherTag] = createRangeFinder(name); break;
        case Node.RECEIVER:         devices[otherTag] = createReceiver(name); break;
        case Node.SKIN:             devices[otherTag] = createSkin(name); break;
        case Node.SPEAKER:          devices[otherTag] = createSpeaker(name); break;
        case Node.TOUCH_SENSOR:     devices[otherTag] = createTouchSensor(name); break;
        default:                    devices[otherTag] = null; break;
      }
    }
    return devices[tag];
  }

  public Robot() {
    this(wrapperJNI.new_Robot(), true);
  }

  public static Robot internalGetInstance() {
    long cPtr = wrapperJNI.Robot_internalGetInstance();
    return (cPtr == 0) ? null : new Robot(cPtr, false);
  }

  public int step(int duration) {
    return wrapperJNI.Robot_step(swigCPtr, this, duration);
  }

  public int waitForUserInputEvent(int event_type, int timeout) {
    return wrapperJNI.Robot_waitForUserInputEvent(swigCPtr, this, event_type, timeout);
  }

  public String getName() {
    return wrapperJNI.Robot_getName(swigCPtr, this);
  }

  public String getUrdf(String prefix) {
    return wrapperJNI.Robot_getUrdf__SWIG_0(swigCPtr, this, prefix);
  }

  public String getUrdf() {
    return wrapperJNI.Robot_getUrdf__SWIG_1(swigCPtr, this);
  }

  public double getTime() {
    return wrapperJNI.Robot_getTime(swigCPtr, this);
  }

  public String getModel() {
    return wrapperJNI.Robot_getModel(swigCPtr, this);
  }

  public String getCustomData() {
    return wrapperJNI.Robot_getCustomData(swigCPtr, this);
  }

  public void setCustomData(String data) {
    wrapperJNI.Robot_setCustomData(swigCPtr, this, data);
  }

  public int getMode() {
    return wrapperJNI.Robot_getMode(swigCPtr, this);
  }

  public void setMode(int arg0, String arg1) {
    wrapperJNI.Robot_setMode(swigCPtr, this, arg0, arg1);
  }

  public boolean getSupervisor() {
    return wrapperJNI.Robot_getSupervisor(swigCPtr, this);
  }

  public boolean getSynchronization() {
    return wrapperJNI.Robot_getSynchronization(swigCPtr, this);
  }

  public String getProjectPath() {
    return wrapperJNI.Robot_getProjectPath(swigCPtr, this);
  }

  public String getWorldPath() {
    return wrapperJNI.Robot_getWorldPath(swigCPtr, this);
  }

  public double getBasicTimeStep() {
    return wrapperJNI.Robot_getBasicTimeStep(swigCPtr, this);
  }

  public int getNumberOfDevices() {
    return wrapperJNI.Robot_getNumberOfDevices(swigCPtr, this);
  }

  private Device getDeviceByIndexPrivate(int index) {
    long cPtr = wrapperJNI.Robot_getDeviceByIndexPrivate(swigCPtr, this, index);
    return (cPtr == 0) ? null : new Device(cPtr, false);
  }

  public Device getDevice(String name) {
    long cPtr = wrapperJNI.Robot_getDevice(swigCPtr, this, name);
    return (cPtr == 0) ? null : new Device(cPtr, false);
  }

  public int getType() {
    return wrapperJNI.Robot_getType(swigCPtr, this);
  }

  public void batterySensorEnable(int samplingPeriod) {
    wrapperJNI.Robot_batterySensorEnable(swigCPtr, this, samplingPeriod);
  }

  public void batterySensorDisable() {
    wrapperJNI.Robot_batterySensorDisable(swigCPtr, this);
  }

  public int batterySensorGetSamplingPeriod() {
    return wrapperJNI.Robot_batterySensorGetSamplingPeriod(swigCPtr, this);
  }

  public double batterySensorGetValue() {
    return wrapperJNI.Robot_batterySensorGetValue(swigCPtr, this);
  }

  public void wwiSendText(String text) {
    wrapperJNI.Robot_wwiSendText(swigCPtr, this, text);
  }

  public String wwiReceiveText() {
    return wrapperJNI.Robot_wwiReceiveText(swigCPtr, this);
  }

  public String getData() {
    return wrapperJNI.Robot_getData(swigCPtr, this);
  }

  public void setData(String data) {
    wrapperJNI.Robot_setData(swigCPtr, this, data);
  }

  public static Device getDeviceFromTag(int tag) {
    long cPtr = wrapperJNI.Robot_getDeviceFromTag(tag);
    return (cPtr == 0) ? null : new Device(cPtr, false);
  }

  private static int getDeviceTypeFromTag(int tag) {
    return wrapperJNI.Robot_getDeviceTypeFromTag(tag);
  }

  private static String getDeviceNameFromTag(int tag) {
    return wrapperJNI.Robot_getDeviceNameFromTag(tag);
  }

  private static int getDeviceTagFromIndex(int index) {
    return wrapperJNI.Robot_getDeviceTagFromIndex(index);
  }

  public static int getDeviceTagFromName(String name) {
    return wrapperJNI.Robot_getDeviceTagFromName(name);
  }

  // Mode 
  public final static int MODE_SIMULATION = 0;
  public final static int MODE_CROSS_COMPILATION = MODE_SIMULATION + 1;
  public final static int MODE_REMOTE_CONTROL = MODE_CROSS_COMPILATION + 1;

  // UserInputEvent 
  public final static int EVENT_QUIT = -1;
  public final static int EVENT_NO_EVENT = 0;
  public final static int EVENT_MOUSE_CLICK = 1;
  public final static int EVENT_MOUSE_MOVE = 2;
  public final static int EVENT_KEYBOARD = 4;
  public final static int EVENT_JOYSTICK_BUTTON = 8;
  public final static int EVENT_JOYSTICK_AXIS = 16;
  public final static int EVENT_JOYSTICK_POV = 32;

}
