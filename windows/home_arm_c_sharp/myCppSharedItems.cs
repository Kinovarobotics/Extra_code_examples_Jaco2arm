// Generated using CppHeader2CS
#define KINOVA_TYPE_H_

using System;
using System.Runtime.InteropServices;


namespace C2CS
{
    [Flags]
    public enum RobotTypeEnum
    {
        eRobotType_Error = 255,
        eRobotType_GenericRobot = 4,
        eRobotType_JACOV1_Service = 0,
        eRobotType_Mico6Dof_Service = 1,
        eRobotType_Mico4Dof_Service = 2,
        eRobotType_JacoV2_6Dof_Service = 3,
        eRobotType_JacoV2_4Dof_Service = 4,
        eRobotType_Mico_6DOF_Assistive = 5,
        eRobotType_JacoV2_6DOF_Assistive = 6,
        eRobotType_Spherical_6DOF_Service = 7,
        eRobotType_Spherical_7DOF_Service = 8,
    };

    [Flags]
    public enum POSITION_TYPE
    {
        NOMOVEMENT_POSITION = 0,
        CARTESIAN_POSITION = 1,
        ANGULAR_POSITION = 2,
        RETRACTED = 3,
        PREDEFINED1 = 4,
        PREDEFINED2 = 5,
        PREDEFINED3 = 6,
        CARTESIAN_VELOCITY = 7,
        ANGULAR_VELOCITY = 8,
        PREDEFINED4 = 9,
        PREDEFINED5 = 10,
        ANY_TRAJECTORY = 11,
        TIME_DELAY = 12,
    };

    [Flags]
    public enum PORT_TYPE
    {
        PERIPHERAL_PORT_ANY = 0,
        PERIPHERAL_PORT_CAN_INTERNAL = 1,
        PERIPHERAL_PORT_PORT_CAN_EXTERNAL = 2,
        PERIPHERAL_PORT_PORT_SPI_0 = 3,
        PERIPHERAL_PORT_PORT_SPI_1 = 4,
        PERIPHERAL_PORT_PORT_USB = 5,
        PERIPHERAL_PORT_PORT_UART_0 = 6,
        PERIPHERAL_PORT_PORT_UART_1 = 7,
        PERIPHERAL_PORT_PORT_UART_2 = 8,
        PERIPHERAL_PORT_PORT_VIRTUAL = 9,
    };

    [Flags]
    public enum PERIPHERAL_TYPE
    {
        PERIPHERAL_TYPE_NONE = 0,
        PERIPHERAL_TYPE_ANY = 1,
        PERIPHERAL_TYPE_UNKNOWN = 2,
        PERIPHERAL_TYPE_ACTUATOR_GENERIC = 100,
        PERIPHERAL_TYPE_ACTUATOR_BIG_19NM = 101,
        PERIPHERAL_TYPE_ACTUATOR_BIG_37NM = 102,
        PERIPHERAL_TYPE_ACTUATOR_SMALL_7NM = 103,
        PERIPHERAL_TYPE_ACTUATOR_K75_SINUS = 104,
        PERIPHERAL_TYPE_ACTUATOR_K75PLUS_SINUS = 105,
        PERIPHERAL_TYPE_ACTUATOR_K58_SINUS = 106,
        PERIPHERAL_TYPE_ACTUATOR_A58_SINUS_AURIS = 107,
        PERIPHERAL_TYPE_ACTUATOR_A75PLUS_SINUS_AURIS = 108,
        PERIPHERAL_TYPE_ACTUATOR_A75_SINUS_AURIS = 109,
        PERIPHERAL_TYPE_LINEAR_ACTUATOR_GENERIC = 200,
        PERIPHERAL_TYPE_LINEAR_ACTUATOR_120N = 201,
        PERIPHERAL_TYPE_JOYSTICK = 300,
        PERIPHERAL_TYPE_VIRTUAL_JOYSTICK = 301,
        PERIPHERAL_TYPE_KINOVA_JOYSTICK_3AXIS = 302,
        PERIPHERAL_TYPE_UNIVERSAL_INTERFACE_V2 = 303,
        PERIPHERAL_TYPE_CAN_INTERFACE = 400,
    };

    public enum HAND_MODE
    {
        HAND_NOMOVEMENT,
        POSITION_MODE,
        VELOCITY_MODE,
        NO_FINGER,
        ONE_FINGER,
        TWO_FINGERS,
        THREE_FINGERS,
    };

    public enum ArmLaterality
    {
        RIGHTHAND,
        LEFTHAND,
    };

    [Flags]
    public enum TORQUECONTROL_TYPE
    {
        DIRECTTORQUE = 0,
        IMPEDANCEANGULAR = 1,
        IMPEDANCECARTESIAN = 2,
    };

    public enum GENERALCONTROL_TYPE
    {
        POSITION,
        TORQUE,
    };

    [Flags]
    public enum Controller
    {
        THREE_AXIS_JOYSTICK = 0,
        TWO_AXIS_JOYSTICK = 1,
        API = 2,
        EASY_RIDER = 3,
        UNIVERSAL_INTERFACE = 4,
        EXTERNAL_CUSTOMINTERFACE = 5,
        NONE = 6,
        OLED_DISPLAY = 7,
    };

    [Flags]
    public enum CONTROL_TYPE
    {
        CONTROL_TYPE_CARTESIAN = 0,
        CONTROL_TYPE_ANGULAR = 1,
        CONTROL_TYPE_UNKNOWN = 9999,
    };

    public enum CONTROL_MODULE
    {
        CONTROL_MODULE_NONE,
        CONTROL_MODULE_ANGULAR_VELOCITY,
        CONTROL_MODULE_ANGULAR_POSITION,
        CONTROL_MODULE_CARTESIAN_VELOCITY,
        CONTROL_MODULE_CARTESIAN_POSITION,
        CONTROL_MODULE_RETRACT,
        CONTROL_MODULE_TRAJECTORY,
        CONTROL_MODULE_PREDEFINED,
        CONTROL_MODULE_TIMEDELAY,
    };

    [Flags]
    public enum RETRACT_TYPE
    {
        RETRACT_TYPE_NORMAL_TO_READY = 0,
        RETRACT_TYPE_READY_STANDBY = 1,
        RETRACT_TYPE_READY_TO_RETRACT = 2,
        RETRACT_TYPE_RETRACT_STANDBY = 3,
        RETRACT_TYPE_RETRACT_TO_READY = 4,
        RETRACT_TYPE_NORMAL_STANDBY = 5,
        RETRACT_TYPE_NOT_INITIALIZED = 6,
        RETRACT_ERROR = 25000,
    };

    public struct EthernetConfiguration
    {
        public byte IPAddress[4];
        public byte MacAddress[6];
        public ushort PortNumber;
        public byte Subnet[4];
        public byte Gateway[4];
    } 

    public struct SdkEthernetConfiguration
    {
        public uint IPAddress;
        public uint Subnet;
        public ushort CommandPortNumber;
        public ushort DiscoverPortNumber;
        public ushort RobotPortNumber;
    } 

    public struct TrajectoryFIFO
    {
        public uint TrajectoryCount;
        public float UsedPercentage;
        public uint MaxSize;
    } 

    public struct SingularityVector
    {
        public int TranslationSingularityCount;
        public int OrientationSingularityCount;
        public float TranslationSingularityDistance;
        public float OrientationSingularityDistance;
    } 

    public struct RobotIdentity
    {
        public sbyte SerialNumber[STRING_LENGTH];
        public sbyte Model[STRING_LENGTH];
        public uint CodeVersion;
        public int RobotType;
    } 

    public struct ClientConfigurations
    {
        public sbyte ClientID[STRING_LENGTH];
        public sbyte ClientName[STRING_LENGTH];
        public sbyte Organization[STRING_LENGTH];
        public sbyte Serial[STRING_LENGTH];
        public sbyte Model[STRING_LENGTH];
        public ArmLaterality Laterality;
        public float MaxTranslationVelocity;
        public float MaxOrientationVelocity;
        public float MaxTranslationAcceleration;
        public float MaxOrientationAcceleration;
        public float MaxForce;
        public float Sensibility;
        public float DrinkingHeight;
        public int ComplexRetractActive;
        public float RetractedPositionAngle;
        public int RetractedPositionCount;
        public float DrinkingDistance;
        public int Fingers2and3Inverted;
        public float DrinkingLenght;
        public int DeletePreProgrammedPositionsAtRetract;
        public int EnableFlashErrorLog;
        public int EnableFlashPositionLog;
        public int RobotConfigSelect;
        public int TorqueSensorsEnable;
        public int FrameType;
        public int Expansion[195];
    } 

    [Flags]
    public enum ControlFunctionalityTypeEnum
    {
        CF_NoFunctionality = 0,
        CF_Disable_EnableJoystick = 1,
        CF_Retract_ReadyToUse = 2,
        CF_Change_TwoAxis_ThreeAxis = 3,
        CF_Change_DrinkingMode = 4,
        CF_Cycle_ModeA_list = 5,
        CF_Cycle_ModeB_list = 6,
        CF_DecreaseSpeed = 7,
        CF_IncreaseSpeed = 8,
        CF_Goto_Position1 = 9,
        CF_Goto_Position2 = 10,
        CF_Goto_Position3 = 11,
        CF_Goto_Position4 = 12,
        CF_Goto_Position5 = 13,
        CF_RecordPosition1 = 14,
        CF_RecordPosition2 = 15,
        CF_RecordPosition3 = 16,
        CF_RecordPosition4 = 17,
        CF_RecordPosition5 = 18,
        CF_X_Positive = 19,
        CF_X_Negative = 20,
        CF_Y_Positive = 21,
        CF_Y_Negative = 22,
        CF_Z_Positive = 23,
        CF_Z_Negative = 24,
        CF_R_Positive = 25,
        CF_R_Negative = 26,
        CF_U_Positive = 27,
        CF_U_Negative = 28,
        CF_V_Positive = 29,
        CF_V_Negative = 30,
        CF_OpenHandOneFingers = 31,
        CF_CloseHandOneFingers = 32,
        CF_OpenHandTwoFingers = 33,
        CF_CloseHandTwoFingers = 34,
        CF_OpenHandThreeFingers = 35,
        CF_CloseHandThreeFingers = 36,
        CF_ForceAngularVelocity = 37,
        CF_ForceControlStatus = 38,
        CF_Trajectory = 39,
        CF_AutomaticOrientationXPlus = 40,
        CF_AutomaticOrientationXMinus = 41,
        CF_AutomaticOrientationYPlus = 42,
        CF_AutomaticOrientationYMinus = 43,
        CF_AutomaticOrientationZPlus = 44,
        CF_AutomaticOrientationZMinus = 45,
        CF_AdvanceGOTO_1 = 46,
        CF_AdvanceGOTO_Clear_1 = 47,
        CF_AdvanceGOTO_Add_1 = 48,
        CF_AdvanceGOTO_2 = 49,
        CF_AdvanceGOTO_3 = 50,
        CF_AdvanceGOTO_4 = 51,
        CF_AdvanceGOTO_5 = 52,
        CF_AdvanceGOTO_Clear_2 = 53,
        CF_AdvanceGOTO_Clear_3 = 54,
        CF_AdvanceGOTO_Clear_4 = 55,
        CF_AdvanceGOTO_Clear_5 = 56,
        CF_AdvanceGOTO_add_2 = 57,
        CF_AdvanceGOTO_add_3 = 58,
        CF_AdvanceGOTO_add_4 = 59,
        CF_AdvanceGOTO_add_5 = 60,
        CF_IncreaseSpasmLevel = 61,
        CF_DecreaseSpasmLevel = 62,
        CF_CycleDown_ModeA_list = 63,
        CF_CycleDown_ModeB_list = 64,
        CF_Theta7_Positive = 65,
        CF_Theta7_Negative = 66,
    };

    public struct StickEvents
    {
        public byte Minus;
        public byte Plus;
    } 

    public struct ButtonEvents
    {
        public byte OneClick;
        public byte TwoClick;
        public byte HoldOneSec;
        public byte HoldTwoSec;
        public byte HoldThreeSec;
        public byte HoldFourSec;
        public byte HoldDown;
    } 

    public enum ControlMappingMode
    {
        OneAxis,
        TwoAxis,
        ThreeAxis,
        SixAxis,
    };

    public struct ControlsModeMap
    {
        public int DiagonalsLocked;
        public int Expansion;
        public StickEvents ControlSticks[STICK_EVENT_COUNT];
        public ButtonEvents ControlButtons[BUTTON_EVENT_COUNT];
    } 

    public struct ControlMapping
    {
        public int NumOfModesA;
        public int NumOfModesB;
        public int ActualModeA;
        public int ActualModeB;
        public ControlMappingMode Mode;
        public ControlsModeMap ModeControlsA[MODE_MAP_COUNT];
        public ControlsModeMap ModeControlsB[MODE_MAP_COUNT];
    } 

    public struct ControlMappingCharts
    {
        public int NumOfConfiguredMapping;
        public int ActualControlMapping;
        public ControlMapping Mapping[CONTROL_MAPPING_COUNT];
    } 

    public enum errorLoggerType
    {
        ERROR_NOTINITIALIZED,
        keos_err1,
        keos_err2,
        keos_err3,
        User_err_start_marker,
        errorlog_Actuator_Temperature,
        errorlog_Actuator_TemperatureOK,
        errorlog_Finger_Temperature,
        errorlog_Finger_TemperatureOK,
        errorlog_voltage,
        errorlog_voltageOK,
        errorlog_current_FingersClosing,
        errorlog_current_FingersOpening,
        errorlog_current_FingersOK,
        errorlog_current_Actuators,
        errorlog_current_ActuatorsOK,
        errorLog_RobotStatus_Build_Incomplete,
        errorLogger_END,
    };

    public struct SystemError
    {
        public uint ErrorHeader;
        public errorLoggerType ErrorType;
        public int FirmwareVersion;
        public int KeosVersion;
        public uint SystemTime;
        public bool LayerErrorStatus[ERROR_LAYER_COUNT];
        public int LifeTime;
        public int DataCount;
        public uint Data[ERROR_DATA_COUNT_MAX];
    } 

    public struct ZoneLimitation
    {
        public float speedParameter1;
        public float speedParameter2;
        public float speedParameter3;
        public float forceParameter1;
        public float forceParameter2;
        public float forceParameter3;
        public float accelerationParameter1;
        public float accelerationParameter2;
        public float accelerationParameter3;
    } 

    [Flags]
    public enum ShapeType
    {
        PrismSquareBase_X = 0,
        PrismSquareBase_Y = 1,
        PrismSquareBase_Z = 2,
        PrismTriangularBase_X = 3,
        PrismTriangularBase_Y = 4,
        PrismTriangularBase_Z = 5,
        Pyramid = 6,
    };

    public struct ForcesInfo
    {
        public float Actuator1;
        public float Actuator2;
        public float Actuator3;
        public float Actuator4;
        public float Actuator5;
        public float Actuator6;
        public float Actuator7;
        public float X;
        public float Y;
        public float Z;
        public float ThetaX;
        public float ThetaY;
        public float ThetaZ;
    } 

    public struct QuickStatus
    {
        public byte Finger1Status;
        public byte Finger2Status;
        public byte Finger3Status;
        public byte RetractType;
        public byte RetractComplexity;
        public byte ControlEnableStatus;
        public byte ControlActiveModule;
        public byte ControlFrameType;
        public byte CartesianFaultState;
        public byte ForceControlStatus;
        public byte CurrentLimitationStatus;
        public byte RobotType;
        public byte RobotEdition;
        public byte TorqueSensorsStatus;
    } 

    public struct Finger
    {
        public sbyte ID[STRING_LENGTH];
        public float ActualCommand;
        public float ActualSpeed;
        public float ActualForce;
        public float ActualAcceleration;
        public float ActualCurrent;
        public float ActualPosition;
        public float ActualAverageCurrent;
        public float ActualTemperature;
        public int CommunicationErrors;
        public int OscillatorTuningValue;
        public float CycleCount;
        public float RunTime;
        public float PeakMaxTemp;
        public float PeakMinTemp;
        public float PeakCurrent;
        public float MaxSpeed;
        public float MaxForce;
        public float MaxAcceleration;
        public float MaxCurrent;
        public float MaxAngle;
        public float MinAngle;
        public uint DeviceID;
        public uint CodeVersion;
        public ushort IsFingerInit;
        public ushort Index;
        public ushort FingerAddress;
        public ushort IsFingerConnected;
    } 

    public struct Gripper
    {
        public sbyte Model[STRING_LENGTH];
        public Finger Fingers[JACO_FINGERS_COUNT];
    } 

    public struct ZoneShape
    {
        public ShapeType shapeType;
        public int Expansion1;
    } 

    public struct Zone
    {
        public int ID;
        public int Expansion1;
        public ZoneShape zoneShape;
        public ZoneLimitation zoneLimitation;
        public int Expansion2;
    } 

    public struct ZoneList
    {
        public int NbZones;
        public int Expansion1;
        public Zone Zones[LEGACY_CONFIG_NB_ZONES_MAX];
    } 

    public struct SystemStatus
    {
        public uint JoystickActive;
        public uint RetractStatus;
        public uint DrinkingMode;
        public uint ArmLaterality;
        public uint TranslationActive;
        public uint RotationActive;
        public uint FingersActive;
        public uint WarningOverchargeForce;
        public uint WarningOverchargeFingers;
        public uint WarningLowVoltage;
        public uint MajorErrorOccured;
    } 

    public struct GeneralInformations
    {
        public double TimeAbsolute;
        public double TimeFromStartup;
        public uint IndexStartup;
        public int ExpansionLong1;
        public float TimeStampSavings;
        public float ExpansionFloat;
        public float SupplyVoltage;
        public float TotalCurrent;
        public float Power;
        public float AveragePower;
        public float AccelerationX;
        public float AccelerationY;
        public float AccelerationZ;
        public float SensorExpansion1;
        public float SensorExpansion2;
        public float SensorExpansion3;
        public uint CodeVersion;
        public uint CodeRevision;
        public ushort Status;
        public ushort Controller;
        public ushort ControlMode;
        public ushort HandMode;
        public ushort ConnectedActuatorCount;
        public ushort PositionType;
        public ushort ErrorsSpiExpansion1;
        public ushort ErrorsSpiExpansion2;
        public ushort ErrorsMainSPICount;
        public ushort ErrorsExternalSPICount;
        public ushort ErrorsMainCANCount;
        public ushort ErrorsExternalCANCount;
        public SystemStatus ActualSystemStatus;
        public ZoneLimitation ActualLimitations;
        public float ControlIncrement[MAXACTUATORNUMBER];
        public float FingerControlIncrement[3];
        public uint PeripheralsConnected[4];
        public uint PeripheralsDeviceID[4];
        public float ActuatorsTemperatures[MAXACTUATORNUMBER];
        public float FingersTemperatures[3];
        public float FutureTemperatures[3];
        public int ActuatorsCommErrors[MAXACTUATORNUMBER];
        public int FingersCommErrors[3];
        public int ExpansionLong2;
        public double ControlTimeAbsolute;
        public double ControlTimeFromStartup;
        public byte ExpansionsBytes[192];
    } 

    public struct PeripheralInfo
    {
        public uint Handle;
        public uint Type;
        public uint Port;
        public uint Address;
        public uint CodeVersion;
    } 

    [Flags]
    public enum GRAVITY_TYPE
    {
        MANUAL_INPUT = 0,
        OPTIMAL = 1,
    };

    [Flags]
    public enum ROBOT_TYPE
    {
        JACOV1_ASSISTIVE = 0,
        MICO_6DOF_SERVICE = 1,
        MICO_4DOF_SERVICE = 2,
        JACOV2_6DOF_SERVICE = 3,
        JACOV2_4DOF_SERVICE = 4,
        MICO_6DOF_ASSISTIVE = 5,
        JACOV2_6DOF_ASSISTIVE = 6,
        SPHERICAL_6DOF_SERVICE = 7,
        SPHERICAL_7DOF_SERVICE = 8,
        ROBOT_ERROR = 255,
        GENERIC_ROBOT = 254,
    };

#endif
    class Constants
    {
        public const int ROBOT_CONFIG_ERROR = 255; 
        public const int ROBOT_CONFIG_GENERIC_ROBOT = 4; 
        public const int ROBOT_CONFIG_JACOV1_ASSISTIVE = 0; 
        public const int ROBOT_CONFIG_MICO_6DOF_SERVICE = 1; 
        public const int ROBOT_CONFIG_MICO_4DOF_SERVICE = 2; 
        public const int ROBOT_CONFIG_JACOV2_6DOF_SERVICE = 3; 
        public const int ROBOT_CONFIG_JACOV2_4DOF_SERVICE = 4; 
        public const int ROBOT_CONFIG_MICO_6DOF_ASSISTIVE = 5; 
        public const int ROBOT_CONFIG_JACOV2_6DOF_ASSISTIVE = 6; 
        public const int ROBOT_CONFIG_SPHERICAL_6DOF_SERVICE = 7; 
        public const int ROBOT_CONFIG_SPHERICAL_7DOF_SERVICE = 8; 
        public const int MAXACTUATORNUMBER = 7; 
        public const int JOYSTICK_BUTTON_COUNT = 16; 
        public const int NB_ADVANCE_RETRACT_POSITION = 20; 
        public const int ERROR_DATA_COUNT_MAX = 50; 
        public const int ERROR_LAYER_COUNT = 7; 
        public const int LEGACY_CONFIG_NB_ZONES_MAX = 10; 
        public const int LEGACY_CONFIG_NB_POINTS_COUNT = 8; 
        public const int CONTROL_MAPPING_COUNT = 6; 
        public const int MODE_MAP_COUNT = 6; 
        public const int STICK_EVENT_COUNT = 6; 
        public const int BUTTON_EVENT_COUNT = 26; 
        public const int STRING_LENGTH = 20; 
        public const int JACO_FINGERS_COUNT = 3; 
        public const int ERROR_UNKNOWFILE = 5001; 
        public const int ERROR_MEMORY = 5002; 
        public const int ERROR_FILEREADING = 5003; 
        public const ushort PAGE_SIZE= 2048;
        public const int ADDRESS_PAGE_SIZE= 4;
        public const ushort PACKET_PER_PAGE_QTY= 40;
        public const int PAGEPACKET_SIZE= 52;
        public const int USB_HEADER_SIZE= 8;
        public const int USB_DATA_SIZE= 56;
#endif
    }
}
