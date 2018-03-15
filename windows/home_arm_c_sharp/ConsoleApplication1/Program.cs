using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Runtime.InteropServices;
using System.Diagnostics;
using System.IO;

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

    unsafe public struct EthernetConfiguration
    {
        public fixed byte IPAddress[4];
        public fixed byte MacAddress[6];
        public ushort PortNumber;
        public fixed byte Subnet[4];
        public fixed byte Gateway[4];
    }

    public struct SdkEthernetConfiguration
    {
        public uint IPAddress;
        public uint Subnet;
        public ushort CommandPortNumber;
        public ushort DiscoverPortNumber;
        public ushort RobotPortNumber;
    }

    public struct AngularInfo
    {
        public float Actuator1;
        public float Actuator2;
        public float Actuator3;
        public float Actuator4;
        public float Actuator5;
        public float Actuator6;
        public float Actuator7;

        public void InitStruct()
        {
            Actuator1 = 0.0f;
            Actuator2 = 0.0f;
            Actuator3 = 0.0f;
            Actuator4 = 0.0f;
            Actuator5 = 0.0f;
            Actuator6 = 0.0f;
            Actuator7 = 0.0f;
        }
    }

    public struct CartesianInfo
    {
        public float X;
        public float Y;
        public float Z;
        public float ThetaX;
        public float ThetaY;
        public float ThetaZ;

        public void InitStruct()
        {
            X = 0.0f;
            Y = 0.0f;
            Z = 0.0f;
            ThetaX = 0.0f;
            ThetaY = 0.0f;
            ThetaZ = 0.0f;
        }
    }

    public struct SensorsInfo
    {
        public float Voltage;
        public float Current;
        public float AccelerationX;
        public float AccelerationY;
        public float AccelerationZ;
        public float ActuatorTemp1;
        public float ActuatorTemp2;
        public float ActuatorTemp3;
        public float ActuatorTemp4;
        public float ActuatorTemp5;
        public float ActuatorTemp6;
        public float ActuatorTemp7;
        public float FingerTemp1;
        public float FingerTemp2;
        public float FingerTemp3;

        public void InitStruct()
        {
            Voltage = 0.0f;
            Current = 0.0f;
            AccelerationX = 0.0f;
            AccelerationY = 0.0f;
            AccelerationZ = 0.0f;
            ActuatorTemp1 = 0.0f;
            ActuatorTemp2 = 0.0f;
            ActuatorTemp3 = 0.0f;
            ActuatorTemp4 = 0.0f;
            ActuatorTemp5 = 0.0f;
            ActuatorTemp6 = 0.0f;
            ActuatorTemp7 = 0.0f;
            FingerTemp1 = 0.0f;
            FingerTemp2 = 0.0f;
            FingerTemp3 = 0.0f;
        }
    }

    public struct FingersPosition
    {
        public float Finger1;
        public float Finger2;
        public float Finger3;

        public void InitStruct()
        {
            Finger1 = 0.0f;
            Finger2 = 0.0f;
            Finger3 = 0.0f;
        }
    }

    public struct CartesianPosition
    {
        public CartesianInfo Coordinates;
        public FingersPosition Fingers;

        public void InitStruct()
        {
            Coordinates.InitStruct();
            Fingers.InitStruct();
        }
    }

    public struct AngularPosition
    {
        public AngularInfo Actuators;
        public FingersPosition Fingers;

        void InitStruct()
        {
            Actuators.InitStruct();
            Fingers.InitStruct();
        }
    }

    public struct Limitation
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

        public void InitStruct()
        {
            speedParameter1 = 0.0f;
            speedParameter2 = 0.0f;
            speedParameter3 = 0.0f;
            forceParameter1 = 0.0f;
            forceParameter2 = 0.0f;
            forceParameter3 = 0.0f;
            accelerationParameter1 = 0.0f;
            accelerationParameter2 = 0.0f;
            accelerationParameter3 = 0.0f;
        }
    }

    public struct UserPosition
    {
        public POSITION_TYPE Type;
        public float Delay;
        public CartesianInfo CartesianPosition;
        public AngularInfo Actuators;
        public HAND_MODE HandMode;
        public FingersPosition Fingers;

        public void InitStruct()
        {
            Type = POSITION_TYPE.CARTESIAN_POSITION;
            Delay = 0.0f;
            CartesianPosition.InitStruct();
            Actuators.InitStruct();
            HandMode = HAND_MODE.POSITION_MODE;
            Fingers.InitStruct();
        }
    }

    public struct TrajectoryPoint
    {
        public UserPosition Position;
        public int LimitationsActive;
        public int SynchroType;
        public Limitation Limitations;

        public void InitStruct()
        {
            Position.InitStruct();
            LimitationsActive = 0;
            SynchroType = 0;
            Limitations.InitStruct();
        }
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

    unsafe public struct JoystickCommand
    {
        public static short[] ButtonValue=new short[Constants.JOYSTICK_BUTTON_COUNT];
        public float InclineLeftRight;
        public float InclineForwardBackward;
        public float Rotate;
        public float MoveLeftRight;
        public float MoveForwardBackward;
        public float PushPull;

        public void InitStruct()
        {
            for (int i = 0; i < Constants.JOYSTICK_BUTTON_COUNT; i++)
            {
                ButtonValue[i] = 0;
            }

            InclineLeftRight = 0.0f;
            InclineForwardBackward = 0.0f;
            Rotate = 0.0f;
            MoveLeftRight = 0.0f;
            MoveForwardBackward = 0.0f;
            PushPull = 0.0f;
        }
    }

        unsafe public struct RobotIdentity
    {
        public fixed sbyte SerialNumber[Constants.STRING_LENGTH];
        public fixed sbyte Model[Constants.STRING_LENGTH];
        public uint CodeVersion;
        public int RobotType;
    }

    unsafe public struct ClientConfigurations
    {
        public fixed sbyte ClientID[Constants.STRING_LENGTH];
        public fixed sbyte ClientName[Constants.STRING_LENGTH];
        public fixed sbyte Organization[Constants.STRING_LENGTH];
        public fixed sbyte Serial[Constants.STRING_LENGTH];
        public fixed sbyte Model[Constants.STRING_LENGTH];
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
        public fixed int Expansion[195];
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
        public static StickEvents[] ControlSticks=new StickEvents[Constants.STICK_EVENT_COUNT];
        public static ButtonEvents[] ControlButtons= new ButtonEvents[Constants.BUTTON_EVENT_COUNT];
    }

    unsafe public struct ControlMapping
    {
        public int NumOfModesA;
        public int NumOfModesB;
        public int ActualModeA;
        public int ActualModeB;
        public ControlMappingMode Mode;
        public static ControlsModeMap[] ModeControlsA=new ControlsModeMap[Constants.MODE_MAP_COUNT];
        public static ControlsModeMap[] ModeControlsB=new ControlsModeMap[Constants.MODE_MAP_COUNT];
    }

    unsafe public struct ControlMappingCharts
    {
        public int NumOfConfiguredMapping;
        public int ActualControlMapping;
        public static ControlMapping[] Mapping= new ControlMapping[Constants.CONTROL_MAPPING_COUNT];
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

    unsafe public struct SystemError
    {
        public uint ErrorHeader;
        public errorLoggerType ErrorType;
        public int FirmwareVersion;
        public int KeosVersion;
        public uint SystemTime;
        public fixed bool LayerErrorStatus[Constants.ERROR_LAYER_COUNT];
        public int LifeTime;
        public int DataCount;
        public fixed uint Data[Constants.ERROR_DATA_COUNT_MAX];
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

    unsafe public struct Finger
    {
        public fixed sbyte ID[Constants.STRING_LENGTH];
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

    unsafe public struct Gripper
    {
        public fixed sbyte Model[Constants.STRING_LENGTH];
        public static Finger[] Fingers=new Finger[Constants.JACO_FINGERS_COUNT];
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

    unsafe public struct ZoneList
    {
        public int NbZones;
        public int Expansion1;
        public static Zone[] Zones = new Zone[Constants.LEGACY_CONFIG_NB_ZONES_MAX];
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

    unsafe public struct GeneralInformations
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
        public fixed float ControlIncrement[Constants.MAXACTUATORNUMBER];
        public fixed float FingerControlIncrement[3];
        public fixed uint PeripheralsConnected[4];
        public fixed uint PeripheralsDeviceID[4];
        public fixed float ActuatorsTemperatures[Constants.MAXACTUATORNUMBER];
        public fixed float FingersTemperatures[3];
        public fixed float FutureTemperatures[3];
        public fixed int ActuatorsCommErrors[Constants.MAXACTUATORNUMBER];
        public fixed int FingersCommErrors[3];
        public int ExpansionLong2;
        public double ControlTimeAbsolute;
        public double ControlTimeFromStartup;
        public fixed byte ExpansionsBytes[192];
    }

    public struct AngularAcceleration
    {
        public float Actuator1_X;
        public float Actuator1_Y;
        public float Actuator1_Z;
        public float Actuator2_X;
        public float Actuator2_Y;
        public float Actuator2_Z;
        public float Actuator3_X;
        public float Actuator3_Y;
        public float Actuator3_Z;
        public float Actuator4_X;
        public float Actuator4_Y;
        public float Actuator4_Z;
        public float Actuator5_X;
        public float Actuator5_Y;
        public float Actuator5_Z;
        public float Actuator6_X;
        public float Actuator6_Y;
        public float Actuator6_Z;
        public float Actuator7_X;
        public float Actuator7_Y;
        public float Actuator7_Z;

        public void InitStruct()
        {
            Actuator1_X = 0.0f;
            Actuator1_Y = 0.0f;
            Actuator1_Z = 0.0f;
            Actuator2_X = 0.0f;
            Actuator2_Y = 0.0f;
            Actuator2_Z = 0.0f;
            Actuator3_X = 0.0f;
            Actuator3_Y = 0.0f;
            Actuator3_Z = 0.0f;
            Actuator4_X = 0.0f;
            Actuator4_Y = 0.0f;
            Actuator4_Z = 0.0f;
            Actuator5_X = 0.0f;
            Actuator5_Y = 0.0f;
            Actuator5_Z = 0.0f;
            Actuator6_X = 0.0f;
            Actuator6_Y = 0.0f;
            Actuator6_Z = 0.0f;
            Actuator7_X = 0.0f;
            Actuator7_Y = 0.0f;
            Actuator7_Z = 0.0f;
        }
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

    static class Constants
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
        public const ushort PAGE_SIZE = 2048;
        public const int ADDRESS_PAGE_SIZE = 4;
        public const ushort PACKET_PER_PAGE_QTY = 40;
        public const int PAGEPACKET_SIZE = 52;
        public const int USB_HEADER_SIZE = 8;
        public const int USB_DATA_SIZE = 56;
    }
}

namespace MoveHome
{   
    unsafe class Program
    {
        [DllImport("CommandLayerWindows.dll")]
        public static extern int InitAPI();
        [DllImport("CommandLayerWindows.dll")]
        public static extern int CloseAPI();
        [DllImport("CommandLayerWindows.dll")]
        public static extern int RefresDevicesList();

        [DllImport("CommandLayerWindows.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern int GetCartesianPosition(ref C2CS.CartesianPosition pose);

        [DllImport("CommandLayerWindows.dll")]
        public static extern int SetCartesianControl();

        [DllImport("CommandLayerWindows.dll", CallingConvention = CallingConvention.Cdecl)]
        public static extern int SendBasicTrajectory(C2CS.TrajectoryPoint tp);

        [DllImport("CommandLayerWindows.dll")]
        public static extern int MoveHome();

        static void Main(string[] args)
        {
            string MyValidPassword = "MyValidPassword";
            C2CS.CartesianPosition pose = new C2CS.CartesianPosition();
            pose.InitStruct();
            C2CS.TrajectoryPoint tp=new C2CS.TrajectoryPoint();
            tp.InitStruct();
            try
            {
                int result=InitAPI();
                RefresDevicesList();
                MoveHome();
                SetCartesianControl();
                GetCartesianPosition(ref pose);
                tp.Position.CartesianPosition.X = pose.Coordinates.X-0.10f;
                tp.Position.CartesianPosition.Y = pose.Coordinates.Y;
                tp.Position.CartesianPosition.Z = pose.Coordinates.Z;
                tp.Position.CartesianPosition.ThetaX = pose.Coordinates.ThetaX;
                tp.Position.CartesianPosition.ThetaY = pose.Coordinates.ThetaY;
                tp.Position.CartesianPosition.ThetaZ = pose.Coordinates.ThetaZ;
                tp.Position.Fingers.Finger1 = pose.Fingers.Finger1;
                tp.Position.Fingers.Finger2 = pose.Fingers.Finger2;
                tp.Position.Fingers.Finger3 = pose.Fingers.Finger3;
                SendBasicTrajectory(tp);
                int result2=CloseAPI();
            }
            catch (Exception ex)
            {
                System.Console.WriteLine("Exception during execution of the example. Verify " +
                "your API installation, verify if your Jaco is " +
               "connected and verify that " +
                "you have a valid password.");
            }
            System.Console.WriteLine("End of the example...");
            System.Console.ReadKey();
        }
    }
}
