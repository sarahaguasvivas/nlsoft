/* THIS FILE HAS BEEN AUTOMATICALLY GENERATED */
/* DO NOT MODIFY IT DIRECTLY AND EXPECT THINGS TO WORK */
/* version: 0.7.0 */



/* -- general -- */
static const unsigned char PROTOCOL_VERSION[3] = {0, 7, 0};
static const unsigned char PKT_START = 58;
static const unsigned char PKT_END = 33;
static const unsigned char PKT_MINSIZE = 6;
static const unsigned char PKT_MAXSIZE = 64;
static const unsigned char ADDR_MAX = 16;
static const unsigned char TRGT_MAX = 16;



/* -- context -- */
static const unsigned char CTX_ASK = 7;
static const unsigned char CTX_CONFIG = 4;
static const unsigned char CTX_DATA = 2;
static const unsigned char CTX_NULL = 0;
static const unsigned char CTX_RUN = 5;
static const unsigned char CTX_SET = 6;
static const unsigned char CTX_STATUS = 1;
static const unsigned char CTX_SYNC = 8;
static const unsigned char CTX_TIME = 3;
static const unsigned char CTX_UNSYNC = 9;



/* -- status -- */
static const unsigned char STS_ADR_ERR = 4;
static const unsigned char STS_CMD_ERR = 6;
static const unsigned char STS_CTX_ERR = 5;
static const unsigned char STS_DEC_ERR = 9;
static const unsigned char STS_ENC_ERR = 8;
static const unsigned char STS_GEN_ERR = 2;
static const unsigned char STS_NULL = 0;
static const unsigned char STS_OK = 1;
static const unsigned char STS_PKT_ERR = 3;
static const unsigned char STS_SYNC_ERR = 10;
static const unsigned char STS_TGT_ERR = 7;



/* -- config -- */
static const unsigned char CFG_ANALOG_IN = 5;
static const unsigned char CFG_ANALOG_OUT = 6;
static const unsigned char CFG_CORE = 1;
static const unsigned char CFG_DIGITAL_IN = 3;
static const unsigned char CFG_DIGITAL_OUT = 4;
static const unsigned char CFG_DRIVER = 7;
static const unsigned char CFG_NULL = 0;
static const unsigned char CFG_SENSOR = 8;
static const unsigned char CFG_TESTING = 2;
static const unsigned char CFG_ULTRAVOLT = 9;



/* -- targets -- */

/* -- CoreChannel targets -- */
static const unsigned char TGT_LED = 1;
static const unsigned char TGT_N_CHNL = 2;
static const unsigned char TGT_CLOCK = 3;
static const unsigned char TGT_BLINK_USEC = 4;
static const unsigned char TGT_SPIN_USEC = 5;
static const unsigned char TGT_STDBY_USEC = 6;
static const unsigned char TGT_SYNC_USEC = 7;

/* -- DriverChannel targets -- */
static const unsigned char TGT_IS_CTRL = 1;
static const unsigned char TGT_CHG_PWM = 2;
static const unsigned char TGT_DRN_PWM = 3;
static const unsigned char TGT_V_REF = 4;
static const unsigned char TGT_V_MON = 5;

/* -- AnalogInChannel targets -- */
static const unsigned char TGT_AIN = 1;

/* -- AnalogOutChannel targets -- */
static const unsigned char TGT_AOUT = 1;

/* -- DigitalInChannel targets -- */
static const unsigned char TGT_DIN = 1;

/* -- DigitalOutChannel targets -- */
static const unsigned char TGT_DOUT = 1;

/* -- NullChannel targets -- */

/* -- SensorChannel targets -- */
static const unsigned char TGT_CHIP_OK = 1;
static const unsigned char TGT_X_MG = 2;
static const unsigned char TGT_Y_MG = 3;
static const unsigned char TGT_Z_MG = 4;

/* -- TestingChannel targets -- */
static const unsigned char TGT_T_BOOL = 1;
static const unsigned char TGT_T_UINT8 = 2;
static const unsigned char TGT_T_UINT16 = 3;
static const unsigned char TGT_T_UINT32 = 4;
static const unsigned char TGT_T_INT8 = 5;
static const unsigned char TGT_T_INT16 = 6;
static const unsigned char TGT_T_INT32 = 7;

/* -- UltravoltChannel targets -- */
static const unsigned char TGT_HV_LOCKED = 1;
static const unsigned char TGT_HV_ACTIVE = 2;



/* -- command -- */
static const unsigned char CMD_ASK_CONFIG = 6;
static const unsigned char CMD_BLINK = 4;
static const unsigned char CMD_CHECK_CHIP = 0;
static const unsigned char CMD_CONFIG_CHIP = 1;
static const unsigned char CMD_CONNECT = 1;
static const unsigned char CMD_DISABLE_HV = 2;
static const unsigned char CMD_DISCONNECT = 2;
static const unsigned char CMD_ENABLE_HV = 1;
static const unsigned char CMD_FORCE_READ = 2;
static const unsigned char CMD_PING = 0;
static const unsigned char CMD_RESET = 5;
static const unsigned char CMD_RESET_CLOCK = 3;
static const unsigned char CMD_RUNTEST = 0;
static const unsigned char CMD_UNLOCK_HV = 0;
