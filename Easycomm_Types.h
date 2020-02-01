#ifndef EASYCOMM_TYPES_H
#define EASYCOMM_TYPES_H

// Numerical representation of EasyComm III commands.
typedef enum Easycomm_Cmd_Defs
{
    // EasyComm I.
    EASYCOMM_AZ =  0, 
    EASYCOMM_EL =  1,
    EASYCOMM_UP =  2,
    EASYCOMM_DN =  3,
    // EasyComm II.
    EASYCOMM_DM =  4,
    EASYCOMM_UM =  5,
    EASYCOMM_DR =  6,
    EASYCOMM_UR =  7,
    EASYCOMM_ML =  8,
    EASYCOMM_MR =  9,
    EASYCOMM_MU = 10,
    EASYCOMM_MD = 11,
    EASYCOMM_SA = 12,
    EASYCOMM_SE = 13,
    EASYCOMM_AO = 14,
    EASYCOMM_LO = 15,
    EASYCOMM_OP = 16,
    EASYCOMM_IP = 17,
    EASYCOMM_AN = 18,
    EASYCOMM_ST = 19,
    EASYCOMM_VE = 20,
    // EasyComm III.
    EASYCOMM_VL = 21, 
    EASYCOMM_VR = 22,
    EASYCOMM_VU = 23,
    EASYCOMM_VD = 24,
    EASYCOMM_CR = 25,
    EASYCOMM_CW = 26,
    EASYCOMM_GS = 27,
    EASYCOMM_GE = 28,
    EASYCOMM_PARK = 29,
    EASYCOMM_RESET = 30,
    EASYCOMM_NUM_CMDS

} Easycomm_Cmd_Defs;

// List of EasyComm III commands.
// Indices match the definitions.
const char* Easycomm_Cmds[] =
{
    // EasyComm I   (4 commands).
    "AZ", 
    "EL",
    "UP",
    "DN",
    // EasyComm II  (17 commands).
    "DM",
    "UM",
    "DR",
    "UR",
    "ML",
    "MR",
    "MU",
    "MD",
    "SA",
    "SE",
    "AO",
    "LO",
    "OP",
    "IP",
    "AN",
    "ST",
    "VE",
    // EasyComm III (8 commands).
    "VL",
    "VR",
    "VU",
    "VD",
    "CR",
    "CW",
    "GS",
    "GE",
    // Custom (4 commands).
    "PARK",
    "RESET",
    "REBOOT",
    "BREAKIN"
};

// Easycomm states.
typedef enum Easycomm_Status
{
    EASYCOMM_STATUS_IDLE     = 0x01,
    EASYCOMM_STATUS_MOVING   = 0x02,
    EASYCOMM_STATUS_POINTING = 0x04,
    EASYCOMM_STATUS_ERROR    = 0x08

} Easycomm_Status;

// Easycomm errors.
typedef enum Easycomm_Error
{
    EASYCOMM_ERROR_SENSOR = 0x01,
    EASYCOMM_ERROR_JAM    = 0x02,
    EASYCOMM_ERROR_HOMING = 0x04

} Easycomm_Error;

// Easycomm configuration registers.
typedef enum Easycomm_Config_Reg
{
    EASYCOMM_REG_MAXSPEED  = 0x00,
    EASYCOMM_REG_OVERSHOOT = 0x0a,
    EASYCOMM_REG_JAMMING   = 0x0b,
    EASYCOMM_REG_ENDPOINTS = 0x0c,
    EASYCOMM_REG_UNSTICK   = 0x0d

} Easycomm_Config_Reg;

#endif // EASYCOMM_TYPES_H
