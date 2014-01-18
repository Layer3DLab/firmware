#include "Marlin.h"
#include "planner.h"
#include "temperature.h"
#include "ultralcd.h"
#include "ConfigurationStore.h"

extern char json_str[JSONSIZE];
long longnumber = 158;
unsigned unsgnd = 15;
unsigned long unsgndlong = 1587;
uint32_t uint32 = 1587;
uint8_t uint8 = 15;

void _EEPROM_writeData(int &pos, uint8_t* value, uint8_t size)
{
    do
    {
        eeprom_write_byte((unsigned char*)pos, *value);
        pos++;
        value++;
    }while(--size);
}
#define EEPROM_WRITE_VAR(pos, value) _EEPROM_writeData(pos, (uint8_t*)&value, sizeof(value))
void _EEPROM_readData(int &pos, uint8_t* value, uint8_t size)
{
    do
    {
        *value = eeprom_read_byte((unsigned char*)pos);
        pos++;
        value++;
    }while(--size);
}
#define EEPROM_READ_VAR(pos, value) _EEPROM_readData(pos, (uint8_t*)&value, sizeof(value))
//======================================================================================




#define EEPROM_OFFSET 100


// IMPORTANT:  Whenever there are changes made to the variables stored in EEPROM
// in the functions below, also increment the version number. This makes sure that
// the default values are used whenever there is a change to the data, to prevent
// wrong data being written to the variables.
// ALSO:  always make sure the variables in the Store and retrieve sections are in the same order.
#define EEPROM_VERSION "V09"

#ifdef EEPROM_SETTINGS
void Config_StoreSettings() 
{
  char ver[4]= "000";
  int i=EEPROM_OFFSET;
  EEPROM_WRITE_VAR(i,ver); // invalidate data first 
  EEPROM_WRITE_VAR(i,axis_steps_per_unit);  
  EEPROM_WRITE_VAR(i,max_feedrate);  
  EEPROM_WRITE_VAR(i,max_acceleration_units_per_sq_second);
  EEPROM_WRITE_VAR(i,acceleration);
  EEPROM_WRITE_VAR(i,retract_acceleration);
  EEPROM_WRITE_VAR(i,minimumfeedrate);
  EEPROM_WRITE_VAR(i,mintravelfeedrate);
  EEPROM_WRITE_VAR(i,minsegmenttime);
  EEPROM_WRITE_VAR(i,max_xy_jerk);
  EEPROM_WRITE_VAR(i,max_z_jerk);
  EEPROM_WRITE_VAR(i,max_j_jerk);
  EEPROM_WRITE_VAR(i,max_e_jerk);
  EEPROM_WRITE_VAR(i,add_homeing);
  #ifdef DELTA
  EEPROM_WRITE_VAR(i,endstop_adj);
  #endif
  #ifndef ULTIPANEL
  int plaPreheatHotendTemp = PLA_PREHEAT_HOTEND_TEMP, plaPreheatHPBTemp = PLA_PREHEAT_HPB_TEMP, plaPreheatFanSpeed = PLA_PREHEAT_FAN_SPEED;
  int absPreheatHotendTemp = ABS_PREHEAT_HOTEND_TEMP, absPreheatHPBTemp = ABS_PREHEAT_HPB_TEMP, absPreheatFanSpeed = ABS_PREHEAT_FAN_SPEED;
  #endif
  EEPROM_WRITE_VAR(i,plaPreheatHotendTemp);
  EEPROM_WRITE_VAR(i,plaPreheatHPBTemp);
  EEPROM_WRITE_VAR(i,plaPreheatFanSpeed);
  EEPROM_WRITE_VAR(i,absPreheatHotendTemp);
  EEPROM_WRITE_VAR(i,absPreheatHPBTemp);
  EEPROM_WRITE_VAR(i,absPreheatFanSpeed);
  #ifdef PIDTEMP
    EEPROM_WRITE_VAR(i,Kp);
    EEPROM_WRITE_VAR(i,Ki);
    EEPROM_WRITE_VAR(i,Kd);
  #else
		float dummy = 3000.0f;
    EEPROM_WRITE_VAR(i,dummy);
		dummy = 0.0f;
    EEPROM_WRITE_VAR(i,dummy);
    EEPROM_WRITE_VAR(i,dummy);
  #endif
  #ifndef DOGLCD
    int lcd_contrast = 32;
  #endif
  EEPROM_WRITE_VAR(i,lcd_contrast);
  char ver2[4]=EEPROM_VERSION;
  i=EEPROM_OFFSET;
  EEPROM_WRITE_VAR(i,ver2); // validate data
  SERIAL_ECHOPGM("Settings Stored");
}
#endif //EEPROM_SETTINGS


#ifndef DISABLE_M503
void Config_PrintSettings()
{  // Always have this function, even with EEPROM_SETTINGS disabled, the current values will be shown
    SERIAL_ECHO(json_str);
    SERIAL_ECHO_START;
    SERIAL_PROTOCOLPGM("{\"steps/unit\":{");
    SERIAL_ECHOPAIR("\"x\":",axis_steps_per_unit[X_AXIS]);
    SERIAL_ECHOPAIR(",\"y\":",axis_steps_per_unit[Y_AXIS]);
    SERIAL_ECHOPAIR(",\"z\":",axis_steps_per_unit[Z_AXIS]);
    SERIAL_ECHOPAIR(",\"j\":",axis_steps_per_unit[J_AXIS]);
    SERIAL_ECHOPAIR(",\"e\":",axis_steps_per_unit[E_AXIS]);
    SERIAL_PROTOCOLPGM("}}");
    SERIAL_MSG_END;

    SERIAL_ECHO_START;
    SERIAL_PROTOCOLPGM("{\"max feedrate (mm/s)\":{");
    SERIAL_ECHOPAIR("\"x\":",max_feedrate[X_AXIS]);
    SERIAL_ECHOPAIR(",\"y\":",max_feedrate[Y_AXIS]);
    SERIAL_ECHOPAIR(",\"z\":",max_feedrate[Z_AXIS]);
    SERIAL_ECHOPAIR(",\"j\":",max_feedrate[J_AXIS]);
    SERIAL_ECHOPAIR(",\"e\":",max_feedrate[E_AXIS]);
    SERIAL_PROTOCOLPGM("}}");
    SERIAL_MSG_END;

    SERIAL_ECHO_START;
    SERIAL_PROTOCOLPGM("{\"max acceleration (mm/s^2)\":{");
    SERIAL_ECHOPAIR("\"x\":",max_acceleration_units_per_sq_second[X_AXIS]);
    SERIAL_ECHOPAIR(",\"y\":",max_acceleration_units_per_sq_second[Y_AXIS]);
    SERIAL_ECHOPAIR(",\"z\":",max_acceleration_units_per_sq_second[Z_AXIS]);
    SERIAL_ECHOPAIR(",\"j\":",max_acceleration_units_per_sq_second[J_AXIS]);
    SERIAL_ECHOPAIR(",\"e\":",max_acceleration_units_per_sq_second[E_AXIS]);
    SERIAL_PROTOCOLPGM("}}");
    SERIAL_MSG_END;

    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("{\"acceleration\":",acceleration);
    SERIAL_ECHOPAIR(",\"retract_acceleration\":",retract_acceleration);
    SERIAL_PROTOCOLPGM("}");
    SERIAL_MSG_END;

    SERIAL_ECHO_START;
    SERIAL_ECHOPAIR("{\"min feedrate (mm/s)\":",minimumfeedrate);
    SERIAL_ECHOPAIR(",\"min travel feedrate (mm/s)\":",mintravelfeedrate);
    SERIAL_ECHOPAIR(",\"min segment time (ms)\":",minsegmenttime);
    SERIAL_PROTOCOLPGM("}");
    SERIAL_MSG_END;

    SERIAL_ECHO_START;
    SERIAL_PROTOCOLPGM("{\"max jerk\":{");
    SERIAL_ECHOPAIR("\"xy\":",max_xy_jerk);
    SERIAL_ECHOPAIR(",\"z\":",max_z_jerk);
    SERIAL_ECHOPAIR(",\"j\":",max_j_jerk);
    SERIAL_ECHOPAIR(",\"e\":",max_e_jerk);
    SERIAL_PROTOCOLPGM("}}");
    SERIAL_MSG_END;

    SERIAL_ECHO_START;
    SERIAL_PROTOCOLPGM("{\"home offsets (mm)\":{");
    SERIAL_ECHOPAIR("\"x\":",add_homeing[X_AXIS]);
    SERIAL_ECHOPAIR(",\"y\":",add_homeing[Y_AXIS]);
    SERIAL_ECHOPAIR(",\"z\":",add_homeing[Z_AXIS]);
    SERIAL_ECHOPAIR(",\"j\":",add_homeing[J_AXIS]);
    SERIAL_PROTOCOLPGM("}}");
    SERIAL_MSG_END;
#ifdef DELTA
    SERIAL_ECHO_START;
    SERIAL_PROTOCOLPGM("{\"endstop adjustment (mm)\":{");
    SERIAL_ECHOPAIR("\"x\":",endstop_adj[X_AXIS]);
    SERIAL_ECHOPAIR(",\"y\":",endstop_adj[Y_AXIS]);
    SERIAL_ECHOPAIR(",\"z\":",endstop_adj[Z_AXIS]);
    SERIAL_ECHOPAIR(",\"j\":",endstop_adj[J_AXIS]);
    SERIAL_PROTOCOLPGM("}}");
    SERIAL_MSG_END;
#endif
#ifdef PIDTEMP
    SERIAL_ECHO_START;
    SERIAL_PROTOCOLPGM("{\"nozzle pid\":{");
    SERIAL_ECHOPAIR("\"p\":",Kp);
    SERIAL_ECHOPAIR(",\"i\":",unscalePID_i(Ki));
    SERIAL_ECHOPAIR(",\"d\":",unscalePID_d(Kd));
    SERIAL_PROTOCOLPGM("}}");
    SERIAL_MSG_END;
#endif
} 
#endif


#ifdef EEPROM_SETTINGS
void Config_RetrieveSettings()
{
    int i=EEPROM_OFFSET;
    char stored_ver[4];
    char ver[4]=EEPROM_VERSION;
    EEPROM_READ_VAR(i,stored_ver); //read stored version
    if (strncmp(ver,stored_ver,3) == 0)
    {
        // version number match
        EEPROM_READ_VAR(i,axis_steps_per_unit);  
        EEPROM_READ_VAR(i,max_feedrate);  
        EEPROM_READ_VAR(i,max_acceleration_units_per_sq_second);
        
        // steps per sq second need to be updated to agree with the units per sq second (as they are what is used in the planner)
		reset_acceleration_rates();
        
        EEPROM_READ_VAR(i,acceleration);
        EEPROM_READ_VAR(i,retract_acceleration);
        EEPROM_READ_VAR(i,minimumfeedrate);
        EEPROM_READ_VAR(i,mintravelfeedrate);
        EEPROM_READ_VAR(i,minsegmenttime);
        EEPROM_READ_VAR(i,max_xy_jerk);
        EEPROM_READ_VAR(i,max_z_jerk);
        EEPROM_READ_VAR(i,max_j_jerk);
        EEPROM_READ_VAR(i,max_e_jerk);
        EEPROM_READ_VAR(i,add_homeing);
        #ifdef DELTA
        EEPROM_READ_VAR(i,endstop_adj);
        #endif
        #ifndef ULTIPANEL
        int plaPreheatHotendTemp, plaPreheatHPBTemp, plaPreheatFanSpeed;
        int absPreheatHotendTemp, absPreheatHPBTemp, absPreheatFanSpeed;
        #endif
        EEPROM_READ_VAR(i,plaPreheatHotendTemp);
        EEPROM_READ_VAR(i,plaPreheatHPBTemp);
        EEPROM_READ_VAR(i,plaPreheatFanSpeed);
        EEPROM_READ_VAR(i,absPreheatHotendTemp);
        EEPROM_READ_VAR(i,absPreheatHPBTemp);
        EEPROM_READ_VAR(i,absPreheatFanSpeed);
        #ifndef PIDTEMP
        float Kp,Ki,Kd;
        #endif
        // do not need to scale PID values as the values in EEPROM are already scaled		
        EEPROM_READ_VAR(i,Kp);
        EEPROM_READ_VAR(i,Ki);
        EEPROM_READ_VAR(i,Kd);
        #ifndef DOGLCD
        int lcd_contrast;
        #endif
        EEPROM_READ_VAR(i,lcd_contrast);

		// Call updatePID (similar to when we have processed M301)
		updatePID();
        SERIAL_ECHOPGM("\"Stored settings retrieved\"");
    }
    else
    {
        Config_ResetDefault();
    }
    #ifdef EEPROM_CHITCHAT
      Config_PrintSettings();
    #endif
}
#endif

void Config_ResetDefault()
{
    float tmp1[]=DEFAULT_AXIS_STEPS_PER_UNIT;
    float tmp2[]=DEFAULT_MAX_FEEDRATE;
    long tmp3[]=DEFAULT_MAX_ACCELERATION;
    for (short i=0;i<NUM_AXIS;i++) 
    {
        axis_steps_per_unit[i]=tmp1[i];  
        max_feedrate[i]=tmp2[i];  
        max_acceleration_units_per_sq_second[i]=tmp3[i];
    }
    
    // steps per sq second need to be updated to agree with the units per sq second
    reset_acceleration_rates();
    
    acceleration=DEFAULT_ACCELERATION;
    retract_acceleration=DEFAULT_RETRACT_ACCELERATION;
    minimumfeedrate=DEFAULT_MINIMUMFEEDRATE;
    minsegmenttime=DEFAULT_MINSEGMENTTIME;       
    mintravelfeedrate=DEFAULT_MINTRAVELFEEDRATE;
    max_xy_jerk=DEFAULT_XYJERK;
    max_z_jerk=DEFAULT_ZJERK;
    max_j_jerk=DEFAULT_JJERK;
    max_e_jerk=DEFAULT_EJERK;
    add_homeing[0] = add_homeing[1] = add_homeing[2] = add_homeing[3] = 0;
#ifdef DELTA
    endstop_adj[0] = endstop_adj[1] = endstop_adj[2] = endstop_adj[3] = 0;
#endif
#ifdef ULTIPANEL
    plaPreheatHotendTemp = PLA_PREHEAT_HOTEND_TEMP;
    plaPreheatHPBTemp = PLA_PREHEAT_HPB_TEMP;
    plaPreheatFanSpeed = PLA_PREHEAT_FAN_SPEED;
    absPreheatHotendTemp = ABS_PREHEAT_HOTEND_TEMP;
    absPreheatHPBTemp = ABS_PREHEAT_HPB_TEMP;
    absPreheatFanSpeed = ABS_PREHEAT_FAN_SPEED;
#endif
#ifdef DOGLCD
    lcd_contrast = DEFAULT_LCD_CONTRAST;
#endif
#ifdef PIDTEMP
    Kp = DEFAULT_Kp;
    Ki = scalePID_i(DEFAULT_Ki);
    Kd = scalePID_d(DEFAULT_Kd);
    
    // call updatePID (similar to when we have processed M301)
    updatePID();
    
#ifdef PID_ADD_EXTRUSION_RATE
    Kc = DEFAULT_Kc;
#endif//PID_ADD_EXTRUSION_RATE
#endif//PIDTEMP
;
SERIAL_ECHOPGM("\"Hardcoded default settings loaded\"");

}
