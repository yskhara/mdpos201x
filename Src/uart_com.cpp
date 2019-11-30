/*
 * uart::com.cpp
 *
 *  Created on: Dec 27, 2018
 *      Author: yusaku
 */

#include "uart_com.hpp"
#include "SerialClass.hpp"
#include "motor_ctrl.hpp"
#include <cstdio>
#include <cstring>

extern SerialClass serial;
extern MotorCtrl control;

static constexpr unsigned int cmd_buf_size = 128;
unsigned int cmd_buf_ptr = 0;
char cmd_buf[cmd_buf_size];

char tx_buf[128];

void uart::process(void)
{
    int ch = serial.read();
    if (ch == -1)
    {
        return;
    }

    char c = static_cast<char>(ch);

    if ('a' <= c && c <= 'z')
    {
        c += 'A' - 'a';
    }
    else if ('A' <= c && c <= 'Z')
    {

    }
    else if ('0' <= c && c <= '9')
    {

    }
    else if (c == '.' || c == '-' || c == ' ')
    {

    }
    else if (c == '\b' || c == 127)
    {
        // backspace
        if (cmd_buf_ptr > 0)
        {
            cmd_buf_ptr--;
            //uint8_t _str[] = {'\b', ' ', '\e', '[', 'D'};
            serial.write((const uint8_t *) "\b \e[D", 5);
        }
        else
        {
            //uint8_t _str[] = {'\a'};
            serial.write((const uint8_t *) "\a", 1);
        }

        return;
    }
    else if (c == '\r' || c == '\n')
    {

    }
    else
    {
        return;
    }

    cmd_buf[cmd_buf_ptr] = c;
    cmd_buf_ptr++;

    serial.write((const uint8_t *) &c, 1);

    if (cmd_buf_ptr >= cmd_buf_size)
    {
        const char * msg = "rx buffer overrun. maximum frame length = 127.\r\n";
        serial.write((const uint8_t *) msg, strlen(msg));
    }

    // if the current character is not a line-break, just return and wait for next character.
    if (c != '\r' && c != '\n')
    {
        return;
    }

    // otherwise, proceed to process the line.
    serial.write((const uint8_t *) "\r\n", 2);

    char cmd[8];
    //strncpy(cmd, cmd_buf, 4);
    //cmd[4] = '\0';

    //char * payload[cmd_buf_size];
    //strcpy(cmd, (cmd_buf+4));
    double payload = 0.0;
    int payload_int;
    if (sscanf(cmd_buf, "%s 0X%x", cmd, &payload_int) != 2)
    {
        if (sscanf(cmd_buf, "%s %lf", cmd, &payload) < 1)
        {
            //const char * msg = "--> Invalid input.\r\n";
            //serial.write((const uint8_t *) msg, strlen(msg));

            uart::prompt();
            return;
        }
    }
    else
    {
        payload = static_cast<double>(payload_int);
    }

    //char tx_buf[128];
    //const char * inv_msg = "Invalid value for";

    if (strcmp(cmd, "SBID") == 0)
    {
        // set CAN standard identifier of Base-ID
        int base_id = static_cast<uint16_t>(payload);

        if (base_id < 0 || 0x7ff < (base_id + 3))
        {
            const char * msg = "--> Invalid value for BID (not in range).\r\n";
            serial.write((const uint8_t *) msg, strlen(msg));
            uart::dump_can_id(false);
        }
        else
        {
            confStruct.can_id_cmd = base_id;
            confStruct.can_id_vel = base_id + 1;
            confStruct.can_id_stat = base_id + 3;
            // well, base+2 is currently reserved for "future use." what a waste, eh?
        }

        uart::dump_can_id(true);
    }
    else if (strcmp(cmd, "GBID") == 0)
    {
        // get CAN standard identifier of Base-ID
        uart::dump_can_id(false);
    }
    /*
     else if (strcmp(cmd, "SVID") == 0)
     {
     // set CAN standard identifier for velocity command
     confStruct.can_id_vel = static_cast<uint16_t>(payload);

     int ret = sprintf(tx_buf, "Set CAN ID (vel): 0x%x\r\n", confStruct.can_id_vel);
     serial.write((const uint8_t *) tx_buf, ret);
     }
     else if (strcmp(cmd, "GVID") == 0)
     {
     // get CAN standard identifier for velocity command
     int ret = sprintf(tx_buf, "Current CAN ID (vel): 0x%x\r\n", confStruct.can_id_vel);
     serial.write((const uint8_t *) tx_buf, ret);
     }
     else if (strcmp(cmd, "SSID") == 0)
     {
     // set CAN standard identifier for status broadcast
     confStruct.can_id_stat = static_cast<uint16_t>(payload);

     int ret = sprintf(tx_buf, "Set CAN ID (stat): 0x%x\r\n", confStruct.can_id_stat);
     serial.write((const uint8_t *) tx_buf, ret);
     }
     else if (strcmp(cmd, "GSID") == 0)
     {
     // set CAN standard identifier for status broadcast
     int ret = sprintf(tx_buf, "Current CAN ID (stat): 0x%x\r\n", confStruct.can_id_stat);
     serial.write((const uint8_t *) tx_buf, ret);
     }
     */
    else if (strcmp(cmd, "SKPR") == 0)
    {
        // set proportional gain Kp
        int ret = control.SetKp(payload);

        if (ret != 0)
        {
            uart::invalid_value("Kp", payload);
        }
        else
        {
            uart::valid_value_set("Kp", "N.m/(rad/s)", payload);
        }
    }
    else if (strcmp(cmd, "GKPR") == 0)
    {
        // get proportional gain Kp
        uart::dump_value("Kp", "N.m/(rad/s)", control.GetKp());
    }
    else if (strcmp(cmd, "SKIT") == 0)
    {
        // set integral gain Ki
        int ret = control.SetKi(payload);

        if (ret != 0)
        {
            uart::invalid_value("Ki", payload);
        }
        else
        {
            uart::valid_value_set("Ki", "N.m/rad", payload);
        }
    }
    else if (strcmp(cmd, "GKIT") == 0)
    {
        // get integral gain Ki
        uart::dump_value("Ki", "N.m/rad", control.GetKi());
    }
    else if (strcmp(cmd, "SKEM") == 0)
    {
        // set emf constant ke
        int ret = control.SetKe(payload);

        if (ret != 0)
        {
            uart::invalid_value("Ke", payload);
        }
        else
        {
            uart::valid_value_set("Ke", "V/(rad/s)", payload);
        }
    }
    else if (strcmp(cmd, "GKEM") == 0)
    {
        // get em constant ke
        uart::dump_value("Ke", "V/(rad/s)", control.GetKe());
    }
    else if (strcmp(cmd, "SKGT") == 0)
    {
        // set winding resistance / torque constant = Kg
        int ret = control.SetKg(payload);

        if (ret != 0)
        {
            uart::invalid_value("Kg", payload);
        }
        else
        {
            uart::valid_value_set("Kg", "V/(N.m)", payload);
        }
    }
    else if (strcmp(cmd, "GKGT") == 0)
    {
        // get winding resistance / torque constant = Kg
        uart::dump_value("Kg", "V/(N.m)", control.GetKg());
    }
    else if (strcmp(cmd, "SPPR") == 0)
    {
        // set pulse per revolution
        int ret = control.SetPPR(payload);

        if (ret != 0)
        {
            uart::invalid_value("PPR", payload);
        }
        else
        {
            uart::valid_value_set("PPR", "pulse/rev.", payload);
        }
    }
    else if (strcmp(cmd, "GPPR") == 0)
    {
        // get pulse per revolution
        uart::dump_value("PPR", "pulse/rev.", control.GetPPR());
    }
    else if (strcmp(cmd, "SKRF") == 0)
    {
        // set reference coefficient
        int ret = control.SetKr(payload);

        if (ret != 0)
        {
            uart::invalid_value("Kr", payload);
        }
        else
        {
            uart::valid_value_set("Kr", "(rad/s)/cmd", payload);
        }
    }
    else if (strcmp(cmd, "GKRF") == 0)
    {
        // get reference coefficient
        uart::dump_value("Kr", "(rad/s)/cmd", control.GetKr());
    }
    else if (strcmp(cmd, "SMVL") == 0)
    {
        // set maximum velocity
        int ret = control.SetMaximumVelocity(payload);

        if (ret != 0)
        {
            uart::invalid_value("Omega_max", payload);
        }
        else
        {
            uart::valid_value_set("Omega_max", "rad/s", payload);
        }
    }
    else if (strcmp(cmd, "GMVL") == 0)
    {
        // get maximum velocity
        uart::dump_value("Omega_max", "rad/s", control.GetMaximumVelocity());
    }
    else if (strcmp(cmd, "SHVL") == 0)
    {
        // set homing velocity
        int ret = control.SetHomingVelocity(payload);

        if (ret != 0)
        {
            uart::invalid_value("Omega_homing", payload);
        }
        else
        {
            uart::valid_value_set("Omega_homing", "rad/s", payload);
        }
    }
    else if (strcmp(cmd, "GHVL") == 0)
    {
        // get homing velocity
        uart::dump_value("Omega_homing", "rad/s", control.GetHomingVelocity());
    }
    else if (strcmp(cmd, "SMTQ") == 0)
    {
        // set maximum torque
        int ret = control.SetMaximumTorque(payload);

        if (ret != 0)
        {
            uart::invalid_value("Tmax", payload);
        }
        else
        {
            uart::valid_value_set("Tmax", "N.m", payload);
        }
    }
    else if (strcmp(cmd, "GMTQ") == 0)
    {
        // get maximum torque
        uart::dump_value("Tmax", "N.m", control.GetMaximumTorque());
    }
    else if (strcmp(cmd, "SVSP") == 0)
    {
        // set supply voltage
        int ret = control.SetSupplyVoltage(payload);
        const char * name = "Vsup";

        if (ret != 0)
        {
            uart::invalid_value(name, payload);
        }
        else
        {
            uart::valid_value_set(name, "V", payload);
        }
    }
    else if (strcmp(cmd, "GVSP") == 0)
    {
        // get maximum velocity
        uart::dump_value("Vsup", "V", control.GetSupplyVoltage());
    }
    else if (strcmp(cmd, "SKVP") == 0)
    {
        // set kv
        int ret = control.SetKv(payload);
        const char * name = "Kv";

        if (ret != 0)
        {
            uart::invalid_value(name, payload);
        }
        else
        {
            uart::valid_value_set(name, "(rad/s)/rad", payload);
        }
    }
    else if (strcmp(cmd, "GKVP") == 0)
    {
        // get kv
        uart::dump_value("Kv", "(rad/s)/rad", control.GetKv());
    }/*
    else if (strcmp(cmd, "CBNK") == 0)
    {
        // change bank
        uint8_t bank_num = payload;

        if (bank_num < 0 || NUM_OF_BANKS <= bank_num)
        {
            return;
        }
        conf_change_bank(bank_num);
        //readConf();
        control.ReadConfig();

        int ret = sprintf(tx_buf, "--> Changed to bank %d\r\n", bank_num);
        serial.write((const uint8_t *) tx_buf, ret);
    }*/
    else if (strcmp(cmd, "WCFG") == 0)
    {
        control.WriteConfig();
        writeConf();

        const char * msg = "written to flash\r\n";
        serial.write((const uint8_t *) msg, strlen(msg));
    }
    else if (strcmp(cmd, "RCFG") == 0)
    {
        readConf();
        control.ReadConfig();

        const char * msg = "read from flash\r\n";
        serial.write((const uint8_t *) msg, strlen(msg));
    }
    else if (strcmp(cmd, "SENV") == 0)
    {
    	const char * msg;
        if(payload==1.0){
        	control.Recover();
        	msg = "enable\r\n";
        }
        else{
        	control.Shutdown();
        	msg = "disable\r\n";
        }
        serial.write((const uint8_t *) msg, strlen(msg));
    }
    else if (strcmp(cmd, "SVTG") == 0)
    {
        control.SetTarget(payload);
        const char * msg = "set velocity target\r\n";
        serial.write((const uint8_t *) msg, strlen(msg));
    }
    uart::prompt();
}

void uart::prompt(void)
{
    for (unsigned int i = 0; i < cmd_buf_size; i++)
    {
        cmd_buf[i] = 0x00;
    }
    cmd_buf_ptr = 0;
    serial.write((const uint8_t *) "> ", 2);
}

void uart::dump_value(const char * name, const char * unit, double value)
{
    //char tx_buf[128];
    int ret = sprintf(tx_buf, "--> Current %s: %lf [%s]\r\n", name, value, unit);
    if (ret < 0)
    {
        const char * msg = "err@sprintf@dump_val";
        serial.write((const uint8_t *) msg, strlen(msg));
    }
    else
    {
        serial.write((const uint8_t *) tx_buf, ret);
    }
}

void uart::invalid_value(const char * name, double value)
{
    int ret = sprintf(tx_buf, "--> Invalid value for %s: %lf\r\n", name, value);
    serial.write((const uint8_t *) tx_buf, ret);
}

void uart::valid_value_set(const char * name, const char * unit, double value)
{
    int ret = sprintf(tx_buf, "--> Set %s: %lf [%s]\r\n", name, value, unit);
    serial.write((const uint8_t *) tx_buf, ret);
}

void uart::dump_can_id(bool set)
{
    const char * fmt;
    if (set)
    {
        fmt = "--> Set CAN ID (cmd): 0x%x\r\n--> Set CAN ID (vel): 0x%x\r\n--> Set CAN ID (stat): 0x%x\r\n";
    }
    else
    {
        fmt = "--> Current CAN ID (cmd): 0x%x\r\n--> Current CAN ID (vel): 0x%x\r\n--> Current CAN ID (stat): 0x%x\r\n";
    }

    int ret = sprintf(tx_buf, fmt, getConf()->can_id_cmd, getConf()->can_id_vel, getConf()->can_id_stat);
    serial.write((const uint8_t *) tx_buf, ret);
}
