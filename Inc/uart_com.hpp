/*
 * uart_com.hpp
 *
 *  Created on: Dec 27, 2018
 *      Author: yusaku
 */

#ifndef UART_COM_HPP_
#define UART_COM_HPP_

namespace uart
{
    void process(void);
    void prompt(void);
    void dump_value(const char * name, const char * unit, double value);
    void invalid_value(const char * name, double value);
    void valid_value_set(const char * name, const char * unit, double value);
    void dump_can_id(bool set);
}

#endif /* UART_COM_HPP_ */
