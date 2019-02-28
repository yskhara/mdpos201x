/*
 * uart_com.hpp
 *
 *  Created on: Dec 27, 2018
 *      Author: yusaku
 */

#ifndef UART_COM_HPP_
#define UART_COM_HPP_



void uart_process(void);
void uart_prompt(void);
void uart_dump_value(const char * name, const char * unit, double value);
void uart_invalid_value(const char * name, double value);
void uart_valid_value_set(const char * name, const char * unit, double value);


#endif /* UART_COM_HPP_ */
