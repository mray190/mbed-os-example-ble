/**
 * @file commander.h
 * @author Red Asset Team
 * 
 * @brief  A simple command line processor and callback for a serial based shell 
 * 
 * @version 0.1
 * @date 2018-12-18
 * 
 * mbed Microcontroller Library
 * Copyright (c) 2018 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef _COMMANDER
#define _COMMANDER

/*
    Commander

    A simple command line processor and callback for a serial based shell
*/

#include <string>
#include <map>
#include <vector>
#include "mbed.h"

using namespace std;

/**
 * @brief Callback type for a command
 * 
 */
typedef Callback<void(std::vector<std::string>&)> pFuncCB;

/**
 * @brief Callback type for command ready
 */
typedef Callback<void()> pFuncReady;

/*
    class: cmd

    internal class used to store the commands added to command
*/

/**
 * @brief Internal class used to store the commands added to commander
 */
class Command
{
public:
    std::string strname;
    std::string strdesc;

    pFuncCB pCB;
};


#ifndef STRING_STACK_LIMIT
#define STRING_STACK_LIMIT    512
#endif

/**
 * @brief The main commander class used to provide the RASH (Redteam Abstract SHell) shell over serial
 */
class Commander
{
public:

   /**
    * @brief Construct a new Commander object
    * 
    * @param tx TX line in mbed os
    * @param rx RX line in mbed os 
    * @param baud Bits per seccond of the serial line
    */
    Commander(PinName tx  = USBTX,
              PinName rx  = USBRX,
              int baud    = MBED_CONF_PLATFORM_STDIO_BAUD_RATE);

    /**
     * @brief Destroy the Commander object
     */
   ~Commander();

    /**
     * @brief Add a command and its callback to commander
     * 
     * @param strname The command to add to the console
     * @param strdesc The description of the command and help text.  Is used like "command <param1> <param2>"
     * @param pcallback The handler for this cli command see commander.h for definition
     * 
     * @return int Returns 0 for success
     */
   int add( std::string strname,
            std::string strdesc,
            pFuncCB pcallback);
   
    /**
     * @brief Initialize the serial console listening and processing
     * 
     */
   void init();

   /**
    * @brief Pump the serial console listening and processing
    * 
    * @return true = We got keys
    * @return false = We got nothing
    */
   bool pump();

   /**
    * @brief Break strings into tokens
    * 
    * Crack the strings up and put them into a list<string> with the command as 1st parameter
    * the delimiter used is space
    * 
    * @param strcmd The command to crack into parts
    * @param lsresult The list to put the results in
    */
   void tokenize(std::string& strcmd,
                  std::vector<std::string>& lsresult);

    /**
     * @brief Process the command line
     * 
     * @param strcommand The complete command line to crack open and process
     * 
     * @return int returns 0 for success
     */
    int process(std::string& strcommand);

    /**
     * @brief Prints formatted data to the attached serial
     * 
     * @param format Standard printf style formatting.
     * 
     * @param ... Parameters to using the the format. 
     */
    inline void printf(const char *format, ...)
    {
        char buffer[STRING_STACK_LIMIT];
        va_list args;
        va_start(args, format);
        int length = vsnprintf(buffer, sizeof(buffer), format, args);
        if (length > 0 && (size_t)length < sizeof(buffer)) {

#if MBED_CONF_PLATFORM_STDIO_CONVERT_NEWLINES
        for (int i = 0; i < length; i++) {
            if (buffer[i] == '\n' && _out_prev != '\r') {
                 _serial.putc('\r');
            }
            _serial.putc(buffer[i]);
            _out_prev = buffer[i];
        }
#else
        _serial.puts(buffer);
#endif

        }
        va_end (args);
    }

    /**
     * @brief Dump the help for the current commands
     * 
     * @param vReturn Ignore for this version
     */
    void help(std::vector<std::string>& params);

    /**
     * @brief Print our banner!
     */
    void banner();

    /**
     * @brief Serial input handler
     * 
     * This is the io handler for the serialport. All it does is append
     * our current input buffer with the last thing on the port.
     */
    void input_handler();

    /**
     * @brief Called when a command is ready to process
     * 
     * add a handler to recieve the on ready to process command 
     * for a more asynchronous behavior to the commander interface
     * 
     * @param cb  The callback to get the ready command
     */
    void on_ready(pFuncReady cb);

    /**
     * @brief Remove a handler to recieve the on ready to process command
     * 
     * @param cb The callback to remove
     */
    void del_ready(pFuncReady cb);

protected:

    /**
     * @brief map of command name to info class with callback 
     */
    std::map<std::string, Command> _cmds;

    /**
     * @brief our instance of the serial class
     */
    RawSerial _serial;

    /**
     * @brief Default command line prompt string
     */
    std::string _prompt;


    /**
     * @brief Default start banner prompt string
     */
    std::string _banner;

    /**
     * @brief Our input buffer for serial 
     */
    std::vector<int> _buffer;

    /**
     * @brief The string we are building with our pump
     */
    std::string _strcommand;

    /**
     * @brief Callbacks interested it async cmd processing
     */
    std::vector<pFuncReady> _vready;

private:
#if MBED_CONF_PLATFORM_STDIO_CONVERT_NEWLINES
    char _out_prev;
#endif
};

//our serial interface cli class
extern Commander cmd;

#endif