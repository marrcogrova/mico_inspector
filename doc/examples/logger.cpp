//---------------------------------------------------------------------------------------------------------------------
//  RGBD_TOOLS
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
//---------------------------------------------------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
//  and associated documentation files (the "Software"), to deal in the Software without restriction,
//  including without limitation the rights to use, copy, modify, merge, publish, distribute,
//  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all copies or substantial
//  portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
//  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
//  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//---------------------------------------------------------------------------------------------------------------------


/// Example of logger system
/// @code
///     #include <mico/base/rgbd_tools.h>
///     #include <thread>
///     #include <chrono>
///     
///     int main(void){
///         LogManager::init("sample_logging_"+std::to_string(time(NULL))+".txt");
///     
///     
///         LogManager::get()->status("This line is stored in the log and shown in cout", true);
///     
///         LogManager::get()->status("Next line is goint to be logged but will not appear here", true);
///         LogManager::get()->status("I am a secret line", false);
///     
///         LogManager::get()->status("Let's use time marks", true);
///         LogManager::get()->saveTimeMark("init_nap");
///         std::this_thread::sleep_for(std::chrono::seconds(2));
///         LogManager::get()->saveTimeMark("end_nap"); 
///     
///         float napTime = LogManager::get()->measureTimeBetweenMarks("end_nap",    "init_nap");
///     
///         LogManager::get()->status("Nap time: " +std::to_string(napTime),     true);
///     }
/// @endcode