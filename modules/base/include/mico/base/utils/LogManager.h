//---------------------------------------------------------------------------------------------------------------------
//  mico
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


#ifndef MICO_BASE_UTILS_LOGMANAGER_H_
#define MICO_BASE_UTILS_LOGMANAGER_H_

#include <string>
#include <fstream>
#include <mutex>
#include <chrono>
#include <iostream>

#include <unordered_map>
namespace mico {
	/// Thread safe class used as logging system. 
	class LogManager {
	public:	//	Static interface.
		/// Initialize the logging system. 
		/// \param _appName: Base name used for the log file.
		/// \param _useCout: Write to cout too.
		static void init(const std::string _appName);

		/// Close the logging system. It makes sure that the log is closed properly.
		static void close();

		/// Get current instance of the logging system.
		static LogManager* get();

	public:	// Public interface.
		/// Color handles for messages
		typedef std::string ColorHandle;
		static const ColorHandle cTextRed;
		static const ColorHandle cTextYellow;
		static const ColorHandle cTextBlue;
		static const ColorHandle cTextGreen;
		static const ColorHandle cTextReset;
	
	
		/// Write message to the log system with a custom tag
		/// \param _tag
		/// \param _msg
		/// \param _useCout
		/// \param _color
		void message(const std::string &_tag, const std::string &_msg, bool _useCout = false, ColorHandle _color = "");
		
		/// Write to the log system with status tag.
		/// \param _msg
		/// \param _useCout
		void status(const std::string &_msg, bool _useCout = false);

		/// Write to the log system with warning tag.
		/// \param _msg
		/// \param _useCout
		void warning(const std::string &_msg, bool _useCout = false);

		/// Write to the log system with error tag.
		/// \param _msg
		/// \param _useCout
		void error(const std::string &_msg, bool _useCout = false);

		/// Save current time with a label for later use
		/// \param _tag
		void saveTimeMark(std::string _tag);

		/// compute time in seconds between two time marks
		/// \param _tag1
		/// \param _tag2
		double measureTimeBetweenMarks(std::string _tag1, std::string _tag2);

	private:	// Private interface.
		LogManager(const std::string _appName);
		~LogManager();

		static LogManager *mSingleton;

		bool mUseCout = false;

		std::unordered_map<std::string, std::chrono::high_resolution_clock::time_point> mTimeMap;
		std::chrono::high_resolution_clock::time_point  mInitTime;
		std::ofstream mLogFile;
		std::mutex mSecureGuard;

	};

	/// Enum types for defining debug level of LoggableInterface
	enum class DebugLevels { Debug, Warning, Error, Null };

	/// Enum types for defining output mode of LoggableInterface
	enum class OutInterfaces { Cout, LogManager, Null };

	/// Interface class to add log capabilities to class.
	/// \param DebugLevel_: Define displayed messages. Debug display all the messages; Warning warnings and errors; Error just the errors 
	/// ; and Null nothing.
	/// \param OutInterfaces: Define how messages are managed. Cout mode uses standart std::cout and LogManager uses rgbd_tools LogManager util 
	/// which writes in the log file too.
	template<DebugLevels DebugLevel_, OutInterfaces OutInterface_>
	class LoggableInterface{	// 777 by now using conditional templates. Look for a more templatized way
	public:
		void status		(const std::string &_tag, const std::string &_msg){
			if(OutInterface_ ==  OutInterfaces::Cout && DebugLevel_ == DebugLevels::Debug)
				std::cout << LogManager::cTextBlue << "["<< _tag << "]\t" << _msg << LogManager::cTextReset << std::endl;  
			else if(OutInterface_ ==  OutInterfaces::LogManager && DebugLevel_ == DebugLevels::Debug)
				LogManager::get()->message(_tag, _msg, 	DebugLevel_ == DebugLevels::Debug, LogManager::cTextBlue);
				
		}
		
		void warning	(const std::string &_tag, const std::string &_msg){
			if(OutInterface_ ==  OutInterfaces::Cout && (	DebugLevel_ == DebugLevels::Debug || 
															DebugLevel_ == DebugLevels::Warning))
				std::cout << LogManager::cTextYellow << "["<< _tag << "]\t" << _msg << LogManager::cTextReset << std::endl;
			else if(OutInterface_ ==  OutInterfaces::LogManager && DebugLevel_ == DebugLevels::Debug)
				LogManager::get()->message(_tag, _msg, 	DebugLevel_ == DebugLevels::Debug || 
														DebugLevel_ == DebugLevels::Warning,
														LogManager::cTextYellow);
		}
		
		void error		(const std::string &_tag, const std::string &_msg){
			if(OutInterface_ ==  OutInterfaces::Cout && (	DebugLevel_ == DebugLevels::Debug || 
															DebugLevel_ == DebugLevels::Warning || 
															DebugLevel_ == DebugLevels::Error))
				std::cout << LogManager::cTextRed << "["<< _tag << "]\t" << _msg << LogManager::cTextReset << std::endl;
			else if(OutInterface_ ==  OutInterfaces::LogManager && DebugLevel_ == DebugLevels::Debug)
				LogManager::get()->message(_tag, _msg, 	DebugLevel_ == DebugLevels::Debug || 
														DebugLevel_ == DebugLevels::Warning || 
														DebugLevel_ == DebugLevels::Error,
														LogManager::cTextRed);
		}

		void message(const std::string &_tag, const std::string &_msg, LogManager::ColorHandle _color = ""){
			if(OutInterface_ ==  OutInterfaces::Cout)
				std::cout << _color << "["<< _tag << "]\t" << _msg << LogManager::cTextReset << std::endl;
			else if(OutInterface_ ==  OutInterfaces::LogManager && DebugLevel_ == DebugLevels::Debug)
				LogManager::get()->message(_tag, _msg, 	true, _color);
		}
	};

}

#endif
