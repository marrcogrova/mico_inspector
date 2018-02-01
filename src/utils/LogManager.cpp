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


#include <rgbd_tools/utils/LogManager.h>
#include <iostream>

using namespace std;

LogManager *LogManager::mSingleton = nullptr;

//---------------------------------------------------------------------------------------------------------------------
void LogManager::init(const string _appName) {
	if (!mSingleton)
        mSingleton = new LogManager(_appName);
	else
		mSingleton->warning("Someone tried to reinitialize the Log Manager");
}

//---------------------------------------------------------------------------------------------------------------------
void LogManager::close(){
	delete mSingleton;
}

//---------------------------------------------------------------------------------------------------------------------
LogManager * LogManager::get(){
	return mSingleton;
}

//---------------------------------------------------------------------------------------------------------------------
void LogManager::message(const std::string & _msg, const std::string & _tag, bool _useCout) {
	double timeSpan = std::chrono::duration<double>(chrono::high_resolution_clock::now() - mInitTime).count();
	std::string logLine = to_string(timeSpan) + "\t [" + _tag + "] " + _msg + "\n";
	mSecureGuard.lock();
	mLogFile << logLine;
	mLogFile.flush();
	mSecureGuard.unlock();
    if(_useCout){
		cout << logLine;
        cout.flush();
	}
}

//---------------------------------------------------------------------------------------------------------------------
void LogManager::status(const std::string & _msg, bool _useCout){
    message(_msg, "STATUS", _useCout);
}

//---------------------------------------------------------------------------------------------------------------------
void LogManager::warning(const std::string & _msg, bool _useCout){
    message(_msg, "WARNING", _useCout);
}

//---------------------------------------------------------------------------------------------------------------------
void LogManager::error(const std::string & _msg, bool _useCout){
    message(_msg, "ERROR", _useCout);
}

//---------------------------------------------------------------------------------------------------------------------
void LogManager::saveTimeMark(string _tag) {
    mTimeMap[_tag] = std::chrono::high_resolution_clock::now();
}

//---------------------------------------------------------------------------------------------------------------------
double LogManager::measureTimeBetweenMarks(string _tag1, string _tag2) {
    return std::chrono::duration<double>(mTimeMap[_tag1] - mTimeMap[_tag2]).count();
}

//---------------------------------------------------------------------------------------------------------------------
LogManager::LogManager(const std::string _appName) {
	mLogFile.open(_appName + to_string(time(NULL))+".txt");
	mInitTime = chrono::high_resolution_clock::now();
	status("Initialized log");
}

//---------------------------------------------------------------------------------------------------------------------
LogManager::~LogManager() {
	mLogFile.close();
}
