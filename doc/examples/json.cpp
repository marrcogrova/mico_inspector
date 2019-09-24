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

/// Example usage of cjson library

#include <mico/base/rgbd_tools.h>

int main(){
    cjson::Json json1;
    json["type"] = 1;
    json["class"] = "meme";

    std::cout << "I am accesing to the first json: " << json1["class"] << ", "<< json1["type"] <<std:endl;

    cjson::Json json2;
    json2["all"] = json1;

    std::cout << "I am accesing to the first json again: " << json2["all"]["class"] <<", " <<json2["all"]["type"] <<std:endl;

    std::string json3Str = R"({
                                "type":"dummy", 
                                "data":
                                {
                                    "state":true
                                }
                            })";
    cjson::Json json3;
	js.parse(json3Str);

    std::cout << json3.serialize() << std::endl;
}