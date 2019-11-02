//---------------------------------------------------------------------------------------------------------------------
//  MICO
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2019 - Pablo Ramon Soria (a.k.a. Bardo91) 
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

#include <mico/flow/Policy.h>
#include <mico/flow/OutPipe.h>

#include <gtest/gtest.h>


using namespace mico;

TEST(stream_creation, stream_creation)  {
    Policy pol1({"i1"});
    Policy pol2({"i1","i2"});
    Policy pol3({"i1","i2","i3"});
    Policy pol4({""});
    ASSERT_DEATH(Policy pol5({}), ".*");
}

TEST(pipe_creation, pipe_creation)  {
    
    OutPipe op1("int");
    OutPipe op2("float");
    Policy pol({"int"});

    ASSERT_TRUE(op1.registerPolicy(&pol));
    ASSERT_FALSE(op2.registerPolicy(&pol));

}

TEST(registration, registration)  {
    // Invalid creation
    ASSERT_DEATH(OutPipe op(""),".*");

    // Valid creation
    OutPipe op("o1");
    ASSERT_STREQ("o1", op.tag().c_str());
    ASSERT_EQ(0, op.registrations());

    Policy pol({"o1"});
    op.registerPolicy(&pol);
    ASSERT_EQ(1, op.registrations());

}

TEST(transmission_int_1, transmission_int)  {
    OutPipe op("int");

    Policy pol({"int"});

    int res = 0;
    pol.setCallback({"int"}, [&](std::unordered_map<std::string,std::any> _data){
        res = std::any_cast<int>(_data["int"]);
    });

    op.registerPolicy(&pol);

    // Good type flush
    op.flush(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    ASSERT_EQ(res, 1);

    // Bad type flush
    ASSERT_DEATH(
        op.flush(5.312f);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    , ".*");
}


TEST(disconnect, disconnect)  {
    OutPipe op("int");
    Policy pol({"int"});

    int res = 0;
    pol.setCallback({"int"}, [&](std::unordered_map<std::string,std::any> _data){
        res = std::any_cast<int>(_data["int"]);
    });
    
    op.registerPolicy(&pol);
    
    op.flush(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    ASSERT_EQ(res, 1);

    pol.disconnect("int");

    op.flush(2);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    ASSERT_EQ(res, 1);
    ASSERT_EQ(0, op.registrations());


}

TEST(transmission_int_2, transmission_int)  {
    OutPipe op("int");

    Policy pol1({"int"});
    Policy pol2({"int"});

    int counterCall1 = 0;
    std::mutex guardCall1;
    pol1.setCallback({"int"}, [&](std::unordered_map<std::string,std::any> _data){
        guardCall1.lock();
        counterCall1++;
        guardCall1.unlock();    
    });
    
    int counterCall2 = 0;
    std::mutex guardCall2;
    pol2.setCallback({"int"}, [&](std::unordered_map<std::string,std::any> _data){
        guardCall2.lock();
        counterCall2++;
        guardCall2.unlock();
    });

    op.registerPolicy(&pol1);
    op.registerPolicy(&pol2);

    // Good type flush
    op.flush(1);
    op.flush(1);
    op.flush(1);
    op.flush(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    ASSERT_EQ(4, counterCall1);
    ASSERT_EQ(4, counterCall2);
}

 
int main(int _argc, char **_argv)  {
    testing::InitGoogleTest(&_argc, _argv);
    return RUN_ALL_TESTS();
}