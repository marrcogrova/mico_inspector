

#include <streamers.h>
#include <block.h>

#include <iostream>

#include <ctime>

#include <any>
#include <opencv2/opencv.hpp>

class callable{
public:
    void operator()(){
        std::cout << "I am callable" << std::endl;
    }
};


template<typename T>
void expander(T t) {
    if(typeid(t) == typeid(int)){
        std::cout << t << std::endl;
    }else if(typeid(t) == typeid(double)){
        std::cout << t*2.0 << std::endl;
    }else{

    }
}

template <>
void expander(callable t) {
    t();
}

template<typename... Ts> 
void func(Ts... args){
    ((void) expander(std::forward<Ts>(args)), ...);
}

int main(){
    callable a;
    func(1, 0.2, 2,a); // Ts... args expand to int E1, double E2, const char* E3
                    // &args... expands to &E1, &E2, &E3
                    // Us... pargs expand to int* E1, double* E2, const char** E3
}