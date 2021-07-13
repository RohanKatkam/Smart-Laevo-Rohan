#ifndef HELLO_H_
#define HELLO_H_

#include <iostream>

using namespace std;

#define kt 2

int ff = 5;

int foo(int x);
void barh(int v);

void barh(int f){
    cout << "f is : " << f << endl;
}

int foo(int x) {
    return x + 20;
}



#endif