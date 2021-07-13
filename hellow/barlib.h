#ifndef BARLIB_H_
#define BARLIB_H_

#include <iostream>

using namespace std;
// void bar(int k);

// void foofoo(float f);

int bar(int k){
   return k + 100; 
}

void foofoo(float f){
    cout << "foo foo: " << f << "," << bar(42) << endl;
}

#endif
