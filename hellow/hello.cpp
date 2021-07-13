#include <iostream>
#include "hello.h"
#include <string.h>
#include "barlib.h"

using namespace std;

int main(){
    cout << "Something: " << foo(ff) << endl;
    string user;
    user  = "Roban: " + std::to_string(foo(ff));
    cout << "This is the ans= " << user << endl;
    barh(foo(200));
    int t = bar(2000);
    cout << "bar: " << t << endl;

    foofoo((float) ff);        
}