
#include <bits/stdc++.h>

using namespace std;



int main()
{
    string S;
    getline(cin, S);
    try {
        int ans = stoi(S);
        cout << ans << endl;
    } 
    catch (...) {
        cout << "Bad String" << endl;
    }
    
    return 0;
}
