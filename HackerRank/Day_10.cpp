#include <bits/stdc++.h>

using namespace std;

string ltrim(const string &);
string rtrim(const string &);



int main()
{
    string n_temp;
    getline(cin, n_temp);
    
    int maxcount = 0;
    int count = 0;
    int n = stoi(ltrim(rtrim(n_temp)));
    
    while(n>0)
    {
        if((n % 2) == 1) count += 1;
        else
        {
            if(count > maxcount) maxcount = count;
            count = 0;
        }
        n /= 2;
    }
    if(count > maxcount) maxcount = count;
    cout<<maxcount<<endl;
    return 0;
}

string ltrim(const string &str) {
    string s(str);

    s.erase(
        s.begin(),
        find_if(s.begin(), s.end(), not1(ptr_fun<int, int>(isspace)))
    );

    return s;
}

string rtrim(const string &str) {
    string s(str);

    s.erase(
        find_if(s.rbegin(), s.rend(), not1(ptr_fun<int, int>(isspace))).base(),
        s.end()
    );

    return s;
}
