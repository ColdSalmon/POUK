<<<<<<< HEAD
#include<iostream>
using namespace std;
 
int main()
{
    int n;
    cin >> n;
    for(int i = 0; i < n; i++)
    {
        if(i % 2) cout << n - (i / 2) << " ";
        else cout << (i / 2) + 1 << " ";
    }
    return 0;
=======
#include<iostream>
using namespace std;
 
int main()
{
    int n;
    cin >> n;
    for(int i = 0; i < n; i++)
    {
        if(i % 2) cout << n - (i / 2) << " ";
        else cout << (i / 2) + 1 << " ";
    }
    return 0;
>>>>>>> 52d73ea9f1c3c74c5c8262cc0bb8d0999369e505
}