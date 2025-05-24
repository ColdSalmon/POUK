<<<<<<< HEAD
// Example program
#include <iostream>
//#include <string>
using namespace std;
 
int main()
{
  int n;
  cin >> n;
  for(int i = 0; i <= n * 2; i++)
  {
      for(int p = 0; p < abs(i - n); p++) cout << "  ";
      for(int j = 0; j < n - abs(i - n); j++) cout << j << " ";
      for(int j = n - abs(i - n); j >= 0; j--)
      {
          cout << j;
          if(j != 0) cout << " ";
      }
      cout<<endl;
  }
=======
// Example program
#include <iostream>
//#include <string>
using namespace std;
 
int main()
{
  int n;
  cin >> n;
  for(int i = 0; i <= n * 2; i++)
  {
      for(int p = 0; p < abs(i - n); p++) cout << "  ";
      for(int j = 0; j < n - abs(i - n); j++) cout << j << " ";
      for(int j = n - abs(i - n); j >= 0; j--)
      {
          cout << j;
          if(j != 0) cout << " ";
      }
      cout<<endl;
  }
>>>>>>> 52d73ea9f1c3c74c5c8262cc0bb8d0999369e505
}