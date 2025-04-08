#include<vector>
#include<iostream>
using namespace std;
 
struct notebook
{
    int speed;
    int ram;
    int hdd;
    int cost;
};
 
int main()
{
    vector<notebook> data;
    int n;
    int min_cost = 1001;
    //int max_speed = 0, max_ram = 0, max_hdd = 0, max_cost = 0;
    int ans = 1;
    cin >> n;
    
    for(int i=0; i < n; i++)
    {
        notebook nb;
        cin >> nb.speed >> nb.ram >> nb.hdd >>nb.cost;
        data.push_back(nb);
    }
    
    for(int i=0; i < n; i++)
    {
        if((data[i].cost < min_cost))
        {
            bool check = true;
            for(int j=0; j < n; j++)
            {
                if(i!=j)
                {
                    if((data[i].speed < data[j].speed) && (data[i].ram < data[j].ram) && (data[i].hdd < data[j].hdd))
                    {
                        check = false;
                        break;
                    }
                }
            }
            if(check)
            {
                min_cost = data[i].cost;
                ans = i+1;
            }
        }
    }
    cout << ans;
    
    
    
    return 0;
}
