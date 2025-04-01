
#include <iostream>
#include <stack>
#include <queue>
using namespace std;

class Solution {
    //Write your code here
    stack<char> st;
    queue<char> que;
    
    public:
    
    void pushCharacter(char ch)
    {
        st.push(ch);
    }
    
    void enqueueCharacter(char ch) 
    {
        que.push(ch);
    }
    
    char popCharacter()
    {
        char ret = st.top();
        st.pop();
        return ret;
    }
    
    char dequeueCharacter()
    {
        char ret = que.front();
        que.pop();
        return ret;
    }
};

