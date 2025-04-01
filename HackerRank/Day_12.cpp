
#include<cmath>
class Student :  public Person{
	private:
		vector<int> testScores;  
	public:        
        /*	
        *   Class Constructor
        *   
        *   Parameters:
        *   firstName - A string denoting the Person's first name.
        *   lastName - A string denoting the Person's last name.
        *   id - An integer denoting the Person's ID number.
        *   scores - An array of integers denoting the Person's test scores.
        */
        // Write your constructor here
    Student(string firstName, string lastName, int id, vector<int>testScores) :Person(firstName, lastName, id)
    {
        this->testScores = testScores;
    }
    
    char calculate()
    {
        int sum = 0;
        int count = this->testScores.size();
        
        for(int i = 0; i < count; i++)
        {
            sum += testScores[i];
        }    
        
        float avg = ((float)sum / (!count ? 1 : count));
        char ans = 'T';
        if(avg < 40) ans = 'T';
        else if(avg < 55) ans = 'D';
        else if(avg < 70) ans = 'P';
        else if(avg < 80) ans = 'A';
        else if(avg < 90) ans = 'E';
        else if(avg <= 100) ans = 'O';
        
        return ans;
    }
    
        /*	
        *   Function Name: calculate
        *   Return: A character denoting the grade.
        */
        // Write your function here
};

