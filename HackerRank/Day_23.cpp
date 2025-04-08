

	void levelOrder(Node * root){
      //Write your code here
        queue<Node*> rt;
        if(root != NULL)
        {
            rt.push(root);
            
            while(!rt.empty())
            {
                Node * curr = rt.front();
                
                cout << curr->data << " ";
                
                if(curr->left != NULL) rt.push(curr->left);
                if(curr->right != NULL) rt.push(curr->right);
                
                rt.pop();
            }
            
        }
	}

