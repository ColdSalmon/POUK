

		int getHeight(Node* root){
            if(root == NULL) return -1;
            return 1 + max(getHeight(root->left), getHeight(root->right));
          //Write your code here
        }

