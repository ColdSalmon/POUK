


      Node* insert(Node *head,int data)
      {
        if(head == NULL) head = new Node(data);
        else{
            Node *nextNode = head;
            while(nextNode->next)
            {
                nextNode = nextNode->next;
            }
            nextNode->next = new Node(data);
        }
        return head;
      }

