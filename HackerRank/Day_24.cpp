
 
          Node* removeDuplicates(Node *head)
          {
            Node *curr = head;
            while(curr != NULL)
            {
                while((curr->next != NULL) && (curr->data == curr->next->data))
                {
                    curr->next = curr->next->next;
                }
                curr = curr->next;
            }
            return head;
          }

