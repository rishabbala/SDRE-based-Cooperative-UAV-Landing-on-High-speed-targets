#!/bin/bash

# Dependencies: gnome, sshpass, ssh

for i in `seq 1 5`;
        do
  
gnome-terminal --tab  --command ' sshpass -p "odroid" ssh odroid@10.1.1.161'
                         
	done
